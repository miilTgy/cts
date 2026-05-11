# Treer Prompt

实现 `treer.h` 和 `treer.cc`。本模块接收 `parser` 得到的 `common::Problem` 和 `partitioner` 得到的 `common::PartitionTree`，构造全局 CTS binary topology。不要实现 bu、td、writer、buffer insertion 或 final rectilinear routing。

## Namespace 和 API

- 所有 treer 相关函数放在 `namespace treer`。
- 复用 `common.h` 中的 `Problem`、`PartitionTree`、`TopoNode` / `TopoTree`；必要时只在 `namespace common` 中补充 tree 结果结构。
- 对外暴露主 API：

```cpp
common::TopoTree treer::build(const common::Problem& problem,
                             const common::PartitionTree& partition_tree,
                             const std::string& sample_name);
```

## 模块职责

treer 只负责把 partition hierarchy 转成 topology hierarchy：

1. 对每个 leaf partition，调用 `partreer` 构造 cluster 内部 local topology。
2. 每个 leaf partition 对外只对应一个 `cluster_root`。
3. internal partition node 不直接等同于 topology node；它只负责把 child partition 的 topology roots merge 成 global topology。
4. 建立 `partition_node_id -> topo_root_node_id` 的 mapping。
5. 最终输出一个包含所有 sinks 的 binary `TopoTree`。

不要把 `PartitionNode` 当成 `TopoNode`。`PartitionNode` 是空间分区节点；`TopoNode` 是 CTS topology 节点。

## 输入假设

`partition_tree` 来自 `partitioner::build(problem)`，并且保留层次结构：root 包含全部 sinks，internal node 包含 child partition ids，leaf node 包含 cluster sink indices。leaf cluster / outlier 的信息由 partitioner 提供。

## 构造流程

递归处理 partition tree：

```text
build_partition_topology(pid, external_target):
    pnode = partition_tree.nodes[pid]

    if pnode is leaf cluster or outlier:
        local = partreer::build(problem, pnode.sink_indices, external_target)
        append local tree into global TopoTree with node id remap
        rep[pid] = remapped local.cluster_root or remapped local.root
        return rep[pid]

    child_roots = []
    for child pid in pnode.children:
        child_external_target = pnode.centroid or external_target
        child_roots.push_back(build_partition_topology(child, child_external_target))

    root = merge_child_roots(child_roots, external_target)
    rep[pid] = root
    return root
```

在所有 leaf partition 的 cluster topology 都生成完毕后，收集所有 `ClusterTop` roots，执行 source-aware 几何 global topology 重建（见下方 "Source-aware 几何 Global Topology 修正" 章节）。最终 global topology 必须以后面的 Source-aware 几何 Global Topology 修正为准；递归 partition merge 只能作为遍历/临时 fallback，不得作为最终 global tree 保留。最终 `TopoTree.root` 必须覆盖全部 sinks。

## Node Kind 分类

`TopoNode::kind` 必须区分：

- `Sink`：sink leaf node。
- `ClusterInternal`：cluster 内部 pairing / tapping node。
- `ClusterAccess`：真实 cluster access tap point；不是 merge node。
- `ClusterBridge`：partreer 合并多个 access tap 时创建的二叉 bridge node。
- `ClusterTop`：leaf partition 对外唯一 root / representative。
- `Global`：treer merge child partition roots 时创建的 node。

禁止 `ClusterAccess` 与 `ClusterAccess` 互为 parent-child。多个 access tap 必须经 `ClusterBridge` 合并，最顶层由 `ClusterTop` 对 treer 暴露。

## Local Cluster 接入规则

每个 leaf partition 的 local topology 由 `partreer` 完成。treer 只接收 `local.cluster_root`，且它必须是 `ClusterTop`：

```text
leaf partition -> partreer local tree -> ClusterTop cluster_root
```

access tree 合法形态：

```text
ClusterTop
├── ClusterAccess / local subtree
└── ClusterAccess / local subtree

ClusterTop
└── ClusterBridge
    ├── ClusterAccess / local subtree
    └── ClusterAccess / local subtree
```

规则：
- `ClusterAccess` 只表示真实接入点。
- access 数量 = 1 时，`ClusterTop` 一元包住它；= 2 时，`ClusterTop` 直接连接两个 `ClusterAccess`。
- access 数量 > 2 时，才用 `ClusterBridge` 组成二叉树并接到 `ClusterTop`。
- treer 不 fallback 到普通 `local.root`，除非它已标记为 `ClusterTop`。

append local tree 到 global tree 时必须 remap node ids，并保留 `kind/sink_indices/bbox/delay/skew/parent/left/right`。

append 后立即执行 topology canonicalize：反复处理仅有 1 个 child 的 node。若 parent 与 child 在下列偏序中可比较，则较大者吸收较小者：`Source > Global`，`ClusterTop > ClusterBridge`，`ClusterBridge > ClusterAccess`，`ClusterAccess > ClusterInternal`，`Sink > ClusterAccess`，`Sink > ClusterInternal`。吸收时保留较大者的 `kind/loc`，接管较小者的 child；若较大者是 child，则用 child 替换 parent 接到 grandparent。偏序外的 pair 视为 incomparable，不强行吸收。随后压缩/重编号 nodes，并重算 parent/child、bbox、sink_indices、delay/skew。

## Global Merge 规则

`merge_child_roots(child_roots, external_target)` 用于连接 internal partition 的 child topology roots。

注意：最终 global topology 以后文 Source-aware 几何 Global Topology 修正章节为准。该规则只用于递归处理 partition hierarchy 时的临时 merge / fallback，最终写出前必须重新收集所有 `ClusterTop` roots 并重建 source-aware global tree。

- 若 `child_roots.size() == 1`，直接返回该 root。
- 若 `child_roots.size() == 2`，创建一个 binary parent node 连接二者，`kind = Global`。
- 若当前是 partition root，且新建 root `Global` 与 `problem.source.loc` 重合，或 source 只有一个 child 且该 child 是 `Global`，则由 source 吸收该 `Global`：不输出该 root global node，source 直接连接它原来的 children，`TopoTree.root = -1` 或使用显式 `SOURCE` root；该规则不限于一元 node，binary root global 也必须吸收。
- 若 `child_roots.size() > 2`，按相对 `external_target` 的方向排序，递归构造 balanced binary tree；所有新建 parent nodes 都设置 `kind = Global`。
- parent node 坐标取 child roots 的 Manhattan midpoint / centroid，并轻微向 `external_target` 偏移。
- parent node 坐标必须在 die boundary 内。
- 每次 merge 后必须更新 `left/right/parent/sink_indices/bbox/delay range/skew/kind`；treer 只能新建 `Global`，不能新建 `ClusterAccess/ClusterBridge/ClusterTop`。

global merge 只构造 abstract topology edge，不生成最终 rectilinear route。

## Delay / Skew 维护

每个 internal `TopoNode` 维护左右子树所有 sinks 到当前 node 的 delay range / skew。

对 internal node `v`：

```text
left_min_delay_to_node  = left.subtree_min_delay_to_node  + manhattan(v.loc, left.loc)
left_max_delay_to_node  = left.subtree_max_delay_to_node  + manhattan(v.loc, left.loc)
left_skew_to_node       = left_max_delay_to_node - left_min_delay_to_node

right_min_delay_to_node = right.subtree_min_delay_to_node + manhattan(v.loc, right.loc)
right_max_delay_to_node = right.subtree_max_delay_to_node + manhattan(v.loc, right.loc)
right_skew_to_node      = right_max_delay_to_node - right_min_delay_to_node

subtree_min_delay_to_node = min(left_min_delay_to_node, right_min_delay_to_node)
subtree_max_delay_to_node = max(left_max_delay_to_node, right_max_delay_to_node)
subtree_skew_to_node      = subtree_max_delay_to_node - subtree_min_delay_to_node
```

leaf sink node 的 subtree delay / skew 全为 0。这里暂不考虑 buffer delay。

## Source-aware 几何 Global Topology 修正

在所有 leaf partition 都已经转换成 cluster representative 之后，需要从所有 `ClusterTop` root 重新构造 global topology。不要直接保留递归 partition hierarchy 作为最终 global tree。

global topology 应该按照 source-aware 的几何访问顺序连接 clusters，使 global trunk 以干净的空间扫描顺序访问各个 cluster，减少大跨度交叉连接。该步骤只修改 global topology，不修改 cluster 内部 topology。

### 输入

- `source.loc`
- 所有 leaf cluster representative：`ClusterTop` roots
- 每个 cluster 的 `bbox` 和/或几何中心

每个 cluster 使用 `bbox_center` 作为排序点。优先使用 `bbox_center`，不要优先使用 `ClusterTop.loc`，因为 `ClusterTop.loc` 可能已经被 access-side placement 偏移过，不一定代表 cluster 的真实几何中心。

### Source-aware 排序

先计算所有 cluster bbox 的 union bbox。然后比较 `source.loc` 和该 union bbox 的相对位置，决定扫描方向：

- 如果 source 在 cluster union bbox 左侧：按 `x` 升序排序，再按 `y` 降序排序。
- 如果 source 在 cluster union bbox 右侧：按 `x` 降序排序，再按 `y` 降序排序。
- 如果 source 在 cluster union bbox 下方：按 `y` 升序排序，再按 `x` 升序排序。
- 如果 source 在 cluster union bbox 上方：按 `y` 降序排序，再按 `x` 升序排序。
- 如果 source 在 union bbox 内部或附近，则选择 spread 更大的轴作为 primary sweep axis：
  - `width >= height`：按 `x` 升序排序。
  - 否则：按 `y` 升序排序。

当 source 在 union bbox 内部或附近时，排序要保证 source 被视作扫描序列中的一个参考分割点。例如 `width >= height` 且存在：

```text
top1.x < top2.x < source.x < top3.x
```

则 global topology 的几何访问顺序应该体现为：

```text
top1 -> top2 -> source -> top3
```

也就是说，source 左侧的 cluster 应该先按从远到近或从左到右的顺序接近 source，source 右侧的 cluster 再继续向右延伸。不要构造出需要跨过 source 后再回到另一侧的 global topology。

### Ordered chain-like binary tree

假设排序后的 cluster roots 为：

```text
C0, C1, C2, ..., Cn-1
```

用 ordered chain 的方式构造 binary global tree，不要用 balanced recursive merge：

```text
root = Cn-1
for i = n-2 downto 0:
    root = new Global(left = Ci, right = root)
```

如果 source 落在 cluster union bbox 内部或附近，并且排序轴上 source 位于若干 cluster 中间，则 ordered chain 需要尊重 source 的分割作用。不要把 source 强行接到整条 chain 的端点，而应该分别构造 source 两侧的 ordered chain，再用最靠近 source 的 `Global` 连接左右两侧 chain。

具体策略：

- 按 primary axis 将 clusters 分成 source 负方向一侧和 source 正方向一侧。
- 负方向一侧的 chain 应该从远离 source 的 cluster 开始，逐步接近 source。
- 正方向一侧的 chain 应该从靠近 source 的 cluster 开始，逐步远离 source。
- 新建一个靠近 `source.loc` 的 `Global` root，左右 child 分别连接两侧 chain root；如果某一侧为空，则该 root 直接连接非空侧 chain。

例如 `width >= height` 且：

```text
top1.x < top2.x < source.x < top3.x < top4.x
```

则几何访问关系应体现为：

```text
top1 -> top2 -> source -> top3 -> top4
```

实现上可以构造为：

```text
        Gsrc
       /    \
  left_chain right_chain
```

其中 `left_chain` 对应 `top1 -> top2`，其 root 靠近 source；`right_chain` 对应 `top3 -> top4`，其 root 也靠近 source。`Gsrc` 是最终连接 source 的 global root。

### Global node 位置

Global node 的位置应保持 ordered chain 的相对几何顺序正确。不要强制所有 global nodes 都放在同一条 source-aligned trunk 上。

placement 策略：

- 优先保证 treer 阶段的 parent-child 直连线段不产生明显交叉。
- 在不引入 treer 阶段线交叉的前提下，global nodes 尽量靠近 source-aligned trunk。
- 对于 left/right sweep，可将 global node 放在对应 cluster `bbox_center.x` 附近，同时 `y` 尽量靠近 `source.y`，但允许为了避免交叉而上下偏移。
- 对于 up/down sweep，可将 global node 放在对应 cluster `bbox_center.y` 附近，同时 `x` 尽量靠近 `source.x`，但允许为了避免交叉而左右偏移。
- 如果 source 在 union bbox 内部或附近，global node 应该围绕 source 的相对位置布置：source 左侧/下侧的 global nodes 保持在对应侧，source 右侧/上侧的 global nodes 保持在对应侧，不要为了对齐 trunk 而跨到 source 的另一侧。

如有需要，将坐标 clamp 到 die/grid 边界内。如果多个 cluster 处在同一个 x/y band 内，需要保持原排序，但可以轻微错开 global node 位置，避免多个 global node 坐标完全相同。

### 重要约束

- 只重建 `ClusterTop` roots 之间的 global topology。
- 不要修改 cluster-internal nodes 或 edges。
- 新建的 global internal nodes 类型必须是 `Global`。
- 结果 topology 必须仍然是合法 binary tree。
- 该 fix 应该在 cluster topology generation 之后、写出 `sample<k>_vtree.txt` 之前执行。

## 输出 tree/sample<k>_vtree.txt

无论 debug 是否开启，都输出 topology 到：

```text
tree/sample<k>_vtree.txt
```

`sample<k>` 从输入文件名推导。若 `tree/` 目录不存在，需要创建目录。

输出格式必须兼容 `sample1_vtree.txt` 风格，并在 `NODE` 行最后额外输出 `node_kind`：

```text
TREE_VALID <0/1>
ROOT <root_node_id>
SOURCE <x> <y>
NUM_NODES <num_nodes>
NUM_SINKS <num_sinks>

NODE <id> <parent> <left> <right> <is_sink> <sink_index> <sink_count> <x> <y> <bbox_lx> <bbox_ly> <bbox_ux> <bbox_uy> <die_lx> <die_ly> <die_ux> <die_uy> <subtree_skew_to_node> <node_kind>
...

LEAF <node_id> <sink_index> <sink_id> <x> <y>
...

EDGE <parent_node_id> <child_node_id>
...
```

`node_kind` 必须输出为以下字符串之一：
- `SINK`
- `CLUSTER_INTERNAL`
- `CLUSTER_ACCESS`
- `CLUSTER_TOP`
- `CLUSTER_BRIDGE`
- `GLOBAL`

要求：
- `ROOT` 是最终 topology root node id。
- 默认情况下 source 不作为 `TopoNode`；`TopoTree.root` 是最靠近 source 的最终 global root，输出 `EDGE SRC <root_node_id>` 表示 source 连接该 root。
- 只有当 root `Global` 与 `source.loc` 完全重合，或确实执行 source 吸收时，才输出 `ROOT -1`，`SOURCE` 为 source 坐标，`EDGE SRC <child_node_id>` 表示 source 直接连接被吸收 global 的原 children；包括 source 只有一个 `Global` child 的情况。source 吸收不能破坏 binary topology 约束，除非输出格式显式允许 `SRC` 连接被吸收 root 的两个原 children。
- `NODE` 行覆盖所有 topology nodes，包括 sink leaf 和 internal nodes。
- `LEAF` 行只列出 sink leaf nodes。
- `EDGE` 行列出所有 parent-child topology edges。
- sink leaf 的 `left/right = -1`，`is_sink = 1`，`node_kind = SINK`。
- internal node 的 `is_sink = 0`，`sink_index = -1`。
- 每个 node 必须有正确的 `node_kind`；`CLUSTER_ACCESS` 不得互连；leaf representative 必须是 `CLUSTER_TOP`。

## Debug 要求

提供 debug 开关：

```cpp
void treer::debug_enable(bool enable);
void treer::debug_output(const common::TopoTree& tree);
```

debug 关闭时不打印 log。debug 开启时输出：
- partition node 到 topology root 的 mapping；
- 每个 leaf partition 的 sink ids / `ClusterTop` root；
- internal partition 的 child roots；
- 每次 global merge 的 left/right/root node id、坐标和 `node_kind`；
- 每个 node 的 `node_kind` 分类统计；
- 每个 internal node 的 left/right/subtree skew；
- 最终 root、node count、edge count；
- `tree/sample<k>_vtree.txt` 输出路径。

## Main 接入要求

修改 `main.cc`：

1. parse input；
2. 调用 `partitioner::build(problem)`；
3. 调用 `treer::build(problem, partition_tree, sample_name)`；
4. 输出 `tree/sample<k>_vtree.txt`；
5. 暂时继续注释掉 `bu`、`td`、`writer`。

## Robustness 要求

- 处理 zero sinks / one sink。
- local tree append 到 global tree 时必须重映射 node ids。
- `partition_node_id -> topo_root_node_id` mapping 必须完整。
- parent/left/right 关系必须一致。
- root 的 `sink_indices` 必须覆盖所有 sinks。
- 若 root `Global` 与 source 重合，或 source 只有一个 `Global` child，必须执行 source 吸收；吸收后 source 的直接 children 的 sink coverage 合并后仍须覆盖所有 sinks。
- 所有 node id 必须等于其在 `nodes` vector 中的 index。
- 所有 node 必须从 root 可达。
- 除 root 外，每个 node 必须有唯一 parent。
- canonicalize 后除 source/leaf/single-sink `ClusterTop` 特例外，不应存在可比较的一元 parent；不可比较的一元关系必须在 debug 中标出。
- 实现必须 deterministic。
- 不允许产生 cycle。
- leaf representative 必须是 `ClusterTop`；`ClusterAccess` 不得互连；`ClusterBridge` 只允许在 leaf access tree 内部。

## Non-Goals

不要在本模块做 partition。不要实现 partreer 内部 pair-first 细节。不要插 buffer。不要做 bottom-up DME。不要做 top-down routing。不要输出 HW3 final solution 文件。