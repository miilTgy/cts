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

从 `partition_tree.root` 开始构造，初始 `external_target = problem.source.loc`。最终 `TopoTree.root` 必须覆盖全部 sinks。

## Node Kind 分类

`TopoNode::kind` 必须用于区分 vtree 输出中的 node 类别：

- `Sink`：sink leaf node。
- `ClusterInternal`：partreer 在 cluster 内部 pairing level 生成的 internal tapping node。
- `ClusterAccess`：partreer 在 access tree 中生成的 access node，包括每个 leaf partition 对外返回的 `cluster_root`。
- `Global`：treer 在 internal partition / root partition 层级 merge child topology roots 时新建的 node。

## Local Cluster 接入规则

每个 leaf partition 的 local topology 由 `partreer` 完成。treer 只接收其 `cluster_root`：

```text
partition leaf node -> partreer local tree -> cluster_root topo node
```

一个 partition 内部可以有多个 access sub-roots，但对 treer 外部只暴露一个 `cluster_root`。

append local tree 到 global tree 时必须：
- 对 local node ids 做 remap。
- 保留 local node 的 `kind`、`sink_indices`、`bbox`、delay/skew 字段。
- 修正 remap 后的 `parent/left/right`。
- 将 remapped `local.cluster_root` 或 `local.root` 记录为该 partition leaf 的 representative。

## Global Merge 规则

`merge_child_roots(child_roots, external_target)` 用于连接 internal partition 的 child topology roots。

- 若 `child_roots.size() == 1`，直接返回该 root。
- 若 `child_roots.size() == 2`，创建一个 binary parent node 连接二者，`kind = Global`。
- 若 `child_roots.size() > 2`，按相对 `external_target` 的方向排序，递归构造 balanced binary tree；所有新建 parent nodes 都设置 `kind = Global`。
- parent node 坐标取 child roots 的 Manhattan midpoint / centroid，并轻微向 `external_target` 偏移。
- parent node 坐标必须在 die boundary 内。
- 每次 merge 后必须更新 `left/right/parent/sink_indices/bbox/delay range/skew/kind`。

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
- `GLOBAL`

要求：
- `ROOT` 是最终 topology root node id。
- `NODE` 行覆盖所有 topology nodes，包括 sink leaf 和 internal nodes。
- `LEAF` 行只列出 sink leaf nodes。
- `EDGE` 行列出所有 parent-child topology edges。
- sink leaf 的 `left/right = -1`，`is_sink = 1`，`node_kind = SINK`。
- internal node 的 `is_sink = 0`，`sink_index = -1`。
- 每个 node 必须有正确的 `node_kind`，用于可视化区分 sink、cluster internal nodes、cluster 接入点和 global nodes。
- `TREE_VALID = 1` 表示 root 覆盖全部 sinks 且所有 node parent/child 关系合法。

## Debug 要求

提供 debug 开关：

```cpp
void treer::debug_enable(bool enable);
void treer::debug_output(const common::TopoTree& tree);
```

debug 关闭时不打印 log。debug 开启时输出：
- partition node 到 topology root 的 mapping；
- 每个 leaf partition 调用 partreer 的 sink ids / cluster root；
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
- 所有 node id 必须等于其在 `nodes` vector 中的 index。
- 所有 node 必须从 root 可达。
- 除 root 外，每个 node 必须有唯一 parent。
- internal node 必须有两个不同 child。
- 实现必须 deterministic。
- 不允许产生 cycle。

## Non-Goals

不要在本模块做 partition。不要实现 partreer 内部 pair-first 细节。不要插 buffer。不要做 bottom-up DME。不要做 top-down routing。不要输出 HW3 final solution 文件。