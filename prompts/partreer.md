# Partreer Prompt

实现 `partreer.h` 和 `partreer.cc`。本模块接收顶层 `treer` 传入的一个 leaf cluster，在该 cluster 内部构造 local binary topology，并返回该 cluster 对外的一个 `cluster_root`。不要实现 partitioner、global treer、bu、td、writer、buffer insertion 或 final routing。

## 模块职责边界

输入：
- `const common::Problem& problem`
- 一个 cluster 内的 sink indices
- 该 cluster 的外部接入参考点 `external_target`，可来自 source 或 parent partition root

输出：
- 一个 local topology subtree
- 该 subtree 对外只有一个 `cluster_root`
- subtree 内部允许多个 access sub-roots，但必须通过 binary topology 汇聚到 `cluster_root`

本模块只负责 cluster 内部 topology construction。不要生成最终 rectilinear routes，不要做 skew repair，不要插 buffer。

## 数据结构要求

在 `common.h` 的 `namespace common` 中新增或复用 topology node 结构。每个 local tree node 至少维护：

```cpp
struct TopoNode {
    int id;
    common::Point loc;

    int parent = -1;
    int left = -1;
    int right = -1;

    bool is_sink = false;
    int sink_index = -1;

    std::vector<int> sink_indices;
    common::BBox bbox;

    int left_min_delay_to_node = 0;
    int left_max_delay_to_node = 0;
    int left_skew_to_node = 0;

    int right_min_delay_to_node = 0;
    int right_max_delay_to_node = 0;
    int right_skew_to_node = 0;

    int subtree_min_delay_to_node = 0;
    int subtree_max_delay_to_node = 0;
    int subtree_skew_to_node = 0;

    enum class NodeKind {
        Sink,
        ClusterInternal,
        ClusterAccess,
        ClusterBridge,
        ClusterTop,
        Global
    } kind = NodeKind::ClusterInternal;
};
```

含义：
- `left_*_to_node` 表示 left subtree 内所有 sinks 到当前 node 的 delay range / skew。
- `right_*_to_node` 表示 right subtree 内所有 sinks 到当前 node 的 delay range / skew。
- `subtree_*_to_node` 表示当前 node 覆盖的所有 sinks 到当前 node 的 delay range / skew。
- leaf sink node 的 `subtree_min_delay_to_node = subtree_max_delay_to_node = subtree_skew_to_node = 0`。
- internal node 的 delay 由 child subtree delay 加上当前 node 到 child node 的 Manhattan distance 得到。
- `kind` 分类：`Sink` 是 sink leaf；`ClusterInternal` 是 cluster 内部 pairing/tapping node；`ClusterAccess` 是真实 access tap；`ClusterBridge` 是 access taps 的二叉 merge node；`ClusterTop` 是对 treer 暴露的唯一 `cluster_root`；`Global` 只由 treer 生成。

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

这里的 delay 只按 topology embedding 中 node-to-node 的 Manhattan distance 估计，不考虑 buffer delay。

## Local Topology 总策略

每个 cluster 对外只返回一个 `ClusterTop cluster_root`，内部可先形成多个 access sub-roots。

使用 Pair-first Access Tree：

1. 将 cluster 内所有 sinks 转成 active nodes，sink node 设置 `kind = Sink`。
2. 重复执行 pairing level。
3. 每一 level 中只连接高置信度的 CONNECTABLE pair。
4. 每个 selected pair 生成一个 internal tapping node，`kind = ClusterInternal`，并连接为 binary parent。
5. 未匹配 node 直接 carry 到下一 level。
6. 当 active node 数量 `<= target_access_points`，或当前 level 找不到合法 pair 时，停止 pairing。
7. 剩余 active nodes 作为真实 access sub-roots，必要时标为 `ClusterAccess`。
8. access 数量为 1 时直接把该 access root 改为 `ClusterTop`，不新增一元 parent；为 2 时 `ClusterTop` 直接连两个 access；大于 2 时才用 `ClusterBridge` 合并后接到 `ClusterTop`。

建议常量：

```text
target_access_points = 2 或 3
k_nearest = 3
max_pair_levels = 8
```

如果 pairing 停止后 active nodes 仍然多于 `target_access_points`，继续用最低 cost 的合法 pair 压缩；若仍无法压缩，则直接对 active nodes 构造 balanced access tree。

## Pair Candidate 生成规则

每一 level 中，不要枚举并强行连接所有 pair。只从局部候选中选择：

- 两个 nodes 在 x-sort 或 y-sort 中相邻/近邻；或
- 两个 nodes 互为 k-nearest candidate；或
- 两个 nodes 的 Manhattan distance 明显较小。

pair cost 越小越优先：

```text
cost = manhattan(a.loc, b.loc)
     + bbox_area_penalty
     + interleave_penalty
     + skew_penalty
```

其中：
- `bbox_area_penalty` 惩罚合并后 bbox 过大。
- `interleave_penalty` 惩罚跨过很多其他 active nodes 的 pair。
- `skew_penalty` 惩罚 pair 后 `subtree_skew_to_node` 过大。

实现可以先用简化 cost，但必须保证 deterministic。

## CONNECTABLE 判断

`CONNECTABLE(a, b)` 为 true 当且仅当：

1. `a` 和 `b` 在当前 level 中都尚未匹配。
2. `a` 和 `b` 是 Pair Candidate。
3. candidate segment `AB` 与 cluster 内已有 topology segments 不存在非法相交。
4. 连接后不会产生 cycle。
5. 连接后生成的 parent node 仍在 die boundary 内。

Pairing 阶段可以先用 candidate segment `AB` 做快速筛选；创建 tapping node `p` 后，必须分别检查实际加入的 `p-a` 和 `p-b` 两条 topology segments 是否与已有 segments 非法相交。

## Segment 非法相交判断

判断 candidate segment `AB` 是否能加入：遍历 cluster 内所有已有 topology segments。若 existing segment 与 `AB` 的交点只发生在 `A` 或 `B`，且该点是同一个 topology node，则允许；除此之外，只要 `AB` 与任意 existing segment 存在相交，就判定为非法，不能加入。

非法相交包括：
- 非端点穿越；
- T-junction，即一个 segment 的端点落在另一条 segment 的内部；
- candidate segment 穿过非自身端点 node；
- 共线重叠。

### 两点式直线交点公式

设：

```text
A = (x1, y1), B = (x2, y2)
C = (x3, y3), D = (x4, y4)
```

两条无限直线 `AB` 和 `CD` 的交点分母：

```text
den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
```

若 `den != 0`，两直线不平行，交点为：

```text
px = ((x1*y2 - y1*x2) * (x3 - x4) - (x1 - x2) * (x3*y4 - y3*x4)) / den
py = ((x1*y2 - y1*x2) * (y3 - y4) - (y1 - y2) * (x3*y4 - y3*x4)) / den
```

然后判断交点是否同时落在两条线段的闭区间 bounding box 内：

```text
min(x1,x2) <= px <= max(x1,x2)
min(y1,y2) <= py <= max(y1,y2)
min(x3,x4) <= px <= max(x3,x4)
min(y3,y4) <= py <= max(y3,y4)
```

如果都满足，则两条有限线段相交。

特殊情况：
- `den == 0` 表示两直线平行或共线。
- 平行但不共线：不相交。
- 共线时，若两线段在 x/y 投影上有重叠，则认为相交。
- candidate segment 与 existing segment 只共享同一个 topology node 端点时，不算非法相交。

实现时可用 `double` 计算 `px/py`，判断闭区间时加入 `eps = 1e-9`。由于输入坐标是整数，也可以用整数叉积辅助处理 `den == 0` 和共线判断。

## Tapping Node 位置

每个 selected pair `(a, b)` 生成一个 parent tapping node `p`：

```text
p.x = round((a.loc.x + b.loc.x) / 2)
p.y = round((a.loc.y + b.loc.y) / 2)
```

然后根据 `external_target` 做轻微 source-aware bias：

```text
if external_target.x > p.x: p.x += offset
if external_target.x < p.x: p.x -= offset
if external_target.y > p.y: p.y += offset
if external_target.y < p.y: p.y -= offset
```

`offset` 取 0 或 1 即可。若 bias 后导致非法相交或出界，则退回 midpoint。

parent node 必须更新：
- `left/right`
- `parent`
- `sink_indices`
- `bbox`
- `left/right/subtree delay range`
- `left/right/subtree skew`
- `kind`

## Access Tree 构造

pairing 结束后的 active nodes 是 access sub-roots。构造规则：

- 若 active node 还不是真实 access tap，可按需要新建/标记 `ClusterAccess` wrapper。
- active size = 1：直接把该 access root 改为 `ClusterTop`，作为 `cluster_root`，不新增一元 parent。
- active size = 2：创建 `ClusterTop` 直接连接两个 `ClusterAccess`。
- active size > 2：用 balanced binary tree 合并；中间 parent 全部为 `ClusterBridge`，最顶层再创建 `ClusterTop`。
- 禁止 `ClusterAccess -> ClusterAccess`。`ClusterAccess` 只表示真实 tap，不承担 merge 语义。
- access tree 完成后执行 canonicalize：反复处理仅有 1 个 child 的 node。若 parent/child 在偏序中可比较，则较大者吸收较小者；partreer 内使用 `ClusterTop > ClusterBridge > ClusterAccess > ClusterInternal`，且 `Sink > ClusterAccess`、`Sink > ClusterInternal`。吸收时保留较大者 `kind/loc`，接管较小者 child；若较大者是 child，则用 child 替换 parent 接到 grandparent。不可比较 pair 不强行吸收，只在 debug 标出。
- 若 `ClusterAccess` 只有一个 `ClusterInternal` child，则吞并该 child：`ClusterAccess` 保持坐标/kind，接管 child 的 `left/right` 并更新 grandchildren 的 `parent`。

构造 access tree 仍必须遵守 CONNECTABLE 检查。若直接连接非法，允许调整 parent tapping node 坐标或改变 pairing 顺序。

最终返回的 `cluster_root.kind` 必须是 `ClusterTop`。

## Debug 输出要求

提供 debug 开关：

```cpp
void partreer::debug_enable(bool enable);
void partreer::debug_output(const common::TopoTree& tree);
```

Debug 开启时输出：
- cluster sink ids / indices；
- 每一 level 的 active nodes；
- candidate pairs 和 cost；
- 被选中的 CONNECTABLE pairs；
- 被 carry 的 nodes；
- 每个 internal node 的 left/right skew 和 subtree skew；
- node kind 分类统计；
- canonicalize 吸收记录和剩余不可比较的一元关系；
- 最终 `ClusterTop cluster_root`。

Debug 关闭时不打印 log。

## Non-Goals

不要在本模块处理 global partition hierarchy。不要连接不同 clusters。不要插 buffer。不要做 bottom-up DME merging segment。不要做 top-down routing。不要输出最终 solution 文件。