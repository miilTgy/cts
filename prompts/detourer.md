

# Detourer Prompt

请实现 `detourer` 阶段：`detourer` 接受 `router` 已经生成的合法 Manhattan polyline route，在不改变 topology、不改变 node loc、不重新 route 的前提下，通过在已有 edge polyline 上加入小绕格 detour 来平衡左右子树 skew。

`detourer` 的职责是 timing/skew repair，不是 detailed router。

---

## 1. 输入与输出

输入：

```cpp
common::Problem problem;
common::TopologyTree tree;
common::LocerResult loc_result;
common::RouterResult& route_result;    // mutable: detourer 原地修改 edge polyline
```

输出：

```cpp
common::DetourerResult result;         // debug record only, 不替代 route_result
```

**核心输出（side effect）**：
- `route_result.edge_debugs[i].polyline` 被原地替换为 detoured polyline
- 后续 writer 直接读取同一个 `route_result`，无需任何改动

**`DetourerResult` 仅用于调试**，记录每条 edge 的 before/after delay、插入的 detour 位置/层级/方向。不包含完整 polyline（已写回 `route_result`）。

输出：

```cpp
common::DetourerResult result;
```

核心输出：

```text
每条 topology edge 的 updated Manhattan polyline
每个 node 的 delay profile
每条 edge 插入的 detour 记录
```

要求：

- 不改变 topology。
- 不改变 node loc。
- 不改变 edge endpoints。
- 不重新选择 route path，只允许在 router 已有 polyline 的某些 segment 上插入局部 detour。
- 所有新增点仍必须是整数 grid 坐标。
- 所有 detour 后 polyline 仍必须是 Manhattan polyline。
- detour 后 route 仍必须合法：不能压到任何 node、edge、bbox obstacle 或其它实体，不能自交，不能和其它 route 非法交叉/overlap。

---

## 2. Delay model

`detourer` 需要根据 router 的 polyline 计算 delay。

对于每条 routed edge：

```text
edge_delay = polyline_total_manhattan_length
```

每个 node 维护：

```text
sink_delays_to_node[node] = list of delay from every sink in this node's subtree to this node
min_sink_delay_to_node
max_sink_delay_to_node
skew_to_node = max - min
```

叶子 sink：

```text
sink_delays_to_node[sink] = [0]
```

内部 node：

```text
sink_delays_to_node[node] = concat(
    for each child c:
        for d in sink_delays_to_node[c]:
            d + route_delay(edge node-c)
)
```

注意：

- 数据结构保留所有 sink delay，方便 debug。
- 但是 detour 平衡时只看每个 child subtree 的最大 delay。
- 对 binary tree，平衡目标是左右子树的 max delay 尽量相等。

---

## 3. Strict bottom-up detour order

`detourer` 必须严格 bottom-up 处理 topology。

处理顺序：

```text
sinks / leaves first
then internal/access
then bridge/top
total cluster subtree完成后
global nodes
source/root last
```

对每个 node，只有当所有 children 的 delay profile 和 incoming edge delay 都已经确定后，才能处理该 node。

对一个 binary node：

```text
left_worst  = max_sink_delay_to_node[left]  + edge_delay(node-left)
right_worst = max_sink_delay_to_node[right] + edge_delay(node-right)
```

只比较：

```text
abs(left_worst - right_worst)
```

不使用平均 delay，不使用所有 sink 的 full profile 做主优化目标。

需要给较短的一侧加 detour：

```text
if left_worst < right_worst:
    add detour on edge node-left or inside left subtree's selected balancing edge
else:
    add detour on edge node-right or inside right subtree's selected balancing edge
```

最简单且推荐的策略：优先在当前 node 到较短 child 的 edge 上加 detour。若该 edge 没有可加位置，再在该 child subtree 内按 edge length 从长到短寻找可加 detour 的 edge。

目标：

```text
shorter_side_worst + added_delay 尽量接近 longer_side_worst
```

允许略微 over-balance，但要加 penalty；优先选择不超过目标差值的 detour。

---

## 4. Detour unit：小绕格定义

一个 `k` 级小绕格表示在某个直线 segment 上加入一个局部 U-shape 绕行。

对水平 segment：

原始局部片段：

```text
(x0, y) -> (x1, y)
```

在中心位置附近插入向上或向下绕格，k=1 时：

```text
... -> (a, y) -> (a, y+dir*1) -> (b, y+dir*1) -> (b, y) -> ...
```

对垂直 segment：

```text
... -> (x, a) -> (x+dir*1, a) -> (x+dir*1, b) -> (x, b) -> ...
```

其中：

```text
k = detour depth
added_delay = 2 * k
```

因为 detour 往外绕 k 格，再绕回来 k 格，比原 segment 多出 `2k` 的 Manhattan length。

k=1：delay +2  
k=2：delay +4  
k=3：delay +6

---

## 5. Detour insertion principle

对需要加 delay 的一侧，选择 edge 和 segment 的原则：

1. 优先选择该侧中最长的 routed edge。
2. 在该 edge 内，优先从最长 segment 的中心附近开始尝试插入 detour。
3. 对同一 segment，优先尝试靠近 segment center 的位置，再向两边扩展。
4. 插入 detour 后，该 edge 的两个 endpoint 附近 1 格范围内的朝向必须保持不变。
5. 插入 detour 不允许压到任何 node、edge、bbox obstacle、已有 route、已有 detour 或任何其它实体。
6. 优先尝试 k=1 detour；如果所有可插位置都加满 k=1 后仍无法平衡，再按同样顺序把已有 k=1 detour 升级为 k=2；再不行升级为 k=3，以此类推。
7. 紧紧相邻的两个 detour 必须朝不同方向绕。否则两个相邻 detour 同向只是在局部平移线段，没有实际制造有效绕行空间。
8. 所有操作都必须保持 route 合法。

---

## 6. Endpoint orientation preservation

在任意 edge polyline 上加入 detour 时，必须保证该 edge 两端点 1 格范围内朝向不变。

定义：

```text
edge polyline = p0 -> p1 -> ... -> pn
p0 是 parent loc
pn 是 child loc
```

detour 不允许修改：

```text
p0 附近 1 grid 内的第一段方向
pn 附近 1 grid 内的最后一段方向
```

具体规则：

- 不允许在距离 parent endpoint L1 <= 1 的位置插 detour。
- 不允许在距离 child endpoint L1 <= 1 的位置插 detour。
- 插入 detour 后，`parent_exit_dir` 必须和 router 原 route 一致。
- 插入 detour 后，`child_entry_dir` 必须和 router 原 route 一致。

原因：router 已经处理了 port capacity 和 preferred connect side；detourer 不能破坏 router 选择的接入方向。

---

## 7. Detour legality check

每次尝试插入或升级 detour，都必须做完整合法性检查。

检查对象包括：

```text
candidate updated polyline for one edge
all other committed edge polylines
all node locs
all obstacle bboxes
all existing detours
```

必须检查：

1. 所有点在 die/grid boundary 内。
2. 所有 segment 水平或垂直。
3. polyline 不自交。
4. 不压到任何非本 edge endpoint 的 node。
5. 不压到任何其它 route segment。
6. 不和任何其它 route segment 非法 crossing。
7. 不和任何其它 route segment 共线 overlap。
8. 不穿过 forbidden bbox。
9. 不形成 topology 外的隐式连接。
10. 不破坏 edge 两端 1 格范围内朝向。

注意：

- detourer 不能假设 router 原始 route 之外的空间都可用。
- detourer 需要基于 router 输出的所有 committed segments 构建 occupied geometry。
- 当前正在修改的 edge 的旧 polyline 应从 occupancy 中暂时移除，然后检查新 polyline 是否与其它实体冲突，再 commit。

---

## 8. Adjacent detour direction rule

如果两个 detour 紧紧相邻，必须朝不同方向绕。

定义：两个 detour 在同一个原始 segment 上，且它们的 anchor interval 相邻或间隔小于等于 1 grid，则视为 tightly adjacent。

规则：

```text
if detour_i and detour_j tightly adjacent:
    detour_i.side != detour_j.side
```

例如水平 segment 上：

```text
上绕 + 上绕  // illegal if tightly adjacent
上绕 + 下绕  // preferred
```

原因：同方向紧邻 detour 往往等价于把一段线整体平移，没有形成有效局部绕格，且更容易制造 overlap/congestion。

---

## 9. Detour growth order

detourer 不能一开始就插大 detour。必须按层级增长。

对某个需要加 delay 的 side：

```text
level = 1
while skew not balanced and level <= max_detour_level:
    if level == 1:
        try inserting new k=1 detours at all legal positions in preferred order
    else:
        try upgrading existing detours from k=level-1 to k=level in the same insertion order
    recompute delay/skew after every accepted detour or upgrade
    if balanced enough:
        stop
    level += 1
```

重要：

- k=2 detour 应该由已有 k=1 detour 升级而来，不是随便在新位置直接插 k=2。
- k=3 由 k=2 升级而来，以此类推。
- 升级顺序必须复用 k=1 插入顺序。
- 每一次升级仍然必须做合法性检查。

---

## 10. Candidate position generation

对于一条 edge，先把 polyline 拆成 segments。

对每个 segment：

```text
segment_length = L1(a, b)
```

segment 长度至少需要满足：

```text
segment_length >= 3
```

否则无法在不破坏 endpoint 附近 1 格朝向的情况下插入 detour。

候选 segment 排序：

```text
longer segment first
then segment closer to edge center
then deterministic index order
```

候选 anchor 排序：

```text
start from segment center
then center-1, center+1, center-2, center+2, ...
```

对每个 anchor，尝试两侧绕行方向：

```text
for side in preferred_side_order(segment):
    try detour(anchor, side, k)
```

`preferred_side_order` 可以根据局部空旷度排序：

```text
side with fewer nearby occupied segments first
side farther from cluster bbox / node congestion first
then deterministic side order
```

---

## 11. Balancing objective

对于当前 node 的左右 child：

```text
left_worst  = max_sink_delay_to_node[left]  + edge_delay(node-left)
right_worst = max_sink_delay_to_node[right] + edge_delay(node-right)
```

设：

```text
short_side = side with smaller worst delay
long_delay = max(left_worst, right_worst)
short_delay = min(left_worst, right_worst)
delta = long_delay - short_delay
```

detour 目标是给 short_side 增加 delay：

```text
target_added_delay ≈ delta
```

每个 k 级 detour 增加：

```text
2 * k
```

由于 detour delay 是偶数增量，可能无法完全等于 delta。选择策略：

1. 优先让 `abs(new_delta)` 最小。
2. 若多个候选相同，优先不过度超过 long_delay。
3. 若仍相同，优先更少 detour 数。
4. 若仍相同，优先更小 k。
5. 若仍相同，优先更靠近最长边中心的位置。

---

## 12. Multi-child node

如果 topology 中存在超过 2 个 children 的 node，仍然使用 worst-delay balance：

```text
child_worst[i] = max_sink_delay_to_node[child_i] + edge_delay(node-child_i)
```

每次选择 delay 最小的 child subtree 加 detour，使其靠近当前最大 child_worst。

但如果 tree 理论上应为 binary，遇到 multi-child 时必须 debug warning。

---

## 13. Data structures

建议在 `common.h` 中补充或复用以下结构。

```cpp
struct DetourRecord {
    int edge_id = -1;
    int node_parent = -1;
    int node_child = -1;
    int segment_index = -1;
    int anchor_index = -1;
    int level = 1;
    int added_delay = 2;
    std::string side;          // UP/DOWN/LEFT/RIGHT relative to original segment
    bool upgraded = false;
};

struct DetourNodeResult {
    int node_id = -1;
    bool valid = false;
    std::vector<double> sink_delays_to_node;
    double min_sink_delay_to_node = 0.0;
    double max_sink_delay_to_node = 0.0;
    double skew_to_node = 0.0;
};

struct DetourerResult {
    bool valid = false;
    std::string error_msg;
    std::vector<DetourRecord> detour_records;       // all inserted detours for debug
    std::vector<DetourNodeResult> node_results;     // per-node delay/skew after detour
};
```

注意：不再需要 `DetourEdgeResult`——detoured polyline 直接写回 `route_result.edge_debugs[i].polyline`。`detour_records` 只记录插入位置和参数，不存完整 polyline。

---

## 14. Main flow

```text
run(problem, tree, loc_result, route_result, input_path):
    validate inputs
    // detourer 原地修改 route_result.edge_debugs[*].polyline
    build occupancy from all route polylines, node locs, bbox obstacles
    build bottom-up node order

    initialize sink delay profiles

    for node in bottom_up_order:
        ensure all child profiles are ready
        compute child worst delays using current detoured edge delays

        while node skew can be improved:
            choose shorter child side by child_worst_delay
            target_delta = longer_worst - shorter_worst
            if target_delta <= tolerance:
                break

            candidate_edges = collect edges in shorter child side, prefer current node-child edge then longer edges
            try_detour_growth(candidate_edges, target_delta)
            if no legal detour/upgrade possible:
                break

            recompute affected edge delay
            recompute child worst delays

        update sink_delays_to_node[node] into result.node_results
        update min/max/skew for node

    final legality check over all detoured polylines
    result.valid = true or false
    debug output / debug file output if enabled
    return result
```

注意：`route_result` 作为 mutable 引用传入，polyline 被原地修改。writer 后续读取同一个 `route_result` 即获得 detoured 结果。

---

## 15. Debug output

需要支持类似 router/locer 的 debug 开关。

```cpp
namespace detourer {
void debug_enable(bool enable);
void debug_file_enable(bool enable);
void debug_output(const DetourerResult& result,
                  const common::Problem& problem,
                  const common::TopologyTree& tree,
                  const common::LocerResult& loc_result,
                  const common::RouterResult& route_result);
bool write_debug_detour_file(const DetourerResult& result,
                             const std::string& input_path,
                             std::string& error_msg);
}
```

stdout debug 输出：

```text
[DETOURER] valid/error_msg/num_edges/num_nodes
for each node in bottom-up order:
  node_id class children
  child_worst_delays before/after
  selected_short_side
  target_delta
  inserted_detour_count
  added_delay
  final_skew

for each edge:
  edge_id parent child
  original_delay final_delay added_delay
  detour_count
  original_polyline
  detoured_polyline
```

---

## 16. Debug detour file

开关：

```cpp
detourer::debug_file_enable(true);
```

文件路径：

```text
detour/sample<k>_detour.txt
```

若无法从 input_path 推导 sample 名，则输出：

```text
detour/detour_debug.txt
```

推荐格式：

```text
# DETOURER_DEBUG v1
# valid=<0/1>
# num_edges=<E>
# num_nodes=<N>

edge <edge_id> <parent> <child> <original_delay> <final_delay> <added_delay> <detour_count> <point_count> <x0> <y0> <x1> <y1> ...

detour <edge_id> <segment_index> <anchor_index> <level> <added_delay> <side> <upgraded>

node <node_id> <sink_delay_count> <min_delay> <max_delay> <skew>
```

写文件前确保 `detour/` 目录存在。

---

## 17. Final legality check

detour 完成后必须做全局 legality check：

- 所有 route 仍是 Manhattan polyline。
- 所有 route endpoints 没变。
- 所有 parent exit / child entry direction 没变。
- 不压任何 node。
- 不压任何其它 edge。
- 不产生非法 crossing / overlap。
- 不自交。
- 不穿 forbidden bbox。
- 所有 delay profile 可从最终 polyline 重新计算得到，且与记录一致。

若失败：

```cpp
result.valid = false;
result.error_msg = "clear reason";
```

---

## 18. main.cc / 构建接入

`main.cc` 中建议：

```cpp
#include "detourer.h"

detourer::debug_enable(true);
detourer::debug_file_enable(true);
common::DetourerResult detour_result = detourer::run(problem, tree, loc_result, route_result, argv[1]);
// route_result.edge_debugs[*].polyline 已被 detourer 原地修改为 detoured polyline
if (!detour_result.valid) {
    std::cerr << "DETOURER error: " << detour_result.error_msg << "\n";
    return 1;
}

// writer 直接使用同一个 route_result，无需改动
writer::WriterResult writer_result =
    writer::write_solution(argv[1], problem, tree, loc_result, route_result);
```

构建文件：

```text
SRC += src/detourer.cc
INC += include/detourer.h
```