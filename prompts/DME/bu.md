# bu.md — Buffered-DME Bottom-Up 实现提示词

请实现 `bu` 模块：接收 `locer` 提取出的 **cluster-local DME 子树数据结构**，只对该 cluster 内部的 `sink / internal / access` node 做 Buffered-DME bottom-up phase，计算每个 local tree node 的 merging segment、branch assigned delay、delay interval、skew、wire estimate、buffer choice 和 detour slack。

`bu` 是 `locer` 的内部/下游模块。它不再直接接收完整 `TopologyTree` 做全局 BU，也不处理 `bridge / top / global / source`。`bu` 只负责 cluster-local bottom-up 几何/时序计算，不负责 parser、treer、top-down、outer placement、routing、writer、evaluator，也不输出 solution txt。

---

# 1. 算法概要

输入：

- `common::Problem problem`
- `common::ClusterDmeInput input`

其中 `input` 由 `locer` 从 treer topology 中抽取，表示单个 cluster 的 DME 子树。该子树只允许包含：

```text
sink
internal
access
```

`input.root_local_id` 必须是该 cluster 的 access node。`bu` 不改变 topology，不创建/删除 node，也不处理任何 outer node。

输出：

- `common::BottomUpResult`
- `node_results[local_id]` 存放该 cluster-local node 的 bottom-up 结果

`BottomUpResult` 中必须保留 local id 到原始 topology node id 的映射，方便 `locer` 写回全局 `LocerResult`。

DME bottom-up phase：

```text
foreach node v in G, in bottom-up order:
    if v is a sink node:
        ms[v] = PL(v)
        // PL(v) 是 sink 坐标对应的 zero-length Manhattan arc
    else:
        (a, b) = children(v)
        CALC_EDGE_LENGTH(e_a, e_b)

        trr[a].core   = ms[a]
        trr[a].radius = |e_a|

        trr[b].core   = ms[b]
        trr[b].radius = |e_b|

        ms[v] = trr[a] ∩ trr[b]
        // ms[v] 是 v 的 merging segment
```

本实现中：

- `ms[v]` 使用已有 `common::MergingSegment`。
- `edge_to_left/right` 是 parent 到 child 的 assigned branch delay/radius。
- `TRR(core, radius)` 表示所有到 `core` 的 Manhattan distance `<= radius` 的点。
- `ms[v]` 从左右 child expanded TRR 的交集中提取。
- 如果 assigned branch delay 大于 parent ms 到 child ms 的最短几何距离，差值记录到 `detour_to_left/right`。
- buffer 只能放在 child node 上，不能插在 edge 中间。
- leaf 只能是 cluster 内部 sink。
- root 只能是 access。
- internal node 只能连接 `sink/internal/access` 子树。
- 遇到 `bridge/top/global/source` 视为 locer 抽取错误，直接返回 invalid。

---

# 2. 职责边界

必须做：

1. 新增/实现 `bu.h`、`bu.cc`。
2. 提供 `bu::run(problem, cluster_dme_input)`，供 `locer` 对每个 cluster 调用。
3. 只处理 `ClusterDmeInput` 中的 local binary tree。
4. 使用 `local_id` 访问本 cluster 内节点，用 `origin_node_id` 记录其来自 treer topology 的原始 node id。
5. 使用已有 `common.h` 中的数据结构；只有确实缺字段时才补充，避免重复定义。
6. 提供可开关 debug 输出。

不要做：

- 不修改 parser/treer 的接口。
- 不直接遍历完整 `TopologyTree`。
- 不处理 `bridge/top/global/source`。
- 不决定 access loc；access loc 由 locer/td 根据 `access.ms` 和 outer side preference 决定。
- 不做 top-down location assignment。
- 不做具体 route polyline。
- 不写 `result/sample<k>_solution.txt`。
- 不调用 evaluator。
- 不用 exception 作为主错误处理。

失败时返回：

```cpp
result.valid = false;
result.error_msg = "清晰错误信息";
```

---

# 3. API

```cpp
#pragma once
#include "common.h"

namespace bu {
using ClusterDmeInput = common::ClusterDmeInput;
using BottomUpResult = common::BottomUpResult;
using BottomUpNodeResult = common::BottomUpNodeResult;
using MergingSegment = common::MergingSegment;
using BufferChoice = common::BufferChoice;

void debug_enable(bool enable);
void debug_output(const BottomUpResult& result,
                  const common::Problem& problem,
                  const ClusterDmeInput& input);
BottomUpResult run(const common::Problem& problem,
                   const ClusterDmeInput& input);
}  // namespace bu
```

`bu.cc` 内部维护：

```cpp
static bool g_debug_enabled = false;
```

---

# 4. 使用已有数据结构

`locer` 传给 BU 的 cluster-local 输入建议结构：

```cpp
enum class DmeNodeClass {
    Sink,
    Internal,
    Access,
};

struct ClusterDmeNode {
    int local_id = -1;
    int origin_node_id = -1;   // treer/global topology node id
    DmeNodeClass node_class = DmeNodeClass::Internal;

    int left = -1;             // local id
    int right = -1;            // local id
    int parent = -1;           // local id

    int sink_index = -1;       // valid only for Sink
    int sink_count = 0;
};

struct ClusterDmeInput {
    int cluster_id = -1;
    int root_local_id = -1;    // must be Access
    int root_origin_node_id = -1;
    std::vector<ClusterDmeNode> nodes;
    bool valid = false;
    std::string error_msg;
};
```

假设 `common.h` 中已有或需要补齐以下语义字段：

```cpp
struct MergingSegment {
    SegmentPoint p1;
    SegmentPoint p2;
    bool valid = false;
};

struct BufferChoice {
    bool has_buffer = false;
    int buffer_type_index = -1;
};

struct BottomUpNodeResult {
    int node_id = -1;
    int local_id = -1;
    int origin_node_id = -1;
    bool valid = false;

    MergingSegment ms;
    double edge_to_left = 0.0;
    double edge_to_right = 0.0;

    BufferChoice buffer_at_left_child;
    BufferChoice buffer_at_right_child;

    double min_delay = 0.0;
    double max_delay = 0.0;
    double skew = 0.0;

    double wire_est = 0.0;
    int buffer_cost = 0;
    double total_cost = 0.0;

    std::string extraction_mode;
    double detour_to_left = 0.0;
    double detour_to_right = 0.0;
};

struct BottomUpResult {
    int cluster_id = -1;
    std::vector<BottomUpNodeResult> node_results;  // index == local id
    std::vector<int> local_to_origin_node_id;
    int root_local_id = -1;
    int root_origin_node_id = -1;
    bool valid = false;
    std::string error_msg;
};
```

字段语义：

- `node_results[local_id]` 对应 `input.nodes[local_id]`。
- `origin_node_id` 对应 treer/global topology 中的原始 node id，供 locer 写回结果。
- `local_to_origin_node_id[local_id]` 必须等于 `input.nodes[local_id].origin_node_id`。
- leaf 的 `ms` 是 sink 坐标退化点。
- internal node 的 `ms` 是两个 child TRR 区域交集得到的 feasible merging segment。
- `edge_to_left/right` 是 bottom-up 分配的 parent-to-child branch delay。
- `detour_to_left/right = assigned_branch_delay - geometric_shortest_distance`，小负数可按数值误差 clamp 到 0。
- `buffer_at_left/right_child` 表示 buffer 放在对应 child node 上，电气上驱动该 child subtree。

---

# 5. run() 主流程

```text
run(problem, input):
    if !problem.valid or !input.valid: return invalid
    check input.root_local_id valid and input.nodes non-empty
    check input.nodes[root].node_class == Access
    check all input.nodes are Sink/Internal/Access only
    result.cluster_id = input.cluster_id
    result.root_local_id = input.root_local_id
    result.root_origin_node_id = input.root_origin_node_id
    result.node_results.resize(input.nodes.size())
    result.local_to_origin_node_id.resize(input.nodes.size())
    fill local_to_origin_node_id from input.nodes
    post-order solve_node(input.root_local_id)
    if success:
        result.valid = true
    if debug enabled:
        debug_output(result, problem, input)
    return result
```

`solve_node(local_id)`：

```text
solve_node(local_id):
    check local_id range
    use state array 0/1/2 to detect cycle
    node = input.nodes[local_id]
    if node.node_class == Sink:
        solve_leaf(local_id)
    else:
        check node.node_class == Internal or Access
        check left/right valid and different
        solve_node(left)
        solve_node(right)
        solve_internal(local_id, left, right)
```

---

# 6. Leaf 处理

对于 leaf：

```text
node = input.nodes[local_id]
sink = problem.sinks[node.sink_index]
origin_node_id = node.origin_node_id
ms = [(sink.x, sink.y), (sink.x, sink.y)]
edge_to_left = edge_to_right = 0
buffer choices = NONE
min_delay = max_delay = skew = 0
wire_est = 0
buffer_cost = 0
total_cost = 0
extraction_mode = "LEAF"
detour_to_left = detour_to_right = 0
valid = true
```

若 `sink_index` 越界，返回 invalid。

---

# 7. Internal node：buffer 枚举 + DME merge

对 local internal/access node `v`：

```text
L = node_results[left]
R = node_results[right]
```

枚举每个 child 的 buffer option：

```text
no_buffer
每一种满足 max_fanout >= input.nodes[child].sink_count 的 real buffer
```

限制：

```text
if input.nodes[child].node_class == Sink:
    不允许在该 child 上插 real buffer
```

对每组 `(BL, BR)`：

```text
l_buf_delay = delay(BL)
r_buf_delay = delay(BR)
added_buf_cost = cost(BL) + cost(BR)

调用 calc_merge_candidate(...)
若失败，跳过该组合

left interval  = [L.min_delay + edge_to_left  + l_buf_delay,
                  L.max_delay + edge_to_left  + l_buf_delay]
right interval = [R.min_delay + edge_to_right + r_buf_delay,
                  R.max_delay + edge_to_right + r_buf_delay]

candidate_min_delay = min(left_min, right_min)
candidate_max_delay = max(left_max, right_max)
candidate_skew = candidate_max_delay - candidate_min_delay

candidate_wire_est = L.wire_est + R.wire_est + edge_to_left + edge_to_right
candidate_buffer_cost = L.buffer_cost + R.buffer_cost + added_buf_cost
candidate_total_cost = 5000*candidate_skew
                     + 50*candidate_wire_est
                     + 200*candidate_buffer_cost

保留 total_cost 最小的 candidate
```

若所有组合都失败，返回 invalid。

---

# 8. 几何约定：rotated coordinate

所有 DME/TRR 几何统一使用 rotated coordinate：

```text
u = x + y
v = x - y
```

物理 Manhattan distance 满足：

```text
dist_L1(p1, p2) = max(abs(u1-u2), abs(v1-v2))
```

因此：

- `MergingSegment` 转成 `(u,v)` 后应是 point、`u fixed` segment 或 `v fixed` segment。
- TRR 在 `(u,v)` 中是 axis-aligned rectangle。
- point-to-region / region-to-region 距离都是 `(u,v)` 下的 L∞ 距离。
- 不要除以 2，不要用欧氏距离。

建议内部结构：

```cpp
struct TRR {
    double u_min, u_max;
    double v_min, v_max;
    bool valid;
};
```

必须实现的核心 helper：

```cpp
to_u(x,y), to_v(x,y), from_uv(u,v)
segment_to_trr(ms)
expand_trr(base, radius)
intersect_trr(a,b)
is_valid_ms_segment(ms)
min_distance_between_ms(a,b)
distance_point_to_trr_in_uv(u,v,base)
min_distance_between_segment_and_base(ms,base)
```

注意：

- `min_distance_between_ms()` 计算两个 child core/ms 的最短距离。
- `min_distance_between_segment_and_base()` 的 `base` 必须是 child core/ms 的 radius=0 region，不能传 expanded TRR。

---

# 9. Edge length / radius 分配

对某个 buffer 组合：

```text
l_min0 = L.min_delay + l_buf_delay
l_max0 = L.max_delay + l_buf_delay
r_min0 = R.min_delay + r_buf_delay
r_max0 = R.max_delay + r_buf_delay

l_mid = (l_min0 + l_max0) / 2
r_mid = (r_min0 + r_max0) / 2

D = min_distance_between_ms(L.ms, R.ms)

raw_left = (D + r_mid - l_mid) / 2
edge_to_left = clamp(raw_left, 0, D)
edge_to_right = D - edge_to_left
```

含义：

```text
l_mid + edge_to_left ≈ r_mid + edge_to_right
edge_to_left + edge_to_right = D
```

若初始 radius 无法得到合法 `ms`，允许 repair 增加 slack/detour：

```text
repair_limit = max(1.0, D + abs(l_mid-r_mid) + 10.0)
extra = 0.0
while true:
    try rL = edge_to_left + extra
        rR = edge_to_right + extra
    if success: return candidate

    if extra == 0.0: extra = 1.0
    else: extra *= 2.0

    if extra > repair_limit + EPS: break
```

repair 后允许：

```text
edge_to_left + edge_to_right > D
```

多出来的部分作为 detour/slack 记录到 `detour_to_left/right`，并被 `wire_est` 惩罚。

---

# 10. strict DME segment extraction

`TRR(core, radius)` 是区域：

```text
{ p | dist_Manhattan(p, core) <= radius }
```

不是边界。对左右 child：

```text
base_left  = segment_to_trr(L.ms)
base_right = segment_to_trr(R.ms)
EL = expand_trr(base_left,  rL)
ER = expand_trr(base_right, rR)
inter = intersect_trr(EL, ER)
```

从 `inter` 中提取 parent `ms` 的优先级：

```text
1. boundary(EL) ∩ boundary(ER)
   extraction_mode = "BOUNDARY_INTERSECTION"

2. 若双边界交集为空，但 inter 非空，退化到 boundary(inter)
   extraction_mode = "INTER_BOUNDARY_FALLBACK"

3. 若仍失败，才从 inter 内部取最长 axis-aligned representative segment
   extraction_mode = "INTERIOR_FALLBACK"
```

禁止 fallback 到 child center 插值。

候选 segment 必须满足 `is_valid_ms_segment()`。

双边界候选选择规则：

```text
优先最长；若并列，选择 midpoint 到左右 base 的距离更接近 rL/rR 的：
score = abs(dl-rL) + abs(dr-rR)
```

`boundary(inter)` fallback 选择规则：

```text
枚举 inter 的四条边，选择 score 最小者；若并列，选更长者。
score = abs(dl-rL) + abs(dr-rR)
```

---

# 11. calc_merge_candidate()

建议签名：

```cpp
static bool calc_merge_candidate(const common::MergingSegment& left_ms,
                                 const common::MergingSegment& right_ms,
                                 double left_min_delay,
                                 double left_max_delay,
                                 double left_buffer_delay,
                                 double right_min_delay,
                                 double right_max_delay,
                                 double right_buffer_delay,
                                 common::MergingSegment& candidate_ms,
                                 double& edge_to_left,
                                 double& edge_to_right,
                                 double& detour_to_left,
                                 double& detour_to_right,
                                 std::string& extraction_mode,
                                 std::string& err);
```

流程：

```text
check left_ms/right_ms valid
base_left  = segment_to_trr(left_ms)
base_right = segment_to_trr(right_ms)
compute D, l_mid, r_mid, initial edge_to_left/right

for extra in 0,1,2,4,... until repair_limit:
    rL = edge_to_left + extra
    rR = edge_to_right + extra
    EL = expand_trr(base_left, rL)
    ER = expand_trr(base_right, rR)
    inter = intersect_trr(EL, ER)
    if inter invalid: continue

    candidate_ms = strict_dme_segment_from_intersection(inter, base_left, base_right, rL, rR)
    if candidate_ms valid:
        edge_to_left = rL
        edge_to_right = rR
        geo_left  = min_distance_between_segment_and_base(candidate_ms, base_left)
        geo_right = min_distance_between_segment_and_base(candidate_ms, base_right)
        detour_to_left  = max(0, edge_to_left  - geo_left)
        detour_to_right = max(0, edge_to_right - geo_right)
        return true

return false
```

---

# 12. 合法性检查

至少检查：

- `problem.valid == true`
- `input.valid == true`
- `input.root_local_id` 合法
- `input.nodes` 非空
- root node class 必须是 `Access`
- 所有 node class 只能是 `Sink/Internal/Access`
- `local_id` 必须等于其在 `input.nodes` 中的 index
- `origin_node_id` 必须非负
- leaf `sink_index` 合法
- internal/access `left/right` 合法且不同
- recursion 无 cycle
- child result 在 merge 前 valid
- input/output `MergingSegment` 合法
- `min_delay <= max_delay`
- `skew >= -EPS`
- `wire_est >= -EPS`
- `buffer_cost >= 0`
- root result valid

失败时设置 `valid=false` 和清晰 `error_msg`。

---

# 13. Debug 输出

`debug_enable(bool)` 控制 debug。

`debug_output()` 开启时打印：

```text
[BU] valid/error_msg/cluster_id/root_local_id/root_origin_node_id/num_node_results
for each local node:
  local_id origin_node_id class parent left right sink_index sink_count
  if leaf: sink id and coord
  ms=[(x1,y1),(x2,y2)]
  edge_left edge_right
  detour_left detour_right
  extraction_mode
  buf_left buf_right
  min_delay max_delay skew
  wire_est buffer_cost total_cost
```

buffer 打印：

```text
NONE
或 name(delay=..., fanout=..., cost=...)
```

fallback 必须能从 debug 中看出：

```text
BOUNDARY_INTERSECTION
INTER_BOUNDARY_FALLBACK
INTERIOR_FALLBACK
```

---

# 14. main.cc / 构建接入

BU 不再由 `main.cc` 直接对完整 topology 调用。它由 `locer` 对每个 cluster 调用：

```cpp
common::ClusterDmeInput cluster_input = locer_internal::build_cluster_dme_input(problem, tree, access_node_id);
common::BottomUpResult bu_result = bu::run(problem, cluster_input);
if (!bu_result.valid) {
    return locer_error("BU failed for cluster " + std::to_string(cluster_input.cluster_id)
                       + ": " + bu_result.error_msg);
}
```

构建文件：

```text
SRC += src/bu.cc
INC += include/bu.h
```