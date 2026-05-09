请实现 Buffered-DME bottom-up 模块，并把它接入现有 `main.cc`。不要实现 parser、treer、top-down、router、evaluator 或输出文件生成器。`bu` 的输入为 `common::Problem` 和 `common::TopologyTree`，输出每个 tree node 的 merging segment、delay interval、skew、wire estimate、buffer choice。`bu` 不输出 txt 文件，只提供可开关 debug 打印；`main.cc` 只负责调用 `bu::run()`、检查 `BottomUpResult::valid`，并在失败时打印错误返回非零。

# bu: Buffered-DME Bottom-up

## 1. 文件 / namespace / API

- 文件：`bu.h`、`bu.cc`；同时需要在 `main.cc` 和构建文件中接入 `bu`。
- 所有函数放在 `namespace bu`。
- 共享数据结构放在 `namespace common` 的 `common.h`。
- 不修改 `parser`、`treer` 接口。
- `main.cc` 在 parser、treer 成功后调用 `bu::run(problem, tree)`；若 `bu_result.valid == false`，打印 `BU error: <error_msg>` 并返回非零。
- 不用 exception 作为主错误处理；失败时返回 `valid=false` 和 `error_msg`。

`bu.h`：

```cpp
#pragma once
#include "common.h"

namespace bu {
using BottomUpResult = common::BottomUpResult;
using BottomUpNodeResult = common::BottomUpNodeResult;
using MergingSegment = common::MergingSegment;
using BufferChoice = common::BufferChoice;

void debug_enable(bool enable);
void debug_output(const BottomUpResult& result,
                  const common::Problem& problem,
                  const common::TopologyTree& tree);
BottomUpResult run(const common::Problem& problem,
                   const common::TopologyTree& tree);
}  // namespace bu
```

## 2. common.h 需要补充的数据结构

如果尚未存在，请在 `namespace common` 中加入：

```cpp
struct SegmentPoint {
    double x = 0.0;
    double y = 0.0;
};

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

    // Debug / implementation trace fields.
    // extraction_mode: "LEAF", "BOUNDARY_INTERSECTION",
    //                  "INTER_BOUNDARY_FALLBACK", "INTERIOR_FALLBACK".
    std::string extraction_mode;
    double detour_to_left = 0.0;
    double detour_to_right = 0.0;
};

struct BottomUpResult {
    std::vector<BottomUpNodeResult> node_results;  // index == tree node id
    int root = -1;
    bool valid = false;
    std::string error_msg;
};
```

语义：

- `node_results[node_id]` 对应 `tree.nodes[node_id]`。
- leaf 的 `ms` 是 sink 坐标退化点。
- internal node 的 `ms` 是 DME bottom-up 通过两个 child TRR 区域交集得到的 merging segment。
- `edge_to_left/right` 是 bottom-up 分配给 parent node 到 child node 的 branch delay/radius。
- parent `ms` 到 child `ms` 的最短 Manhattan 距离可能小于 `edge_to_left/right`；差值记录为 `detour_to_left/right`，表示该 child 侧需要在树/布线中额外实现的 detour/slack。
- buffer choice 表示是否在对应 child node 位置插 buffer。buffer 不插在 edge 中间；它作为 child node 上的器件，电气上驱动该 child subtree 的所有 downstream sinks。

## 3. run() 主流程

`bu.cc` 内部维护：

```cpp
static bool g_debug_enabled = false;
```

`run(problem, tree)`：

```text
if !problem.valid or !tree.valid: return invalid
check tree.root valid and tree.nodes non-empty
result.node_results.resize(tree.nodes.size())
post-order solve_node(tree.root)
if success: result.root=tree.root, result.valid=true
if debug enabled: debug_output(result, problem, tree)
return result
```

建议递归 helper：

```cpp
static bool solve_node(int node_id,
                       const common::Problem& problem,
                       const common::TopologyTree& tree,
                       common::BottomUpResult& result,
                       std::string& err);
```

逻辑：

```text
solve_node(v):
    check v range
    node = tree.nodes[v]
    if leaf: solve_leaf(v)
    else:
        check left/right valid and different
        solve_node(left), solve_node(right)
        solve_internal(v, left, right)
```

可用 `state` 数组检测 cycle：0 unvisited，1 visiting，2 done。

## 4. leaf 初始化

对于 leaf node：

```text
sink = problem.sinks[node.sink_index]
ms = [(sink.x, sink.y), (sink.x, sink.y)]
edge_to_left = edge_to_right = 0
buffer choices = NONE
min_delay = max_delay = skew = 0
wire_est = buffer_cost = total_cost = 0
extraction_mode = "LEAF"
detour_to_left = detour_to_right = 0
valid = true
```

若 `sink_index` 越界，返回 invalid。

## 5. internal node merge + buffer 枚举

对 internal node `v`：

```text
left_id = tree.nodes[v].left
right_id = tree.nodes[v].right
L = result.node_results[left_id]
R = result.node_results[right_id]
```

每个 child 的 buffer option：

```text
no_buffer
每一种满足 max_fanout >= tree.nodes[child_id].sink_count 的 real buffer
```

real buffer 额外限制：

```text
if tree.nodes[child_id].is_leaf: 不允许插 real buffer
```

buffer 插入语义：

```text
buffer_at_left_child / buffer_at_right_child 表示 buffer 放在对应 child node 上，
不是放在 parent-child edge 中间。该 buffer delay 作为 lumped delay 加到该 child subtree
所有 downstream sinks 上；物理输出阶段应在 child node 处标记/放置 buffer。
```

枚举：

```text
best_total_cost = INF
for BL in left buffer options:
  for BR in right buffer options:
    l_buf_delay = delay(BL), r_buf_delay = delay(BR)
    added_buf_cost = cost(BL) + cost(BR)

    ok = calc_merge_candidate(L.ms, R.ms,
                              L.min_delay, L.max_delay, l_buf_delay,
                              R.min_delay, R.max_delay, r_buf_delay,
                              candidate_ms,
                              edge_to_left, edge_to_right,
                              detour_to_left, detour_to_right,
                              extraction_mode,
                              err)
    if !ok: continue

    l_min = L.min_delay + edge_to_left + l_buf_delay
    l_max = L.max_delay + edge_to_left + l_buf_delay
    r_min = R.min_delay + edge_to_right + r_buf_delay
    r_max = R.max_delay + edge_to_right + r_buf_delay

    candidate_min_delay = min(l_min, r_min)
    candidate_max_delay = max(l_max, r_max)
    candidate_skew = candidate_max_delay - candidate_min_delay

    candidate_wire_est = L.wire_est + R.wire_est + edge_to_left + edge_to_right
    candidate_buffer_cost = L.buffer_cost + R.buffer_cost + added_buf_cost
    candidate_total_cost = 5000*candidate_skew
                         + 50*candidate_wire_est
                         + 200*candidate_buffer_cost

    keep minimum candidate_total_cost
```

若没有合法 candidate，返回 invalid。否则写入 `result.node_results[v]`。

## 6. DME TRR region intersection + boundary-preferred extraction

不要使用 child segment center 插值。TRR（Tilted Rectangular Region）的正确定义是：以 child merging segment / Manhattan arc 为 core，收集所有到该 core 的 Manhattan distance `<= radius` 的点形成的区域。因此：

```text
TRR(core, radius) = { p | dist_Manhattan(p, core) <= radius }
TRR boundary      = { p | dist_Manhattan(p, core) == radius }
```

TRR 本身是区域，不是边界。DME bottom-up 的核心步骤是：先对两个 child core 分别按 `rL`、`rR` 扩张得到两个 TRR 区域，再求区域交集作为 parent feasible merging region。若交集是二维区域，工程实现中优先从其边界中提取更接近 `dist_to_left == rL`、`dist_to_right == rR` 的代表性 merging segment；若找不到合适边界 segment，允许从 feasible region 内部 fallback，但必须 debug 标记。

### 6.1 helper 和内部结构

在 `bu.cc` 匿名 namespace 中实现：

```cpp
struct TRR {
    double u_min = 0.0, u_max = 0.0;
    double v_min = 0.0, v_max = 0.0;
    bool valid = false;
};

static constexpr double EPS = 1e-9;
static double to_u(double x, double y);
static double to_v(double x, double y);
static common::SegmentPoint from_uv(double u, double v);
static common::MergingSegment point_segment(double x, double y);
static double clamp_double(double x, double lo, double hi);
static TRR segment_to_trr(const common::MergingSegment& ms);
static TRR expand_trr(const TRR& base, double radius);
static TRR intersect_trr(const TRR& a, const TRR& b);
static double min_distance_between_ms(const common::MergingSegment& a,
                                      const common::MergingSegment& b);
static double min_distance_between_base_regions(const TRR& a, const TRR& b);

static bool is_valid_ms_segment(const common::MergingSegment& ms);
static double distance_point_to_trr_in_uv(double u, double v, const TRR& base);
static common::MergingSegment strict_dme_segment_from_intersection(
    const TRR& inter,
    const TRR& left_base,
    const TRR& right_base,
    double rL,
    double rR,
    std::string& extraction_mode);
static double min_distance_between_segment_and_base(
    const common::MergingSegment& ms,
    const TRR& base);
```

`segment_to_trr(ms)`：

```text
u1=p1.x+p1.y, v1=p1.x-p1.y
u2=p2.x+p2.y, v2=p2.x-p2.y
return [min(u1,u2), max(u1,u2)] x [min(v1,v2), max(v1,v2)]
```

`MergingSegment` 合法性要求：任意 node 都允许 point segment，只要 `ms.valid=true`。非 point segment 必须是 rotated coordinate 下 axis-aligned 的 segment，即 `abs(u1-u2)<=EPS` 或 `abs(v1-v2)<=EPS`。实现 `is_valid_ms_segment(ms)`：先检查 `ms.valid`，再计算两端点的 `(u,v)`，若既不是 point、也不是 u-fixed、也不是 v-fixed，则 invalid。`calc_merge_candidate()` 开头必须检查 left/right ms 合法；`solve_internal()` 写入 node result 后也要检查 candidate ms 合法。

几何距离 helper 的精确定义：

所有距离计算统一使用 rotated coordinate：

```text
u = x + y
v = x - y
```

在这个坐标系下，两个物理点的 Manhattan distance 满足：

```text
dist_L1((x1,y1),(x2,y2)) = max(abs(u1-u2), abs(v1-v2))
```

因此，TRR 在 `(u,v)` 中是 axis-aligned rectangle，所有 point-to-region / region-to-region Manhattan distance 都应实现为 `(u,v)` 下的 L∞ 距离，而不是再除以 2，也不是欧氏距离。

`distance_point_to_trr_in_uv(u, v, base)`：

```text
if !base.valid: return INF

du = 0
if u < base.u_min: du = base.u_min - u
else if u > base.u_max: du = u - base.u_max

dv = 0
if v < base.v_min: dv = base.v_min - v
else if v > base.v_max: dv = v - base.v_max

return max(du, dv)
```

含义：返回物理点 `from_uv(u,v)` 到 `base` 所表示的 core/ms 的最小 Manhattan distance。

`min_distance_between_base_regions(a,b)`：

```text
if !a.valid or !b.valid: return INF

du = 0
if a.u_max < b.u_min: du = b.u_min - a.u_max
else if b.u_max < a.u_min: du = a.u_min - b.u_max

dv = 0
if a.v_max < b.v_min: dv = b.v_min - a.v_max
else if b.v_max < a.v_min: dv = a.v_min - b.v_max

return max(du, dv)
```

含义：返回两个 `(u,v)` axis-aligned base region 之间的最小 Manhattan distance。

`min_distance_between_ms(a,b)`：

```text
if !is_valid_ms_segment(a) or !is_valid_ms_segment(b): return INF
base_a = segment_to_trr(a)
base_b = segment_to_trr(b)
return min_distance_between_base_regions(base_a, base_b)
```

含义：返回两个 child merging segment / core 之间的最小 Manhattan distance。外部逻辑只应调用这个函数，不要直接拿 expanded TRR 去算 `D`。

`min_distance_between_segment_and_base(ms, base)`：

```text
if !is_valid_ms_segment(ms) or !base.valid: return INF
ms_base = segment_to_trr(ms)
return min_distance_between_base_regions(ms_base, base)
```

含义：返回 candidate parent merging segment 到 child base/core 的最小 Manhattan distance，用于计算：

```text
detour = assigned_branch_delay - geometric_shortest_distance
```

注意：这里的 `base` 必须是 radius=0 的 child core/ms region，不是 `expand_trr(base, r)` 后的 expanded TRR。若传入 expanded TRR，detour 会被错误低估。

### 6.2 radius / edge length 分配

枚举 buffer 后：

```text
l_min0 = left_min_delay  + left_buffer_delay
l_max0 = left_max_delay  + left_buffer_delay
r_min0 = right_min_delay + right_buffer_delay
r_max0 = right_max_delay + right_buffer_delay
l_mid = (l_min0 + l_max0)/2
r_mid = (r_min0 + r_max0)/2
D = min_distance_between_ms(left_ms, right_ms)  // child core/ms 之间的最小 Manhattan distance，等价于 (u,v) 下 L∞ region distance
raw_left = (D + r_mid - l_mid)/2
edge_to_left = clamp(raw_left, 0, D)
edge_to_right = D - edge_to_left
```

该公式来自：

```text
l_mid + edge_to_left ≈ r_mid + edge_to_right
edge_to_left + edge_to_right = D
```

如果初始 radius 下两个 expanded TRR 没有合法区域交集，或交集存在但无法提取合适的 boundary-preferred segment，可做 repair：

```text
repair_limit = max(1.0, D + abs(l_mid-r_mid) + 10.0)
extra = 0.0
while true:
    try edge_to_left+extra and edge_to_right+extra
    if success: return candidate

    if extra == 0.0:
        extra = 1.0
    else:
        extra = extra * 2.0

    if extra > repair_limit + EPS:
        break
```
该 loop 明确包含 `extra=0` 的原始尝试，也包含所有 `extra <= repair_limit + EPS` 的 repair 尝试。`repair_limit` 是 double，因此比较时统一使用 `extra > repair_limit + EPS` 作为退出条件。不要写容易漏掉 `0 -> 1` 转换的倍增循环。

repair 后允许 `edge_to_left + edge_to_right > D`，代表 slack / detour，会被 wire cost 惩罚。

repair extra represents inserted slack/detour；top-down/router 必须在对应 node/branch 上保留这部分 assigned branch delay，否则 BU delay 与最终 evaluator delay 可能不一致。实现时应在 `detour_to_left/right` 中记录：

```text
detour_to_left  = edge_to_left  - min_distance_between_segment_and_base(candidate_ms, base_left)
detour_to_right = edge_to_right - min_distance_between_segment_and_base(candidate_ms, base_right)
```

若数值误差导致 detour 为极小负数，可 clamp 到 0。

### 6.3 严格 DME boundary intersection + fallback

关键要求：`intersect_trr(expand_trr(left,rL), expand_trr(right,rR))` 得到的是两个 TRR 区域的交集，即 parent feasible merging region。TRR 本身不是 `dist == r` 的边界；边界只是 TRR 区域的一部分。

严格 DME 优先级：

```text
1. 先求 boundary(EL) ∩ boundary(ER)，其中：
   EL = expand_trr(left_base, rL)
   ER = expand_trr(right_base, rR)
2. 若存在非空交集，parent ms 必须从该双边界交集中提取，
   extraction_mode = "BOUNDARY_INTERSECTION"。
3. 若双边界交集为空，但 TRR 区域交集 inter 非空，才退化到 boundary(inter)，
   extraction_mode = "INTER_BOUNDARY_FALLBACK"。
4. 若 boundary(inter) 也无法形成合法 segment，最后才允许从 inter 内部取最长
   axis-aligned-in-uv representative segment，
   extraction_mode = "INTERIOR_FALLBACK"。
   该 fallback 作为防御逻辑保留；在正常 axis-aligned TRR rectangle 实现下通常不应触发。
5. 不允许退回 child center 插值。
```

实现 `strict_dme_segment_from_intersection(inter, left_base, right_base, rL, rR, extraction_mode)`：

```text
1. 若 inter invalid，返回 invalid segment。
2. 构造 expanded left/right TRR：EL=expand_trr(left_base,rL), ER=expand_trr(right_base,rR)。
3. 枚举 EL 四条边与 ER 四条边的 pairwise intersection：
   - 边形式只允许 u=const, v in [lo,hi] 或 v=const, u in [lo,hi]。
   - 两条边平行且共线时，取 interval overlap，得到 segment。
   - 两条边垂直时，若 fixed coordinate 落在对方 interval 内，得到 point。
   - 候选还必须落在 inter 内。
   - 候选必须通过 is_valid_ms_segment()。
4. 若存在双边界候选：
   - 优先选择长度最长者；
   - 若并列，选择 midpoint 到两个 base 的 distance 更接近 rL/rR 者：
     score = abs(dl-rL) + abs(dr-rR)。
   - 返回该候选，extraction_mode="BOUNDARY_INTERSECTION"。
5. 若双边界候选为空，枚举 boundary(inter) 的四条边作为候选：
   u=inter.u_min, u=inter.u_max, v=inter.v_min, v=inter.v_max。
   对每条边计算 midpoint 到 left_base/right_base 的 Manhattan distance：
   dl = distance_point_to_trr_in_uv(mid_u, mid_v, left_base)
   dr = distance_point_to_trr_in_uv(mid_u, mid_v, right_base)
   // distance_point_to_trr_in_uv 已经返回物理 Manhattan distance，公式为 (u,v) 下的 L∞ distance。
   score = abs(dl-rL) + abs(dr-rR)
   选择 score 最小者；若并列，选线段长度较长者。
   返回该候选，extraction_mode="INTER_BOUNDARY_FALLBACK"。
6. 若 inter 退化为 point，则返回该 point，extraction_mode="INTER_BOUNDARY_FALLBACK"。
7. 若仍没有候选，从 inter 内部取最长 axis-aligned-in-uv representative segment：
   - 若 inter.u_width >= inter.v_width，取 v_mid=(v_min+v_max)/2，
     segment: (u_min,v_mid) 到 (u_max,v_mid)。
   - 否则取 u_mid=(u_min+u_max)/2，
     segment: (u_mid,v_min) 到 (u_mid,v_max)。
   返回该候选，extraction_mode="INTERIOR_FALLBACK"。该路径是防御 fallback，用于处理前面边界候选构造意外失败的情况；不得替代正常的 boundary extraction。
```

把候选边转回 `MergingSegment`：

```text
u fixed: p1=from_uv(u, v_lo), p2=from_uv(u, v_hi)
v fixed: p1=from_uv(u_lo, v), p2=from_uv(u_hi, v)
point: p1=p2=from_uv(u,v)
```

这样 parent `ms` 优先来自严格的两个 child TRR boundary intersection；只有双边界不存在时，才退化到 `boundary(inter)`；最后才允许 interior fallback，并且必须在 result/debug 中记录。

### 6.4 calc_merge_candidate()

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
check left_ms/right_ms valid and pass is_valid_ms_segment()
base_left=segment_to_trr(left_ms), base_right=segment_to_trr(right_ms)  // radius=0 child core/ms regions
compute l_mid/r_mid, then D = min_distance_between_ms(left_ms, right_ms), then initial edge_to_left/right
repair_limit = max(1.0, D + abs(l_mid-r_mid) + 10.0)
// D must be the minimum distance between the two child cores/ms, not the distance between expanded TRRs.
extra = 0.0
while true:
    // try current extra first; then update 0 -> 1 -> 2 -> 4 -> ...
    // break when next extra exceeds repair_limit + EPS
    rL = edge_to_left + extra
    rR = edge_to_right + extra
    inter = intersect_trr(expand_trr(base_left,rL), expand_trr(base_right,rR))
    if inter.valid:
        candidate_ms = strict_dme_segment_from_intersection(inter, base_left, base_right,
                                                            rL, rR, extraction_mode)
        if candidate_ms.valid and is_valid_ms_segment(candidate_ms):
            edge_to_left = rL
            edge_to_right = rR
            geo_left  = min_distance_between_segment_and_base(candidate_ms, base_left)
            geo_right = min_distance_between_segment_and_base(candidate_ms, base_right)
            detour_to_left  = max(0, edge_to_left  - geo_left)
            detour_to_right = max(0, edge_to_right - geo_right)
            return true
return false with err
```

不要在失败时使用 child segment center 插值。允许的 fallback 只能来自 TRR feasible region 内部，并且必须可 debug 追踪。

## 7. delay / skew / cost

对 candidate：

```text
left interval  = [L.min_delay + edge_to_left  + delay(BL),
                  L.max_delay + edge_to_left  + delay(BL)]
right interval = [R.min_delay + edge_to_right + delay(BR),
                  R.max_delay + edge_to_right + delay(BR)]

min_delay = min(left_min, right_min)
max_delay = max(left_max, right_max)
skew = max_delay - min_delay
wire_est = L.wire_est + R.wire_est + edge_to_left + edge_to_right
buffer_cost = L.buffer_cost + R.buffer_cost + cost(BL) + cost(BR)
total_cost = 5000*skew + 50*wire_est + 200*buffer_cost
extraction_mode = mode returned by calc_merge_candidate()
detour_to_left/right = detour returned by calc_merge_candidate()
```

`skew[v]` 是 subtree 内所有 downstream sinks 的完整 delay interval 宽度。shared trunk above `v` 不改变 subtree skew，但 final evaluator 的 source-to-sink delay 仍包含所有 Manhattan wire edges。`wire_est` 只是 bottom-up estimate，最终 wirelength 应在 top-down/routing 后重算。

## 8. debug 输出

`debug_enable(bool)` 控制 debug。

`debug_output(result, problem, tree)` 在 debug 关闭时直接 return。开启时打印：

```text
[BU] valid/error_msg/root/num_node_results
for each node:
  node_id parent left right is_leaf sink_index sink_count
  if leaf: sink id and coord
  ms=[(x1,y1),(x2,y2)] edge_left edge_right detour_left detour_right
  extraction_mode
  buf_left buf_right
  min_delay max_delay skew wire_est buffer_cost total_cost
```

buffer choice 打印 `NONE` 或 `name(delay=..., fanout=..., cost=...)`。

若 `g_debug_enabled`，`calc_merge_candidate()` 内部可额外打印：

```text
base_left_trr, base_right_trr,
expanded_left_trr, expanded_right_trr,
intersection_trr, selected_segment,
extraction_mode=BOUNDARY_INTERSECTION/INTER_BOUNDARY_FALLBACK/INTERIOR_FALLBACK,
detour_left, detour_right
```

## 9. 合法性检查

至少检查：

- `problem.valid == true`，`tree.valid == true`。
- `tree.root` 合法，`tree.nodes` 非空。
- leaf `sink_index` 合法。
- internal node `left/right` 合法且不同。
- child result 在 merge 前 valid。
- 每个 input/output `MergingSegment` 必须通过 `is_valid_ms_segment()`：任意 node 都允许 point segment，只要 `ms.valid=true`；非 point segment 必须是 rotated coordinate 下 `u fixed` 或 `v fixed` 的 segment。
- `detour_to_left/right >= -EPS`；若 extraction_mode 是 fallback，debug 必须能显示是哪种 fallback。
- 每个 valid result：`ms.valid`，`min_delay <= max_delay`，`skew >= -EPS`，`wire_est >= -EPS`，`buffer_cost >= 0`。
- root result valid。

失败时设置 `result.valid=false` 和清晰 `error_msg`。

## 10. main.cc / 构建接入

```cpp
#include "bu.h"

parser::debug_enable(true);
treer::debug_enable(true);
bu::debug_enable(true);

common::Problem problem = parser::parse(argv[1]);
treer::TopologyTree tree = treer::build(problem, argv[1]);
common::BottomUpResult bu_result = bu::run(problem, tree);
if (!bu_result.valid) {
    std::cerr << "BU error: " << bu_result.error_msg << "\n";
    return 1;
}
```

构建文件也要接入：

```text
SRC += src/bu.cc
INC += include/bu.h
```

不要让 `bu` 生成输出 txt 文件；debug 只通过 `bu::debug_enable()` 控制 stdout 打印。
