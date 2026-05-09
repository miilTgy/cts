请实现 Buffered-DME top-down 模块。TD 输入 `common::Problem`、`common::TopologyTree`、`common::BottomUpResult`，输出每个 tree node 的实际 location、最终 parent-child branch polyline、buffer marker、以及 debug 用的 route / compensation 信息。TD 不生成最终 txt，只提供 debug；`main.cc` 只调用 `td::run()` 并检查 `TopDownResult::valid`。

核心职责边界：TD 必须为每个非 root node 输出 `final_route_to_parent`，表示从 parent loc 到当前 node loc 的最终合法 rectilinear polyline。该 polyline 已经包含 obstacle-aware routing、绕障 extra、sibling balancing compensation 等所有 TD 几何结果。Writer 只允许读取 `final_route_to_parent` 并按格式输出，不需要、也不允许理解或拼接 TD 内部的 route / compensation 细节。

# td: Buffered-DME Top-down with Candidate-Loc DAG Routing

核心要求：

1. Top-down placement 不能只在 `ms(child)` 上随便选一个点，而要枚举一组 candidate loc。
2. 对同一个 child 的所有 candidate loc，把它们一起作为局部 routing DAG 的终点集合。
3. 在该 DAG 中搜索从 `parent_loc` 到任一 candidate loc 的最小 cost 合法路径。
4. 合法路径必须是 Manhattan polyline，且不能穿过非本 branch 端点的 sink。
5. 若某一侧 child branch 因绕障使 `routed_len > BU assigned_edge`，不要返回 BU；应在 sibling branch 上加入等长 compensation detour，使 parent 的左右子树增加相同 common-mode delay，从而不引入额外 skew。补偿后的最终几何必须体现在 child 的 `final_route_to_parent` 中。

## 1. API

新增 `td.h / td.cc`，namespace 为 `td`：

```cpp
#pragma once
#include "common.h"

namespace td {
using TopDownResult = common::TopDownResult;
using TopDownNodeResult = common::TopDownNodeResult;

void debug_enable(bool enable);
void debug_output(const TopDownResult& result,
                  const common::Problem& problem,
                  const common::TopologyTree& tree,
                  const common::BottomUpResult& bu_result);
TopDownResult run(const common::Problem& problem,
                  const common::TopologyTree& tree,
                  const common::BottomUpResult& bu_result);
}  // namespace td
```

`main.cc` 在 `bu::run()` 后调用：

```cpp
common::TopDownResult td_result = td::run(problem, tree, bu_result);
if (!td_result.valid) {
    std::cerr << "TD error: " << td_result.error_msg << "\n";
    return 1;
}
```

## 2. common.h 数据结构

若尚未存在，在 `namespace common` 中加入：

```cpp
struct TopDownNodeResult {
    int node_id = -1;
    bool valid = false;

    SegmentPoint loc;
    int parent_id = -1;

    double assigned_edge_to_parent = 0.0;       // BU edge_to_left/right
    double geometric_distance_to_parent = 0.0;  // L1(parent_loc, loc)
    double routed_length_to_parent = 0.0;       // obstacle-aware DAG route length
    double compensation_detour_to_parent = 0.0; // snake added for sibling balancing
    double final_length_to_parent = 0.0;        // routed_length + compensation

    std::vector<SegmentPoint> route_to_parent;              // base obstacle-aware route, parent loc -> loc, debug/internal
    std::vector<SegmentPoint> compensation_snake_to_parent; // optional local snake, debug/internal
    std::vector<SegmentPoint> final_route_to_parent;        // final branch polyline, parent loc -> loc, consumed by Writer

    MergingSegment feasible_ms;

    bool has_buffer = false;
    int buffer_type_index = -1;

    double min_delay = 0.0;
    double max_delay = 0.0;
    double skew = 0.0;

    // Extra delay common to all sinks under this node, introduced by TD route/snake.
    double td_common_extra_delay = 0.0;
};

struct TopDownResult {
    std::vector<TopDownNodeResult> node_results;  // index == tree node id
    int root = -1;
    bool valid = false;
    std::string error_msg;
};
```

语义：

- root 的 `route_to_parent`、`compensation_snake_to_parent`、`final_route_to_parent` 为空，所有 parent-edge length 为 0。
- 非 root 的 `route_to_parent.front()` 必须等于 parent loc，`back()` 必须等于本 node loc；该字段只表示基础 obstacle-aware route，用于 debug。
- 非 root 的 `final_route_to_parent.front()` 必须等于 parent loc，`back()` 必须等于本 node loc；该字段表示最终 branch polyline，是 Writer 唯一应该消费的 parent-child route。
- `compensation_snake_to_parent` 只用于 debug / 内部检查，Writer 不应读取它。
- `assigned_edge_to_parent` 是 BU 原始分配；TD 允许 `final_length_to_parent > assigned_edge_to_parent`，但必须通过 sibling balancing 保持左右 extra delay 相等，并把最终几何落实到 `final_route_to_parent`。

## 3. run() 流程

```text
run(problem, tree, bu_result):
    validate problem/tree/bu_result
    result.node_results.resize(tree.nodes.size())
    root = tree.root
    result[root].loc = midpoint(bu[root].ms)
    result[root].valid = true
    result[root].feasible_ms = bu[root].ms
    preorder from root:
        for each internal parent:
            place_sibling_pair_with_balancing(parent)
    final legality check
    return result
```

递归必须用 `state` 数组检测 cycle：0 unvisited，1 visiting，2 done。

## 4. DME top-down candidate loc 生成

对 child `c`，parent 位置为 `P`，BU edge 为 `e`：

```text
child_ms = bu[c].ms
parent_trr = TRR(core=P, radius=e)
feasible = child_ms ∩ parent_trr
```

不要只选一个 representative point。实现：

```cpp
static std::vector<SegmentPoint> generate_candidate_locs(
    const MergingSegment& child_ms,
    const TRR& feasible,
    const SegmentPoint& parent_loc,
    const std::vector<Sink>& sinks);
```

候选点集合：

```text
if feasible valid:
    add midpoint(child_ms) clamped into feasible
    add feasible rectangle corners
    add feasible rectangle edge midpoints
else:
    add nearest point on child_ms to parent_loc

for each forbidden sink o:
    add projection of x=o.x-1 and x=o.x+1 onto child_ms/feasible if valid
    add projection of y=o.y-1 and y=o.y+1 onto child_ms/feasible if valid

deduplicate by EPS
remove points outside child_ms
if feasible valid: remove points outside feasible
sort by L1(parent_loc, candidate)
keep at most K candidates, e.g. K=16 or 32
```

说明：sink 周围的 `±1` corridor 点是为了让 route DAG 更容易选择绕开 sink 的终点位置；DME 允许 `loc` 是 `ms(c) ∩ TRR(parent)` 中任意点，所以这里应把 location selection 和 routing legality 联合优化。

## 5. Candidate-loc L-shape route

本阶段先不要实现 full visibility graph / 多拐点 DAG。为避免产生很多无意义拐弯，TD route 只允许从 parent loc 到 child candidate loc 的 L-shape 搜索空间。

实现函数名可以暂时保留 `route_to_best_candidate_loc_dag()`，但语义改为：对每个 candidate loc 枚举 1 或 2 条 L-shape，并在所有合法 L-shape 中选择 cost 最小者。

```cpp
struct RouteCandidate {
    bool valid = false;
    SegmentPoint loc;
    std::vector<SegmentPoint> path;
    double geo = 0.0;
    double routed_len = 0.0;
    double cost = 0.0;
};

static RouteCandidate route_to_best_candidate_loc_dag(
    const SegmentPoint& parent_loc,
    const std::vector<SegmentPoint>& candidate_locs,
    const std::vector<Sink>& sinks,
    int allowed_parent_sink,
    int allowed_child_sink,
    double assigned_edge,
    std::string& err);
```

### 5.1 obstacle 规则

所有 sink 都是 point obstacle，除了当前 branch 的端点 sink：

```text
allowed_parent_sink = parent is leaf ? parent.sink_index : -1
allowed_child_sink  = child  is leaf ? child.sink_index  : -1
```

Horizontal/vertical segment 若经过 forbidden sink，则非法。判断包含端点；如果 internal location 落在 forbidden sink 上，也非法。

### 5.2 L-shape 搜索空间

令 `A = parent_loc`，某个候选 child 位置为 `T`。

对每个 `T` 枚举 L-shape path：

```text
HV: A -> (T.x, A.y) -> T
VH: A -> (A.x, T.y) -> T
```

如果 `A.x == T.x` 或 `A.y == T.y`，说明 parent 和 child candidate 已经水平或垂直对齐，此时搜索空间只有一条直线路径：

```text
A -> T
```

要求：

- 不构建 `x_coords × y_coords` 的 full visibility graph。
- 不使用 `sink.x±1` / `sink.y±1` corridor 作为中间转折点。
- 不生成超过 1 个 bend 的 route。
- 对齐时不要保留重复中间点，例如不要输出 `A -> A -> T` 或 `A -> T -> T`。
- 每条候选 path 生成后调用 `simplify_polyline()` 去掉重复点和共线中间点，但不能改变 route 的几何形状。
- 只接受 `polyline_crosses_forbidden_sink(path, ...) == false` 的候选路径。

伪代码：

```text
best = invalid
for each candidate T in candidate_locs:
    paths = []

    if A.x == T.x or A.y == T.y:
        paths.push([A, T])
    else:
        paths.push([A, (T.x, A.y), T])  // HV
        paths.push([A, (A.x, T.y), T])  // VH

    for each path in paths:
        path = simplify_polyline(path)
        if path invalid rectilinear: continue
        if path crosses forbidden sink: continue
        routed_len = polyline_length(path)
        geo = L1(A, T)
        cost = route_cost(path, T)
        update best

if no legal L-shape exists:
    err = "no legal L-shape route from parent to any child candidate"
    return invalid
return best
```

### 5.3 cost

因为只允许 L-shape，`routed_len == L1(A,T)`。cost 仍然需要偏好：

1. 不超过 BU assigned edge；
2. 少拐弯；
3. route 短；
4. candidate loc 尽量靠近 parent，作为稳定 tie-break。

推荐：

```text
route_cost(path, T) =
    10000.0 * max(0, routed_len - assigned_edge)
  +   100.0 * bend_count(path)
  +     1.0 * routed_len
  +     0.001 * L1(parent_loc, T)
```

由于目前只允许直线或 L-shape，`bend_count(path)` 只可能是 0 或 1。

输出：

```text
best.loc = selected endpoint candidate
best.path = selected legal L-shape path
best.geo = L1(A, best.loc)
best.routed_len = polyline_length(best.path)
best.cost = route_cost
```

要求：

- `best.path.front()==A`
- `best.path.back()==best.loc`
- 每段 horizontal/vertical
- 不穿 forbidden sink
- 最多 1 个 bend

## 6. sibling balancing

不要独立 finalize left/right。以 parent 为单位同时 route 两侧：

```cpp
static bool place_sibling_pair_with_balancing(
    int parent_id,
    int left_id,
    int right_id,
    const Problem& problem,
    const TopologyTree& tree,
    const BottomUpResult& bu_result,
    TopDownResult& result,
    std::string& err);
```

临时结构：

```cpp
struct BranchCandidate {
    int child_id = -1;
    bool is_left_child = false;
    int allowed_parent_sink = -1;
    int allowed_child_sink = -1;
    double assigned_edge = 0.0;
    RouteCandidate route;
    double route_excess = 0.0;
    double compensation = 0.0;
    double final_len = 0.0;
    MergingSegment feasible_ms;
};
```

流程：

```text
for each side child:
    build feasible = child_ms ∩ TRR(parent_loc, assigned_edge)
    locs = generate_candidate_locs(child_ms, feasible, parent_loc, sinks)
    // despite the function name, this stage currently only searches legal straight/L-shape routes
    route = route_to_best_candidate_loc_dag(parent_loc, locs, sinks,
                                            allowed_parent_sink,
                                            allowed_child_sink,
                                            assigned_edge)
    if route invalid: TD invalid
    route_excess = max(0, route.routed_len - assigned_edge)

left_child_extra  = result[left].td_common_extra_delay  if already valid else 0
right_child_extra = result[right].td_common_extra_delay if already valid else 0

common_extra = max(left_child_extra  + left.route_excess,
                   right_child_extra + right.route_excess)

for each side child:
    child_extra = result[child].td_common_extra_delay if already valid else 0
    final_len = assigned_edge + (common_extra - child_extra)
    compensation = final_len - route.routed_len
    if compensation < -EPS: internal error
    if compensation > EPS:
        compensation_snake = make_compensation_snake(route.path, compensation, sinks, allowed sinks)
    final_route = merge_base_route_and_compensation(route.path, compensation_snake)
    write TopDownNodeResult, including final_route_to_parent = final_route
```

关键不变量：

```text
left_child_extra + final_len_left  - assigned_edge_left
==
right_child_extra + final_len_right - assigned_edge_right
== common_extra
```

因此 TD 允许整体 delay 增大，但不改变左右相对 delay，不引入额外 skew。

`final_route_to_parent` 是 TD 对外输出的最终 branch route。无论 compensation 是以 out-and-back snake、局部绕线，还是未来改成更复杂的合法 detour 方式实现，最终都必须被合并进 `final_route_to_parent`。Writer 不负责把 `route_to_parent` 和 `compensation_snake_to_parent` 拼起来。

## 7. compensation snake

接口：

```cpp
static std::vector<SegmentPoint> make_compensation_snake(
    const std::vector<SegmentPoint>& base_route,
    double required_extra,
    const std::vector<Sink>& sinks,
    int allowed_parent_sink,
    int allowed_child_sink,
    std::string& err);
```

实现：

```text
if required_extra <= EPS: return empty
for each point anchor on base_route:
    try out-and-back horizontal: anchor -> (anchor.x+d, anchor.y) -> anchor, 2d=required_extra
    try out-and-back vertical:   anchor -> (anchor.x, anchor.y+d) -> anchor, 2d=required_extra
    try opposite directions too
    accept first snake that does not cross forbidden sink
if all fail: invalid
```

若输出要求整数坐标，则把 `required_extra` 向上取整到可表示长度，并在 sibling balancing 中两边使用同一个 rounded common extra。

补偿 snake 的输出不是 Writer 接口。TD 必须在 sibling balancing 阶段把 base route 和 compensation snake 合并为 `final_route_to_parent`。推荐实现 helper：

```cpp
static std::vector<SegmentPoint> merge_base_route_and_compensation(
    const std::vector<SegmentPoint>& base_route,
    const std::vector<SegmentPoint>& compensation_snake);
```

最简单实现：如果 `compensation_snake` 为空，返回 `base_route`；否则把 snake 插入到它的 anchor 点位置，保证最终 polyline 仍然从 parent loc 到 child loc，且每段都是水平/垂直线段。不要让 Writer 再做这一步。

## 8. geometry helper

在 `td.cc` 匿名 namespace 中实现：

```cpp
struct TRR { double u_min, u_max, v_min, v_max; bool valid; };
static constexpr double EPS = 1e-9;
static constexpr double INF = 1e100;

static double to_u(double x, double y);       // x+y
static double to_v(double x, double y);       // x-y
static SegmentPoint from_uv(double u, double v);
static TRR segment_to_trr(const MergingSegment& ms);
static TRR expand_trr(const TRR& base, double radius);
static TRR intersect_trr(const TRR& a, const TRR& b);
static bool point_in_trr(const SegmentPoint& p, const TRR& r);
static bool point_on_ms(const SegmentPoint& p, const MergingSegment& ms);
static bool is_valid_ms_segment(const MergingSegment& ms);
static SegmentPoint midpoint_of_ms(const MergingSegment& ms);
static SegmentPoint nearest_point_on_ms_to_point(const MergingSegment& ms, const SegmentPoint& p);
static double manhattan(const SegmentPoint& a, const SegmentPoint& b);
static double polyline_length(const std::vector<SegmentPoint>& path);
static int bend_count(const std::vector<SegmentPoint>& path);
static std::vector<SegmentPoint> simplify_polyline(std::vector<SegmentPoint> path);
// Only remove duplicate adjacent points and collinear middle points; never reroute or change geometry.
static bool segment_crosses_forbidden_sink(...);
static bool polyline_crosses_forbidden_sink(...);
```

Rotated coordinate 定义：

```text
u = x + y
v = x - y
L1 distance = max(abs(u1-u2), abs(v1-v2))
```

`MergingSegment` 合法性：`ms.valid=true`，且在 `(u,v)` 中是 point、u-fixed segment 或 v-fixed segment。

## 9. buffer 规则

BU 的 buffer choice 表示 buffer 插在 child node 上：

```text
parent_bu.buffer_at_left_child  -> mark left child node has_buffer
parent_bu.buffer_at_right_child -> mark right child node has_buffer
```

TD 不重新选 buffer，只检查 index 合法并复制 marker。

## 10. delay / skew 检查

对 internal parent，检查 TD 没有改变左右相对 delay：

```text
left_extra = left_td.td_common_extra_delay
           + left_td.final_length_to_parent
           - parent_bu.edge_to_left

right_extra = right_td.td_common_extra_delay
            + right_td.final_length_to_parent
            - parent_bu.edge_to_right

abs(left_extra - right_extra) <= 1e-6
parent_td.td_common_extra_delay == left_extra == right_extra
```

实际 arrival interval：

```text
left_actual  = left_bu interval  + left_td.final_length_to_parent  + left_td.td_common_extra_delay  + buffer_delay_left
right_actual = right_bu interval + right_td.final_length_to_parent + right_td.td_common_extra_delay + buffer_delay_right
```

相对 BU 的 extra 相同，所以 skew 不变。

同时检查最终输出几何：

```text
final_route_to_parent.front() == parent loc
final_route_to_parent.back()  == node loc
polyline_length(final_route_to_parent) == final_length_to_parent
```

允许 `route_to_parent` 和 `compensation_snake_to_parent` 只作为 debug 字段存在，但 `final_route_to_parent` 必须完整表达最终 branch 长度。

## 11. debug 输出

`debug_output()` 打印：

```text
[TD] root valid error_msg
for each node:
  node_id parent left right is_leaf sink_index
  loc
  assigned_edge geo routed_len compensation final_len td_common_extra_delay
  candidate_loc_count selected_loc route_cost
  route_to_parent                 // base route, debug
  compensation_snake_to_parent    // debug/internal
  final_route_to_parent           // final Writer-facing branch route
  buffer marker
  bu_ms bu_min bu_max bu_skew
```

`place_sibling_pair_with_balancing()` debug 打印：

```text
parent left right
left:  candidates selected_loc assigned routed excess child_extra compensation final_len
right: candidates selected_loc assigned routed excess child_extra compensation final_len
common_extra
```

## 12. 合法性检查

至少检查：

- problem/tree/bu valid。
- root 合法，BU result 数量等于 tree node 数。
- 每个 BU merging segment 合法。
- internal node 的 left/right 合法且不同。
- 每个非 root node：`route_to_parent` 非空，首点是 parent loc，末点是 node loc。
- 每个非 root node：`final_route_to_parent` 非空，首点是 parent loc，末点是 node loc。
- `route_to_parent` 和 `final_route_to_parent` 每段都必须水平/垂直，不穿 forbidden sink。
- `routed_length_to_parent == polyline_length(route_to_parent)`。
- `final_length_to_parent == routed_length_to_parent + compensation_detour_to_parent`。
- `final_length_to_parent == polyline_length(final_route_to_parent)`。
- 同一 parent 左右 branch 的 extra delay 相等。
- buffer index 合法。
- 所有 tree nodes 被访问。

失败时返回 `valid=false` 和清晰 `error_msg`。

---

全文中任何暗示 Writer 应拼接 `route_to_parent` 和 `compensation_snake_to_parent` 的表述，均已改为：TD 自身必须产出 `final_route_to_parent`，Writer 只消费 `final_route_to_parent`。