# td.md — Cluster-Local DME Top-Down Location Assignment 提示词

请实现 `td` 模块：接收 `locer` 提取出的 **cluster-local DME 子树数据结构**、`bu` 的 bottom-up 结果，以及 `locer` 选定的 access/root loc，只对该 cluster 内部的 `sink / internal / access` node 做 DME top-down location assignment。

`td` 是 `locer` 的内部/下游模块。它不再直接接收完整 `TopologyTree`，不处理 `bridge / top / global / source`，也不决定 access loc。`td` 只负责从已确定的 access loc 往下，为 cluster-local DME 子树中的 internal/sink node 选择实际 loc。

---

# 1. 算法概要

输入：

- `common::Problem problem`
- `common::ClusterDmeInput input`
- `common::BottomUpResult bu_result`
- `common::TopDownConfig config`

其中：

- `input` 由 `locer` 从 treer topology 中抽取，只包含 `sink / internal / access`。
- `input.root_local_id` 必须是 access node。
- `bu_result` 必须来自同一个 `ClusterDmeInput`。
- `config.root_loc` 是 locer 根据 `access.ms` 和 outer side preference 选择好的 access loc。

DME top-down phase：

```text
root = input.root_local_id        // access
p[root] = config.root_loc

foreach child v in top-down order:
    par = parent(v)
    parent_loc = p[par]
    edge = BU assigned edge from par to v

    feasible = ms[v] ∩ TRR(parent_loc, edge)
    if feasible non-empty:
        p[v] = best point in feasible
    else:
        p[v] = nearest point on ms[v] to parent_loc
```

本实现中：

- `ms[v]` 来自 `bu_result.node_results[local_id].ms`。
- `edge` 来自 parent BU result 的 `edge_to_left/right`。
- `TRR(core=parent_loc, radius=edge)` 表示 child loc 距离 parent loc 不超过 assigned edge 的可行区域。
- `td` 不改变 BU 的 delay/skew model。
- 若 fallback 导致 `L1(parent, child) > edge`，只记录 debug 信息，不补 detour、不修 route。

---

# 2. 职责边界

TD 必须做：

1. 新增/实现 `td.h`、`td.cc`。
2. 提供 `td::run(problem, cluster_dme_input, bu_result, config)`，供 `locer` 对每个 cluster 调用。
3. 只处理 `ClusterDmeInput` 中的 local binary tree。
4. root/access loc 直接使用 `config.root_loc`，不自行选择 midpoint 或 side point。
5. 为每个 local node 产生实际 `loc`。
6. 使用 `local_id` 访问本 cluster 内节点，用 `origin_node_id` 记录其来自 treer topology 的原始 node id。
7. 提供可开关 debug 输出。

TD 不做：

- 不直接遍历完整 `TopologyTree`。
- 不处理 `bridge/top/global/source`。
- 不决定 access loc。
- 不生成 route polyline。
- 不生成 `final_route_to_parent`。
- 不做 obstacle-aware routing 搜索。
- 不做 detour、compensation snake、sibling balancing。
- 不插 buffer、不重新选择 buffer、不复制 buffer marker。
- 不写 `result/sample<k>_solution.txt`。
- 不调用 evaluator。
- 不修改 parser/treer/bu 的接口。

失败时返回：

```cpp
result.valid = false;
result.error_msg = "清晰错误信息";
```

---

# 3. API

`td.h`：

```cpp
#pragma once
#include "common.h"

namespace td {
using ClusterDmeInput = common::ClusterDmeInput;
using TopDownConfig = common::TopDownConfig;
using TopDownResult = common::TopDownResult;
using TopDownNodeResult = common::TopDownNodeResult;

void debug_enable(bool enable);
void debug_output(const TopDownResult& result,
                  const common::Problem& problem,
                  const ClusterDmeInput& input,
                  const common::BottomUpResult& bu_result,
                  const TopDownConfig& config);
TopDownResult run(const common::Problem& problem,
                  const ClusterDmeInput& input,
                  const common::BottomUpResult& bu_result,
                  const TopDownConfig& config);
}  // namespace td
```

`td.cc` 内部维护：

```cpp
static bool g_debug_enabled = false;
```

---

# 4. common.h 数据结构

优先使用已有 `common.h`。若缺少 TD 结果结构，请在 `namespace common` 中补齐以下最小字段；不要加入 route、detour、buffer 字段。

`locer` 传入的 root/access loc 配置：

```cpp
struct TopDownConfig {
    bool has_root_loc = false;
    SegmentPoint root_loc;
    std::string root_loc_mode;  // ACCESS_MIDPOINT / ACCESS_LEFT / ACCESS_RIGHT / ACCESS_BOTTOM / ACCESS_TOP
};
```

TD 结果：

```cpp
struct TopDownNodeResult {
    int node_id = -1;           // keep compatible; set to origin_node_id
    int local_id = -1;
    int origin_node_id = -1;
    bool valid = false;

    SegmentPoint loc;
    int parent_local_id = -1;
    int parent_origin_node_id = -1;

    double assigned_edge_to_parent = 0.0;
    double geometric_distance_to_parent = 0.0;

    MergingSegment feasible_ms;
    bool used_feasible_intersection = false;
    std::string loc_mode;  // ROOT_FROM_CONFIG / FEASIBLE_CANDIDATE / NEAREST_MS_FALLBACK

    int candidate_count = 0;
    double loc_score = 0.0;

    double min_delay = 0.0;
    double max_delay = 0.0;
    double skew = 0.0;
};

struct TopDownResult {
    int cluster_id = -1;
    std::vector<TopDownNodeResult> node_results;  // index == local id
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
- `loc` 是 TD 输出的唯一核心结果。
- root/access 的 `loc` 必须等于 `config.root_loc`。
- `assigned_edge_to_parent` 来自 parent BU result 的 `edge_to_left/right`。
- `geometric_distance_to_parent = L1(parent_loc, loc)`。
- `feasible_ms` 表示 `ms[child] ∩ TRR(parent_loc, assigned_edge)` 提取出的可行 segment / point。
- `used_feasible_intersection=true` 表示 loc 位于 feasible intersection 内。
- `min_delay/max_delay/skew` 默认复制 BU 对应 local node 的估计值；TD 当前不改 delay model。

---

# 5. run() 主流程

```text
run(problem, input, bu_result, config):
    validate problem/input/bu_result/config
    check input.root_local_id valid
    check input.nodes[root].node_class == Access
    check config.has_root_loc == true
    check config.root_loc is on bu_result.node_results[root].ms

    result.cluster_id = input.cluster_id
    result.root_local_id = input.root_local_id
    result.root_origin_node_id = input.root_origin_node_id
    result.node_results.resize(input.nodes.size())
    result.local_to_origin_node_id.resize(input.nodes.size())
    fill local_to_origin_node_id from input.nodes

    place root:
        local_id = root
        origin_node_id = input.nodes[root].origin_node_id
        loc = config.root_loc
        parent ids = -1
        assigned/geometric = 0
        feasible_ms = bu[root].ms
        used_feasible_intersection = true
        loc_mode = ROOT_FROM_CONFIG
        copy BU delay/skew fields

    preorder DFS from root:
        for each child of current parent:
            place_child_loc(parent_local_id, child_local_id)

    final legality check
    result.valid = true
    if debug enabled: debug_output(...)
    return result
```

递归必须使用 `state` 数组检测 cycle：0 unvisited，1 visiting，2 done。

---

# 6. child loc 选择原则

对 local child `c`，parent loc 为 `P`，BU edge 为 `e`：

```text
child_ms = bu_result.node_results[c].ms
parent_trr = TRR(core=P, radius=e)
feasible = child_ms ∩ parent_trr
```

`e` 的来源：

```text
if c == input.nodes[parent].left:
    e = bu_result.node_results[parent].edge_to_left
else if c == input.nodes[parent].right:
    e = bu_result.node_results[parent].edge_to_right
else:
    invalid
```

若 `feasible` 非空：

```text
candidates = generate_candidate_locs(child_ms, feasible, P, problem.sinks)
choose candidate with minimum loc_score
loc_mode = FEASIBLE_CANDIDATE
used_feasible_intersection = true
```

若 `feasible` 为空：

```text
loc = nearest point on child_ms to P
loc_mode = NEAREST_MS_FALLBACK
used_feasible_intersection = false
```

TD 只选择 loc。即使 `L1(P, loc) > assigned_edge`，也只记录该事实，不在 TD 中补 detour 或修改 BU。

---

# 7. candidate loc 生成

实现：

```cpp
static std::vector<common::SegmentPoint> generate_candidate_locs(
    const common::MergingSegment& child_ms,
    const common::MergingSegment& feasible_ms,
    const common::SegmentPoint& parent_loc,
    const std::vector<common::Sink>& sinks);
```

候选点：

```text
add midpoint(feasible_ms)
add endpoints(feasible_ms)
add projection of parent_loc onto feasible_ms

for each sink o:
    add projection of x=o.x-1 and x=o.x+1 onto feasible_ms if valid
    add projection of y=o.y-1 and y=o.y+1 onto feasible_ms if valid

deduplicate by EPS
remove points outside feasible_ms
sort by loc_score preliminary order
keep at most K candidates, e.g. K=16 or 32
```

说明：

- sink 周围 `±1` projection 只是为了选出更 routable 的 loc。
- TD 不生成实际绕线路径。
- 若所有候选被删空，fallback 到 `midpoint(feasible_ms)`。

---

# 8. loc_score

选择 loc 时使用简单可解释的 score：

```text
score(loc) =
    10000.0 * max(0, L1(parent_loc, loc) - assigned_edge)
  +   100.0 * lshape_forbidden_sink_penalty(parent_loc, loc)
  +     1.0 * abs(L1(parent_loc, loc) - assigned_edge)
  +     0.001 * L1(parent_loc, loc)
```

含义：

1. 优先不超过 BU assigned edge。
2. 优先选择简单 L-shape 更不容易穿过 forbidden sink 的 loc。
3. 在可行范围内，距离 assigned edge 更接近通常更符合 DME edge 语义。
4. 最后用距离作为稳定 tie-break。

`lshape_forbidden_sink_penalty(A,T)`：

```text
if A and T 水平/垂直对齐:
    若直线经过 forbidden sink，penalty = 1，否则 0
else:
    check HV: A -> (T.x,A.y) -> T
    check VH: A -> (A.x,T.y) -> T
    penalty = number of illegal L-shapes among {HV,VH}
```

Forbidden sink：除当前 branch 的端点 sink 外，所有 sink 都是 forbidden point obstacle。

```text
allowed_parent_sink = parent is Sink ? parent.sink_index : -1
allowed_child_sink  = child  is Sink ? child.sink_index  : -1
```

注意：这里仅用于 loc 评分，不输出 route。

---

# 9. 几何 helper

在 `td.cc` 匿名 namespace 中实现：

```cpp
struct TRR { double u_min, u_max, v_min, v_max; bool valid; };
static constexpr double EPS = 1e-9;
static constexpr double INF = 1e100;

static double to_u(double x, double y);       // x+y
static double to_v(double x, double y);       // x-y
static common::SegmentPoint from_uv(double u, double v);
static TRR segment_to_trr(const common::MergingSegment& ms);
static TRR expand_trr(const TRR& base, double radius);
static TRR intersect_trr(const TRR& a, const TRR& b);
static common::MergingSegment extract_ms_from_trr_intersection(const TRR& inter);
static bool point_in_trr(const common::SegmentPoint& p, const TRR& r);
static bool point_on_ms(const common::SegmentPoint& p, const common::MergingSegment& ms);
static bool is_valid_ms_segment(const common::MergingSegment& ms);
static common::SegmentPoint midpoint_of_ms(const common::MergingSegment& ms);
static common::SegmentPoint nearest_point_on_ms_to_point(const common::MergingSegment& ms,
                                                         const common::SegmentPoint& p);
static common::SegmentPoint project_point_to_ms(const common::SegmentPoint& p,
                                                const common::MergingSegment& ms);
static double manhattan(const common::SegmentPoint& a,
                        const common::SegmentPoint& b);
static bool segment_crosses_forbidden_sink(...);
static int lshape_forbidden_sink_penalty(...);
```

Rotated coordinate：

```text
u = x + y
v = x - y
L1 distance = max(abs(u1-u2), abs(v1-v2))
```

`MergingSegment` 合法性：`ms.valid=true`，且在 `(u,v)` 中是 point、u-fixed segment 或 v-fixed segment。

---

# 10. delay / skew 语义

TD 当前不改变 BU 的 delay model：

```text
node_td.min_delay = node_bu.min_delay
node_td.max_delay = node_bu.max_delay
node_td.skew      = node_bu.skew
```

TD 只记录几何距离：

```text
geometric_distance_to_parent = L1(parent_loc, loc)
```

如果：

```text
geometric_distance_to_parent > assigned_edge_to_parent + EPS
```

只在 debug 中显示为 over-assigned-edge，不在 TD 中修正。

---

# 11. Debug 输出

`debug_enable(bool)` 控制 debug。

`debug_output()` 开启时打印：

```text
[TD] valid/error_msg/cluster_id/root_local_id/root_origin_node_id/root_loc/root_loc_mode/num_node_results
for each local node:
  local_id origin_node_id class parent_local parent_origin left right sink_index
  loc=(x,y)
  assigned_edge geometric_distance over_assigned_edge
  feasible_ms=[(x1,y1),(x2,y2)] used_feasible_intersection loc_mode
  candidate_count loc_score
  bu_ms=[...] bu_min bu_max bu_skew
```

若某个 loc 的简单 L-shape 会穿过 forbidden sink，应在 debug 中显示 penalty，便于后续 route 模块排查。

---

# 12. 合法性检查

至少检查：

- `problem.valid == true`
- `input.valid == true`
- `bu_result.valid == true`
- `config.has_root_loc == true`
- `input.root_local_id` 合法
- `input.nodes` 非空
- root node class 必须是 `Access`
- 所有 node class 只能是 `Sink/Internal/Access`
- `bu_result.node_results.size() == input.nodes.size()`
- `bu_result.cluster_id == input.cluster_id`
- `bu_result.root_local_id == input.root_local_id`
- 每个 BU `ms` 合法
- `local_id` 必须等于其在 `input.nodes` 中的 index
- `origin_node_id` 必须非负
- internal/access node 的 left/right 合法且不同
- DFS 无 cycle
- 每个 node 都被访问
- root loc 等于 `config.root_loc`
- root loc 在 `bu_result.node_results[root].ms` 上
- 非 root loc 在 `bu_result.node_results[local_id].ms` 上
- 若 `used_feasible_intersection=true`，loc 必须在 `feasible_ms` 上
- `candidate_count >= 0`
- `min_delay <= max_delay`
- `skew >= -EPS`

失败时返回 `valid=false` 和清晰 `error_msg`。

---

# 13. locer / 构建接入

TD 不再由 `main.cc` 直接对完整 topology 调用。它由 `locer` 对每个 cluster 调用：

```cpp
common::ClusterDmeInput cluster_input = locer_internal::build_cluster_dme_input(problem, tree, access_node_id);
common::BottomUpResult bu_result = bu::run(problem, cluster_input);

common::TopDownConfig td_config;
td_config.has_root_loc = true;
td_config.root_loc = locer_internal::choose_access_loc_from_ms(
    bu_result.node_results[cluster_input.root_local_id].ms,
    preferred_side);
td_config.root_loc_mode = locer_internal::side_to_root_loc_mode(preferred_side);

common::TopDownResult td_result = td::run(problem, cluster_input, bu_result, td_config);
if (!td_result.valid) {
    return locer_error("TD failed for cluster " + std::to_string(cluster_input.cluster_id)
                       + ": " + td_result.error_msg);
}
```

构建文件：

```text
SRC += src/td.cc
INC += include/td.h
```