

# Router Prompt

你需要实现 `router` 阶段：`locer` 已经为 topology 中每个 node 给出了固定点位，`router` 不再改变 node 位置，也不重新生成 topology，只根据 parent-child 关系生成合法的 Manhattan polyline route。

核心目标：
1. 按 topology 连接每条 parent-child edge。
2. 优先使用 pattern route：I-shape、L-shape、Z-shape。
3. 必须支持每个 node 的上下左右 4 个接入方向，并保证每个方向最多只被 1 条 edge 使用。
4. 每 route 一条边都要立刻做合法性检查并 commit；后续 route 必须避开已经 commit 的 routes。
5. 必须采用 two-stage bottom-up route order：先逐 cluster 从 sinks/internal/access bottom-up route 到该 cluster top，其中 access/bridge/top 可 pattern-first 后 fallback maze；所有 cluster 到 top 后，再将 global/source route bottom-up 汇合到 source。
6. 优先保证合法性，其次减少交叉/拥塞，再考虑线长和 bend 数。

---

## 1. 输入与输出

输入：
- parsed problem，包括 source、sinks、grid/die boundary。
- treer/partreer/locer 产生的 topology nodes、node kind、parent-child edges。
- locer 给出的每个 node 的固定 `loc`。
- 如果 node 有 cluster bbox，也需要读取 bbox 信息。

输出：
- 每条 topology edge 的 Manhattan polyline。
- polyline 起点必须是 parent loc，终点必须是 child loc。
- 中间点只能作为水平/垂直折线的 turning points。
- 不允许改变 locer 给出的 node loc。


source 可以作为虚拟 endpoint 使用 problem source loc，不要求它一定存在于 topology nodes 中。

---

## 1.5 Debug API

router 需要支持类似 `locer` 的可开关 debug 输出。locer 已经有 `debug_enable(bool)`、`debug_file_enable(bool)`、`debug_output(...)` 和 debug 文件输出接口；router 请采用同样风格，便于 main.cc 统一打开/关闭 debug。参考 locer 的 debug 设计：stdout debug 与文件 debug 是两个独立开关，文件输出失败时需要设置错误信息。

建议 API：

```cpp
namespace router {
void debug_enable(bool enable);
void debug_file_enable(bool enable);
void debug_output(const RouterResult& result,
                  const common::Problem& problem,
                  const common::TopologyTree& tree,
                  const common::LocerResult& loc_result);
bool write_debug_route_file(const RouterResult& result,
                            const common::Problem& problem,
                            const common::TopologyTree& tree,
                            const common::LocerResult& loc_result,
                            const std::string& input_path,
                            std::string& error_msg);
}
```

`router.cc` 内部维护：

```cpp
static bool g_debug_enabled = false;
static bool g_debug_file_enabled = false;
```

`run(...)` 结束前：

```cpp
if (g_debug_enabled) {
    debug_output(result, problem, tree, loc_result);
}
if (g_debug_file_enabled) {
    if (!write_debug_route_file(result, problem, tree, loc_result, input_path, error_msg)) {
        result.valid = false;
        result.error_msg = error_msg;
    }
}
```

---

## 2. Route Policy

根据 edge 两端 node kind 选择 routing policy。

### 2.1 LocalClusterPatternOnly

适用边：
- Sink <-> ClusterInternal
- ClusterInternal <-> ClusterInternal
- ClusterInternal <-> ClusterAccess
- ClusterAccess <-> ClusterAccess，如果存在

策略：
- 只使用 pattern route：I-shape、L-shape、Z-shape。
- 不要全局 maze 乱绕。
- 若全部 pattern candidate 失败，可以报错，或只使用 very bounded local maze。
- 如果使用 bounded local maze，只能限制在当前 edge 附近的小窗口内，不允许跨 cluster 或全局绕行。
- 目标是局部、短线、少 bend、不破坏 cluster 内部结构。

### 2.2 ExternalAccessPatternThenMaze

适用边：
- ClusterAccess <-> ClusterBridge
- ClusterAccess <-> ClusterTop
- ClusterBridge <-> ClusterTop

策略：
- 先尝试 I-shape、L-shape、Z-shape。
- 如果全部失败，fallback 到 A* maze routing with bend penalty。
- A* 应倾向形成近似“匚”字形路线：先从 access 离开 cluster bbox，再沿 bbox 外侧走，最后接入 bridge/top。
- A* cost 至少包含：path_length、bend_penalty、crossing_penalty、near_sink_penalty、inside_cluster_penalty、wrong_side_penalty、over_detour_penalty。
- crossing、压 sink、压 node 应该是 forbidden 或极大 penalty。

注意：ExternalAccessPatternThenMaze 虽然是 access/bridge/top 的接入线 policy，但在 route order 上属于对应 cluster 的 Stage A；不是等所有 cluster 完成后才统一 route。

### 2.3 GlobalPatternThenMaze

适用边：
- Source <-> Global
- Global <-> Global
- Global <-> ClusterTop
- Source <-> ClusterTop

策略：
- 先 pattern route，再 fallback A*。
- global route 应像全局主干，bend_penalty 比 access route 更大。
- 尽量保持 global chain 的主轴顺序。
- 不要穿过 cluster bbox 内部，除非目标就是该 cluster top。
- Z-shape 中间轨道优先选择 global chain 主轴附近或 source 轴附近。

---

## 3. Directional Port / Pin Access Constraint

每个 node 有 4 个接入方向：
- UP
- DOWN
- LEFT
- RIGHT

每个方向最多只能被一条 edge 使用。也就是说，同一个 node 的 UP/DOWN/LEFT/RIGHT 四个 port 的 capacity 都是 1。

一条 routed edge 不只是：

```text
parent -> child
```

而是：

```text
(parent, parent_exit_dir) -> (child, child_entry_dir)
```

candidate route 需要从 polyline 推断：
- parent exit direction：看第一段从 parent loc 离开的方向。
- child entry direction：看最后一段进入 child loc 的方向。

示例：
- 最后一段从左向右进入 child，则 child entry direction = LEFT。
- 最后一段从右向左进入 child，则 child entry direction = RIGHT。
- 最后一段从下向上进入 child，则 child entry direction = DOWN。
- 最后一段从上向下进入 child，则 child entry direction = UP。

具体 UP/DOWN 与当前坐标系保持一致。

route candidate 在最终被接受前，必须检查：
- parent exit port 未被占用。
- child entry port 未被占用。

只有 route 被最终选中并 commit 后，才标记这两个 port 已占用。

---

## 4. Preferred Access Direction

对于有 bbox 的 node，preferred access direction 由 bbox 长边方向决定：

- 如果 `bbox_width >= bbox_height`，preferred directions = `{LEFT, RIGHT}`。
- 如果 `bbox_height > bbox_width`，preferred directions = `{UP, DOWN}`。

含义：bbox 平行于长边的方向是 preferred 接入方向。

preferred 判断只看 route 最后一段/第一段的接入方向，与 route 是 I-shape、L-shape、Z-shape 还是 maze 无关。只要最终进入该 node 的方向属于 preferred directions，就认为该 node 使用了 preferred entry。对于从该 node 出发的边，也同理检查 exit direction。

preferred 是 soft constraint，不是硬约束。score 中加入 non-preferred penalty：

```text
score += parent_non_preferred_penalty if parent_exit_dir not in parent.preferred_dirs
score += child_non_preferred_penalty  if child_entry_dir not in child.preferred_dirs
```

建议：
- ExternalAccessPatternThenMaze 的 preferred penalty 较大，使 access/bridge/top 尽量沿 bbox 长边方向接入。
- LocalClusterPatternOnly 的 preferred penalty 较小，避免为了 preferred 过度绕线。
- GlobalPatternThenMaze 的 preferred penalty 中等，同时保持 global 主干简洁。

---

## 5. Pattern Route Candidate Generation

给定两个 endpoint：`a(x1, y1)` 和 `b(x2, y2)`。

### 5.1 I-shape

如果 `x1 == x2` 或 `y1 == y2`，候选为：

```text
(x1, y1) -> (x2, y2)
```

### 5.2 L-shape

枚举两种：

```text
HV: (x1, y1) -> (x2, y1) -> (x2, y2)
VH: (x1, y1) -> (x1, y2) -> (x2, y2)
```

### 5.3 Z-shape

若主方向为 X，枚举若干 `xm`：

```text
(x1, y1) -> (xm, y1) -> (xm, y2) -> (x2, y2)
```

若主方向为 Y，枚举若干 `ym`：

```text
(x1, y1) -> (x1, ym) -> (x2, ym) -> (x2, y2)
```

`xm/ym` 不要只取中点，应枚举：
- midpoint。
- source/global axis 附近 track。
- cluster bbox 外侧 track。
- access preferred side 外侧 track。
- endpoint 附近若干 safe track。

pattern route 不应只枚举几何形状，还要枚举 endpoint ports：

```text
for each available parent_exit_dir:
  for each available child_entry_dir:
    generate I/L/Z candidates whose first segment matches parent_exit_dir
    and whose last segment matches child_entry_dir
```

如果 candidate 的第一段/最后一段方向与枚举 port 不一致，则丢弃。

每个 candidate 都必须先 canonicalize：
- 删除重复点。
- 删除零长度 segment。
- 合并同向连续 segment。

然后再做合法性检查和打分。

---

## 6. Incremental Legality Check

router 必须是 incremental routing。每 route 一条 edge，都要基于当前已经 commit 的 routes 检查合法性。只有合法 candidate 才能 commit；commit 后，该 route 的 segments 成为后续 route 的障碍。

每条 candidate route 至少检查以下规则。

### 6.1 不越界

- 所有点必须在 grid/die boundary 内。
- 所有 segment 必须是水平或垂直线段。

### 6.2 不压到其他 node

route 的每个 segment 都不能穿过或压到任何非当前 edge 两端的 node。

node 先按点障碍处理：
- sink loc 是障碍点。
- internal/access/bridge/top/global loc 是障碍点。
- source loc 是障碍点。

如果 node 有 bbox，则 bbox 内部也视为障碍区域。route 不能穿过其他 node 的 bbox。

例外：
- route 可以从当前 edge 的起点 node 出发。
- route 可以进入当前 edge 的终点 node。
- 但不能继续穿过 endpoint 后方。
- 如果 endpoint 有 bbox，只允许从选定 entry/exit direction 接入，不允许从 bbox 内部横穿。

### 6.2.5 LocalClusterPatternOnly 的合法性特例

对于 LocalClusterPatternOnly，合法性检查必须允许 route 位于自己的 cluster bbox 内。cluster 内部的 sink/internal/access 本来就在 bbox 内，如果把自己的 cluster bbox 当成 forbidden obstacle，会导致最短的 sink-internal route 也全部 failed。

LocalClusterPatternOnly 的特殊规则：

- 当前 edge 的 parent node 和 child node 不作为障碍物。
- route 允许从 parent loc 出发，也允许进入 child loc。
- 当前 edge 所属 cluster 的 bbox 不作为 forbidden bbox。
- route 允许完全位于自己的 cluster bbox 内。
- 只禁止压到非当前 edge endpoint 的 node。
- 只禁止穿过其它 cluster 的 bbox。
- 只禁止和已经 committed routes 发生非法 crossing/overlap。

不要因为 route segment touching parent/child loc 就判定 `hit_node`。
不要因为 local cluster route 位于 related cluster bbox 内就判定 `hit_bbox`。

### 6.3 不和已 commit routes 非法交叉

candidate 的每个 segment 都要和所有已 commit route segments 做 intersection 检查。

如果水平段和垂直段相交：
- 若交点不是双方共享 endpoint，则 illegal crossing。
- 若交点是当前 candidate 的合法 endpoint，可以允许。
- 若交点是已有 route 的中间点，默认 illegal，除非作业明确允许 T-junction。

如果两个 segment 共线：
- 只共享 endpoint 可以允许。
- 有长度重叠默认 illegal overlap，除非作业明确允许共线复用。
- 为安全起见，默认 forbidden overlap。

### 6.4 不自交

candidate 内部 segment 两两检查：
- 非相邻 segment 不能相交。
- 相邻 segment 只能共享连接点。
- 共线回退造成 overlap 是 illegal。

### 6.5 port capacity 合法

candidate 第一段决定 parent exit direction。candidate 最后一段决定 child entry direction。

必须检查：
- parent 对应 direction port 未被占用。
- child 对应 direction port 未被占用。

### 6.6 不形成 topology 外的隐式连接

candidate 不允许碰到非本 edge 的 node 或 route 中间点，避免隐式短接。

也就是说：
- 除了当前 edge 两端，route 不应该与任何 node loc 接触。
- 除了允许的共享 endpoint，route 不应该与已有 route 接触。

---

## 7. Scoring

所有合法 candidate 使用 score 选择最优：

```text
score = wirelength
      + bend_penalty * bends
      + congestion_penalty
      + bbox_penalty
      + wrong_side_penalty
      + parent_non_preferred_penalty
      + child_non_preferred_penalty
      + over_detour_penalty
```

非法情况，例如压 node、压 sink、越界、非法交叉、port 被占用，应该直接判 illegal，不参与 scoring。

A* maze routing 只在 policy 允许时调用。A* 的 neighbor 是四方向 grid movement，需要记录前一个方向以计算 bend penalty。

---

## 8. Route Order

router 必须采用 two-stage bottom-up route order。这是硬要求，不是建议。

核心原则：

1. 先对每个 physical cluster 内部单独 bottom-up route，一直 route 到该 cluster 的 top。
2. in-cluster route 包含两部分：
   - cluster local DME edges：sink/internal/access；
   - cluster access-to-top edges：access/bridge/top。
3. `access -> bridge -> top` 或 `access -> top` 这部分仍属于该 cluster 的 in-cluster routing stage，应在切换到 global/source 之前完成。
4. 所有 cluster 都 route 到 top 之后，再 route global/source 主干。
5. global/source 主干也必须 bottom-up，从各个 cluster top 往 global 汇合，最后 route 到 source。

不要采用 global-first。不要先 route source/global/top 主干再回头 route cluster 内部短线。否则主干线会提前占用局部通道，导致本应简单成功的 sink-internal route failed。

### 8.1 Stage A：per-cluster in-cluster bottom-up routing to top

对每个 physical cluster / ClusterTop，单独收集该 cluster 内部需要 route 到 top 的 edges。

第一类是 LocalClusterPatternOnly edges：

- Sink <-> ClusterInternal
- ClusterInternal <-> ClusterInternal
- ClusterInternal <-> ClusterAccess
- ClusterAccess <-> ClusterAccess，如果存在

第二类是该 cluster 自己的 access-to-top edges，使用 ExternalAccessPatternThenMaze policy：

- ClusterAccess <-> ClusterBridge
- ClusterAccess <-> ClusterTop
- ClusterBridge <-> ClusterTop

注意：这些 access/bridge/top 边虽然使用 ExternalAccessPatternThenMaze policy，但在 route order 上仍属于当前 cluster 的 Stage A。也就是说，一个 cluster 内部必须先从 sink/internal/access bottom-up route 到 access，再继续 route access/bridge/top，直到该 cluster top 被接通。

也就是说，对 cluster 内部 DME tree，优先 route 靠近 sink 的短边，再逐层向 access/root 汇合；随后立即 route 该 cluster 的 access/bridge/top，使该 cluster 在 Stage A 结束时已经完整接到 top。

推荐 local edge sort key：

```text
cluster_id first
within each cluster:
  policy_priority:
    LocalClusterPatternOnly        = 0
    ExternalAccessPatternThenMaze  = 1   // only this cluster's access/bridge/top edges
  for LocalClusterPatternOnly:
    smaller subtree_depth_from_sink first
    or larger distance_to_access/root later
    or simply leaf/sink-adjacent edges first
  for this cluster's ExternalAccessPatternThenMaze:
    access-side edge first
    bridge-to-top later
    shorter Manhattan distance first
  then lower edge_id for deterministic order
```

注意：Stage A 是"每个 cluster 内部一直 route 到 top"，不是只 route 到 access，也不是把所有 sink-adjacent edges 混在一起后再 route 所有 internal edges。推荐流程是：

```text
for each cluster in deterministic cluster_id order:
    route all LocalClusterPatternOnly edges in this cluster bottom-up
    route this cluster's access/bridge/top edges bottom-up until top is connected
```

这样每个 cluster 内部的短线和接入 top 的线都可以先完成，避免后续 global/source route 影响 cluster 内部可行性。

access/bridge/top 的 route 策略仍然是 pattern-first：先尝试 I-shape、L-shape、Z-shape；如果失败，可以 fallback 到 A* maze routing with bend penalty。maze fallback 只用于该 cluster 的 access/bridge/top 接入，不要用于 sink/internal 局部短线的全局乱绕。

### 8.2 Stage B：global bottom-up routing to source

当所有 cluster 都已经在 Stage A 中 route 到各自 top 后，再 route 全局边。

Stage B 只处理跨 cluster 的 global/source 主干，不再处理 access/bridge/top。access/bridge/top 已经在各自 cluster 的 Stage A 中完成。

GlobalPatternThenMaze edges 的顺序必须从 cluster/top 侧向 source 汇合：

- ClusterTop <-> Global
- Global <-> Global
- Source <-> Global
- Source <-> ClusterTop

应先 route farther_from_source 的 global/top 边，source-adjacent edge 最后 route。

`Source <-> ClusterTop` 仅在 topology 中 source 直接连接 ClusterTop 时适用；一般情况是 `ClusterTop -> Global -> ... -> Source`。

推荐 global edge sort key：

```text
policy_priority:
  GlobalPatternThenMaze = 2

within GlobalPatternThenMaze:
  larger topology_depth_from_source first
  source-adjacent edge last
  shorter Manhattan distance first within same depth
  then lower edge_id
```

如果实现里已有 `sort_edges_by_route_order()`，必须改为 `sort_edges_by_two_stage_bottom_up_route_order()` 或等价逻辑，并确保：

```text
for each cluster, all LocalClusterPatternOnly edges are routed before that cluster's access/bridge/top edges
for each cluster, access/bridge/top edges are routed before any global/source edge
all clusters must be connected to their top before Stage B starts
global/source edges are routed bottom-up toward source, not source-first
```

---

## 9. Main Flow

实现主流程：

```text
validate problem/tree/loc_result
build_obstacles_from_nodes_and_bboxes()
build_edge_list_from_topology()
classify_edges_into_route_policies()
sort_edges_by_two_stage_bottom_up_route_order()
assert route_order is two-stage bottom-up: for each cluster route LocalClusterPatternOnly then that cluster's access/bridge/top ExternalAccessPatternThenMaze until top; after all clusters reach top, route GlobalPatternThenMaze bottom-up to source
initialize RouterResult and per-edge debug records

for edge in sorted_edges:
    candidates = generate_pattern_candidates(edge)
    record_debug(edge, stage="pattern", candidate_count=candidates.size())
    legal_candidates = []

    for cand in candidates:
        cand = canonicalize(cand)
        if check_legality(cand, committed_routes, occupied_ports, obstacles):
            legal_candidates.push_back(cand)

    if legal_candidates.empty() and policy_allows_maze(edge):
        maze_candidates = astar_maze_candidates(edge)
        record_debug(edge, stage="maze", candidate_count=maze_candidates.size())
        for cand in maze_candidates:
            cand = canonicalize(cand)
            if check_legality(cand, committed_routes, occupied_ports, obstacles):
                legal_candidates.push_back(cand)

    if legal_candidates.empty():
        record_debug_failure(edge, reason)
        report_route_failure(edge)
        continue or fail hard according to existing project style

    best = min_score(legal_candidates)
    record_debug_best(edge, best)
    commit_route(best)
```

`commit_route(best)` 必须：
1. 保存该 edge 的 polyline。
2. 把 polyline segments 加入 committed_routes。
3. 标记 parent exit port / child entry port 已占用。

最终 router 应是一个 two-stage bottom-up、pattern-first、incremental legality-driven Manhattan router：先逐 cluster 从 sinks bottom-up route 到 top，其中 access/bridge/top 可 pattern-first 后 fallback maze；所有 cluster 到 top 后，再将 global/source route bottom-up 汇合到 source。

---

## 10. Debug 输出

router 的 debug 输出用于检查每条 edge 为什么选择某条 route，以及为什么其它 candidate 被拒绝。debug 不改变 routing 结果，不影响 evaluator 输出。

`debug_output()` 打印：

```text
[ROUTER] valid/error_msg/num_edges/num_routed_edges
route_order=[...]
for each route policy:
  policy_name edge_count routed_count failed_count

for each edge in route order:
  edge_id parent child parent_class child_class policy
  parent_loc child_loc
  selected_shape = I | L | Z | MAZE | FAILED
  parent_exit_dir child_entry_dir
  parent_port_available child_port_available
  used_preferred_parent used_preferred_child
  candidate_count pattern_candidate_count maze_candidate_count legal_candidate_count
  selected_score wirelength bends
  legality_status failure_reason
  polyline=(x0,y0)->(x1,y1)->...
```

如果 candidate 被拒绝，建议在 debug 中统计拒绝原因数量：

```text
reject_stats:
  out_of_boundary=<N>
  non_manhattan=<N>
  hit_node=<N>
  hit_bbox=<N>
  crossing_committed_route=<N>
  overlap_committed_route=<N>
  self_intersection=<N>
  port_occupied=<N>
  implicit_connection=<N>
```

当 A* fallback 被调用时，额外打印：

```text
maze_used=true
maze_expanded_nodes=<N>
maze_best_cost=<cost>
maze_failed_reason=<reason if failed>
```

---

## 11. Debug route 文件输出

router 需要像 locer 一样支持 debug 文件输出。该输出只用于检查 route，不是最终 `result/sample<k>_solution.txt`，不由 evaluator 读取。

开关：

```cpp
router::debug_file_enable(true);
```

关闭时不写任何 `route/*.txt` 文件。

文件路径：

```text
route/sample<k>_route.txt
```

`<k>` 从输入文件名推导：

```text
samples/sample1.txt  -> route/sample1_route.txt
sample2.txt          -> route/sample2_route.txt
```

若 `input_path` 为空或无法解析 sample 名，则使用：

```text
route/route_debug.txt
```

写文件前必须确保 `route/` 目录存在；不存在则创建。

推荐格式：

```text
# ROUTER_DEBUG_ROUTE v1
# valid=<0/1>
# num_edges=<E>
# columns: edge_id parent child parent_class child_class policy selected_shape parent_exit_dir child_entry_dir score wirelength bends pattern_candidate_count maze_candidate_count legal_candidate_count failure_reason point_count points...
edge <edge_id> <parent> <child> <parent_class> <child_class> <policy> <selected_shape> <parent_exit_dir> <child_entry_dir> <score> <wirelength> <bends> <pattern_candidate_count> <maze_candidate_count> <legal_candidate_count> <failure_reason> <point_count> <x0> <y0> <x1> <y1> ...
```

示例：

```text
edge 7 20 12 bridge access ExternalAccessPatternThenMaze Z LEFT RIGHT 42.5 18 2 24 0 3 OK 4 34 10 28 10 28 18 22 18
edge 8 31 20 top bridge ExternalAccessPatternThenMaze MAZE DOWN UP 91.0 27 4 18 1 1 OK 6 30 3 30 8 26 8 26 12 24 12 24 16
```

要求：

- 按最终 two-stage bottom-up route order 输出 edge：先逐 cluster 从 local edges route 到 access，再 route 该 cluster 的 access/bridge/top 直到 top；所有 cluster 到 top 后，再输出 global/source edges bottom-up to source。
- 每条 topology edge 输出一行。
- 成功 route 的 edge 输出完整 polyline。
- 失败 edge 也输出一行，`selected_shape=FAILED`，`failure_reason` 写清楚。
- `failure_reason` 不要包含空格；可使用 `NO_LEGAL_PATTERN`、`NO_LEGAL_MAZE`、`PORT_OCCUPIED`、`HIT_NODE` 等枚举字符串。
- 文件输出失败时返回 false，并设置 `error_msg`。
- debug 文件输出不得替代 stdout debug。

---

## 12. Debug 记录结构建议

如果 `common.h` 中还没有 router 结果结构，可以补充最小 debug 字段：

```cpp
struct RouterEdgeDebug {
    int edge_id = -1;
    int parent = -1;
    int child = -1;
    std::string parent_class;
    std::string child_class;
    std::string policy;
    std::string selected_shape;
    std::string parent_exit_dir;
    std::string child_entry_dir;

    int pattern_candidate_count = 0;
    int maze_candidate_count = 0;
    int legal_candidate_count = 0;

    double selected_score = 0.0;
    double wirelength = 0.0;
    int bends = 0;

    bool routed = false;
    std::string failure_reason;
    std::vector<common::SegmentPoint> polyline;

    std::map<std::string, int> reject_stats;
};

struct RouterResult {
    bool valid = false;
    std::string error_msg;
    std::vector<RouterEdgeDebug> edge_debugs;
    // existing route solution fields can stay here
};
```

debug 信息应来自真实 routing 过程，不要在输出阶段重新推断过多内容。每次生成 candidate、拒绝 candidate、选择 best candidate、commit route 时，都应同步更新对应 edge 的 debug record。

---

## 13. main.cc / 构建接入

`main.cc` 中建议与 locer 一样显式打开 debug：

```cpp
router::debug_enable(true);
router::debug_file_enable(true);
common::RouterResult route_result = router::run(problem, tree, loc_result, argv[1]);
if (!route_result.valid) {
    std::cerr << "ROUTER error: " << route_result.error_msg << "\n";
    return 1;
}
```

构建文件：

```text
SRC += src/router.cc
INC += include/router.h
```