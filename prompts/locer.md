# locer.md — Cluster DME + Congestion-Aware Outer Location Assignment 提示词

请实现 `locer` 模块：接收 `treer` 生成的 topology，对每个 cluster 内部单独执行 DME BU/TD，确定 `sink / internal / access` 的 loc；随后用 congestion-aware + skew-aware heuristic 确定 `bridge / top / global / source` 的 loc。

注意：`access` 仍属于 cluster 内部节点，不要求在 cluster bbox 外；`top / global` 需要满足 bbox 外侧约束。`bridge` 不要求严格 outside bbox，只需尽量避开 congestion。

`locer` 只负责 node location assignment，不负责实际 route polyline、不负责 detour、不负责 buffer insertion、不负责 writer 输出 solution。

---

# 0.5 整数坐标契约

最终写入 `common::LocerNodeResult::loc` 的坐标必须是整数 grid 坐标。

- Locer 内部可以继续使用 floating point 做 DME、candidate generation、scoring、delay/profile 估计。
- 在 commit 每个最终 locer output 之前，所有非 sink node 必须 snap 到最近整数 grid point，并 clamp 到 die 范围内。
- Sink node 必须保持 input sink 的精确整数坐标，不允许被 DME midpoint 或 snap 改动。
- Final validation 必须拒绝任何非整数 `result.node_results[*].loc`。
- 这个整数化是 locer 的输出边界规范，不应改变 topology、不应改变候选评分公式、不应改 route/writer 逻辑。

---

# 1. 输入输出

输入：

```cpp
common::Problem problem;
common::TopologyTree tree;
```

输出：

```cpp
common::LocerResult result;
```

核心结果：

```text
result.node_results[node_id].loc
```

每个 topology node 都必须被赋予一个 loc。

---

# 2. 职责边界

必须做：

1. 新增/实现 `locer.h`、`locer.cc`。
2. 使用 treer 已生成的 topology，不修改 topology。
3. 识别 node class：`sink / internal / access / bridge / top / global / source`。
4. 对每个 cluster 内部子树调用 DME：
   - BU：从 sinks/internal 向上算到 access；
   - TD：从 access 向下确定 sink/internal/access loc。
5. 用 congestion-aware + skew-aware heuristic 确定：
    - `bridge` loc；
    - `top` loc；
    - `global` loc：**order-constrained placement，不再强依赖 treer 给出的具体 global 坐标，只保持 global chain 在 primary axis 上的顺序**；
    - `source` loc。
6. `access` 不要求在 cluster bbox 外；`bridge` 不要求严格 outside bbox，只需尽量避开 congestion；`top / global` 的 loc 必须尽量在相关 cluster bbox 外。若 `top` 明确属于某个 cluster，则必须严格在该 cluster bbox 外；`global` 至少不能落入任何单个 cluster bbox 内部。
7. 提供可开关 debug 输出，包括 stdout debug 和 `loc/sample<k>_loc.txt` 文件输出。

不要做：

- 不生成 route polyline。
- 不做 detailed routing / maze routing / DAG routing。
- 不做 detour 补偿。
- 不插 buffer。
- 不写 `result/sample<k>_solution.txt`。
- 不调用 evaluator。

---

# 3. API

`locer.h`：

```cpp
#pragma once
#include "common.h"
#include <string>

namespace locer {
using LocerResult = common::LocerResult;
using LocerNodeResult = common::LocerNodeResult;

void debug_enable(bool enable);
void debug_file_enable(bool enable);
void debug_output(const LocerResult& result,
                  const common::Problem& problem,
                  const common::TopologyTree& tree);
bool write_debug_loc_file(const LocerResult& result,
                          const common::Problem& problem,
                          const common::TopologyTree& tree,
                          const std::string& input_path,
                          std::string& error_msg);
LocerResult run(const common::Problem& problem,
                const common::TopologyTree& tree,
                const std::string& input_path = "");
}  // namespace locer
```

`locer.cc` 内部维护：

```cpp
static bool g_debug_enabled = false;
static bool g_debug_file_enabled = false;
```

---

# 4. common.h 最小数据结构

优先复用已有 `common.h`。若缺少 locer 结果结构，请补齐以下最小字段。

```cpp
struct LocerNodeResult {
    int node_id = -1;
    bool valid = false;

    SegmentPoint loc;
    std::string node_class;
    int cluster_id = -1;

    std::string loc_mode;
    double loc_score = 0.0;
    int candidate_count = 0;

    bool inside_related_bbox = false;
    double congestion_penalty = 0.0;
    double lshape_penalty = 0.0;
    double wire_est_to_parent = 0.0;

    std::vector<double> sink_delays_to_node;
    double min_sink_delay_to_node = 0.0;
    double max_sink_delay_to_node = 0.0;
    double skew_to_node = 0.0;
    double skew_penalty = 0.0;
};

struct LocerResult {
    std::vector<LocerNodeResult> node_results;  // index == tree node id
    bool valid = false;
    std::string error_msg;
};
```

字段语义：

- `loc` 是唯一核心输出。
- `loc_mode` 用于 debug，例如：
  - `DME_ACCESS_ROOT`
  - `DME_INTERNAL`
  - `DME_SINK`
  - `BRIDGE_CONGESTION_AWARE`
  - `TOP_CONGESTION_AWARE`
  - `GLOBAL_CONGESTION_AWARE`
  - `SOURCE_FIXED_OR_NEAREST`
- `inside_related_bbox=true` 表示该 outer loc 违反或接近 bbox 约束，应在 debug 中突出。
- `sink_delays_to_node` 存放该 node subtree 内所有 sinks 到该 node 的 estimated delay。
- `min_sink_delay_to_node / max_sink_delay_to_node / skew_to_node` 由 `sink_delays_to_node` 统计得到。
- `skew_penalty` 用于 loc scoring，表示该候选 loc 会造成或放大的 subtree skew 代价。

---

# 5. 总体流程

```text
run(problem, tree, input_path):
    validate input
    result.node_results.resize(tree.nodes.size())

    classify all nodes
    compute subtree sink sets
    identify physical clusters / ClusterTop nodes
    group access nodes by physical cluster / ClusterTop
    compute one cluster bbox for every physical cluster from all sinks under that cluster
    initialize empty per-node subtree sink delay profiles

    // Stage 0: pre-compute cluster_preferred_side for every physical cluster
    // IMPORTANT: Stage 0 must use the loc/cx/cy already stored in treer input nodes as geometric guidance.
    // Do not wait for locer placement result; preferred_side is decided before locer assigns locs.
    for each physical cluster / ClusterTop:
        resolve external anchor from treer node loc/cx/cy first, then source/die center fallback
        compute preferred_side using anchor-to-bbox-center direction
        cache to cluster_preferred_side[cluster_id]

    for each physical cluster / ClusterTop:
        access_list = all access nodes in this physical cluster
        preferred_side = cluster_preferred_side[cluster_id]  // from Stage 0

        for each access node in access_list:
            extract access DME subtree rooted at access
            run cluster-local DME BU
            choose access loc from access.ms using this cluster's shared preferred_side
            run cluster-local DME TD to assign internal/sink loc
            write locs into result

        after all access-subtrees in this cluster finish:
            compute delay profiles for sink/internal/access

     place bridge nodes with congestion-aware + skew-aware candidates
     bridge 继承所在 cluster 的 cluster_preferred_side[cluster_id] from Stage 0
    after each bridge placement:
        update delay profile for that bridge

     place top nodes with congestion-aware + skew-aware candidates
     top 继承所在 cluster 的 cluster_preferred_side[cluster_id] from Stage 0
    after each top placement:
        update delay profile for that top

    place global nodes with order-constrained placement
    从 treer/topology 中提取 global chain 顺序与 primary axis
    locer 重新选择 global loc，但必须保持 global nodes 在 primary axis 上的相对顺序
    after each global placement:
        update delay profile for that global

    place source node
    after source placement:
        update delay profile for source

    final legality check
    result.valid = true
    if debug enabled: debug_output(...)
    if debug file enabled:
        write loc/sample<k>_loc.txt
        if write failed: result.valid=false; result.error_msg=...
    return result
```

要求：

- DME 仅作用于 `sink / internal / access`。
- `bridge / top / global / source` 不使用 TRR/MS。
- outer placement 只使用 bbox、已有 loc、topology 关系、congestion score、wire estimate 和 skew-aware delay profile。
- 每个 node 都必须维护其 subtree sinks 到该 node 的 estimated delays，后续 bridge/top/global placement根据这些 delays 尽量 balance skew。
- cluster id / bbox / preferred_side 的单位是 physical cluster / ClusterTop，不是 access node。
- 一个 physical cluster 可以包含多个 access；这些 access 分别作为 DME root 调用 BU/TD，但共享同一个 cluster bbox 和 preferred_side。
- 不允许因为一个 cluster 内有多个 access 就生成多个 cluster id 或多个 preferred_side。
- preferred_side 由 Stage 0 预判，access / bridge / top 全部继承，不允许在 placement 阶段各自重新估计。

---

# 5.5 Stage 0：cluster preferred_side 预判

在任何 access / bridge / top loc 选择之前，locer 必须先为每个 physical cluster / ClusterTop 预判一个 cluster 级 `preferred_side`，并缓存到：

```text
cluster_preferred_side[cluster_id] = LEFT | RIGHT | BOTTOM | TOP
```

后续属于同一个 cluster 的 `access`、`bridge`、`top` 都必须继承这个 `preferred_side`。不要在 access / bridge / top placement 阶段各自重新估计 side；placement scoring 只在该 side 对应的候选点中选最优点。只有该 side 全部候选非法时，才允许 fallback 到其它 side，并必须 debug 标记。

**重要：Stage 0 的 preferred_side 必须以 treer 传入的 topology nodes 中已有的几何位置作为指导。** 这些位置包括 treer 输出/解析进 `common::TopologyTree` 的 node loc、`cx/cy`、estimated loc、debug center 或等价字段。Stage 0 发生在 locer placement 之前，因此不要依赖 locer 后续生成的 `result.node_results[parent].loc`。如果 parent/global/top 在 treer 中已有位置估计，必须优先使用该位置来判断 anchor 方向。

## 5.5.1 preferred_side 的 external anchor

对每个 `CLUSTER_TOP` / physical cluster，先找到外部接入 anchor：

```text
resolve_external_anchor(cluster_top_node):
    parent = tree.nodes[cluster_top_node.parent] if parent exists

    // Stage 0 runs before locer assigns result locs, so use treer-provided node geometry first.
    if parent exists and parent has treer-provided loc / cx/cy / estimated center:
        return parent treer loc / cx/cy
    else if nearest ancestor GLOBAL/TOP/SOURCE has treer-provided loc / cx/cy / estimated center:
        return ancestor treer loc / cx/cy
    else if cluster_top_node is root or parent does not exist:
        return source.loc from problem if available
    else if problem has source loc:
        return source.loc
    else:
        return die center
```

说明：

- Stage 0 必须优先使用 treer 传入的 parent/global/top node 几何位置，例如 `node.loc`、`node.cx/cy`、estimated loc、debug center 或等价字段。
- 不要把 `result.node_results[parent].loc` 作为 Stage 0 的主要依据，因为 Stage 0 执行时 locer 尚未完成 parent/global/top placement。
- root cluster 或找不到有效 treer anchor 时，才使用 problem 中的 source loc。
- 若 source loc 也不可用，最后 fallback 到 die center。
- 这个 anchor 只用于预判 side，不直接决定最终 loc。

## 5.5.2 preferred_side 判定公式

对 cluster bbox：

```text
bbox_center = ((bbox_lx + bbox_ux) / 2, (bbox_ly + bbox_uy) / 2)
dx = anchor.x - bbox_center.x
dy = anchor.y - bbox_center.y
```

使用主导轴判断：

```text
if abs(dx) >= abs(dy):
    preferred_side = RIGHT if dx >= 0 else LEFT
else:
    preferred_side = TOP if dy >= 0 else BOTTOM
```

加入 tie 处理，避免方向抖动：

```text
tie_eps = max(1.0, 0.05 * max(bbox_width, bbox_height))

if abs(abs(dx) - abs(dy)) <= tie_eps:
    choose side among the two dominant candidates by lower estimated congestion / fewer bbox crossing risk
    if still tied, keep deterministic order: TOP, RIGHT, BOTTOM, LEFT
```

注意：congestion 只能在 tie 时辅助选择，不能在非 tie 情况下推翻 anchor 主导方向。

## 5.5.3 preferred_side 的继承规则

```text
for every node under the same physical cluster:
    node.preferred_side = cluster_preferred_side[cluster_id]
```

具体要求：

- `ClusterAccess`：在自己的 access.ms / DME segment 上选点时，优先选靠近 `preferred_side` 的点。
- `ClusterBridge`：不要求严格 outside bbox，但候选点必须优先落在 `preferred_side` corridor / 浅层 corridor；access -> bridge 的第一段应沿 `preferred_side` 方向。
- `ClusterTop`：必须优先放在 `preferred_side` 对应的 bbox 外侧。
- `GLOBAL/SOURCE`：不继承 cluster preferred_side。
- multi-cluster `top/global`：若覆盖多个 clusters 且这些 clusters 的 preferred_side 相同，则继承该 shared side；否则使用 union bbox scoring，但增加 strong side-consistency penalty。

## 5.5.4 debug 输出

每个 cluster 必须输出：

```text
cluster_id
cluster_top_node_id
bbox
external_anchor
anchor_source = parent_treer_loc | ancestor_treer_loc | source | die_center
dx dy
preferred_side
tie_used
tie_candidates
```

每个 access / bridge / top 必须输出：

```text
inherited_preferred_side
actual_side
fallback_side_used
```

---

# 6. Cluster 内部 DME

对每个 physical cluster / ClusterTop，先收集该 cluster 下的所有 access nodes。每个 access node 是一个独立的 DME root，但不是一个独立 cluster。

对每个 access node：

```text
access_dme_subtree = descendants(access) restricted to sink/internal/access
```

一个 physical cluster 可以包含多个 `access_dme_subtree`。这些 access-subtrees 分别调用 BU/TD，但共享同一个：

```text
cluster_id
cluster_bbox
preferred_side
```

执行：

```text
BU: sinks/internal -> access
TD: access -> internal/sink
```

access loc 不一定取 `ms` midpoint，而是根据该 physical cluster 共享的 `preferred_side` 选择：

```text
if preferred_side == LEFT:   choose point in access.ms with minimum x
if preferred_side == RIGHT:  choose point in access.ms with maximum x
if preferred_side == BOTTOM: choose point in access.ms with minimum y
if preferred_side == TOP:    choose point in access.ms with maximum y
if unknown:                  choose midpoint(access.ms)
```

这样可以让 cluster 内部仍保持 DME 可行，同时让 access 更靠近后续 bridge/top 的接入方向。

注意：`access.loc` 只需要位于 `access.ms` 上，不需要 outside cluster bbox。access 是 cluster 内部 DME root，不是 outer node。

关键约束：不要把 `access_dme_subtree` 当成 cluster。`ClusterDmeInput.root_local_id` 是 access，只说明 BU/TD 的 DME root 是 access；locer 的 cluster 分组、bbox 和 preferred_side 仍然来自 physical cluster / ClusterTop。

---

# 7. Cluster bbox

cluster bbox 定义为 physical cluster / ClusterTop 内所有 sinks 的 bounding box，而不是单个 access subtree 的 bbox：

```text
xmin = min sink.x
xmax = max sink.x
ymin = min sink.y
ymax = max sink.y
```

严格在 bbox 外：

```text
outside_bbox(p, bbox) = p.x < xmin or p.x > xmax or p.y < ymin or p.y > ymax
```

边界点算在 bbox 内，不算 outside。

bbox outside 约束从 `top` 开始生效，不作用于 `sink/internal/access/bridge`：

```text
sink/internal/access/bridge: allowed anywhere, as long as DME loc is valid
top:                        must be strictly outside related cluster bbox
global:                     must not be inside any single cluster bbox
source:                     no hard bbox outside constraint
```

对于 `top`：

```text
related_bbox = subtree sinks 的 bbox
loc 必须 outside related_bbox
```

对于 `bridge`：

```text
bridge 不要求严格 outside bbox。其 candidate 不做 bbox 硬过滤；inside_or_on_bbox 从 INF 降级为 soft penalty，只在 congestion_penalty 中间接体现。
```

对于 `global`：

```text
related_bbox = 其 child subtrees 覆盖的 cluster bboxes 的 union
loc 应尽量 outside union bbox；若全局结构无法满足，至少不能落入任何单个 cluster bbox 内部
```

对于 `source`：

```text
若 problem 中给了 source 坐标，则固定使用 source 坐标；否则选择靠近所有 global/top 的中心位置，不做 bbox outside 硬约束。
```

---

# 8. subtree sink delay profile

locer 必须为每个 node 维护一个 delay profile：

```text
sink_delays_to_node[node] = list of estimated delay from every sink in node's subtree to this node
```

该 profile 用于 skew-aware placement。它不是最终 signoff delay，只是 loc assignment 阶段的估计模型。

skew balance 的核心原则：**每次选择 loc 时，先找每个 child subtree 中最大的 sink delay，并优先让左右/多个 child 的最大 sink delay 尽量接近**。

数据结构仍保留所有 sinks 的 delay，便于 debug 和后续更细的统计；但 loc scoring 的主项使用 child subtree 的 worst sink delay。

基本递推：

```text
if node is sink:
    sink_delays_to_node = [0]

else:
    sink_delays_to_node = []
    for each child c:
        edge_delay = estimated_edge_delay(node.loc, c.loc)
        for d in sink_delays_to_node[c]:
            sink_delays_to_node.push_back(d + edge_delay)
```

统计：

```text
min_sink_delay_to_node = min(sink_delays_to_node)
max_sink_delay_to_node = max(sink_delays_to_node)
skew_to_node = max_sink_delay_to_node - min_sink_delay_to_node
```

对任意候选 parent loc，必须先计算每个 child 的 worst delay：

```text
child_worst_delay_after_edge(c) = max_sink_delay_to_node[c] + estimated_edge_delay(parent_candidate_loc, loc[c])
```

对 binary tree：

```text
left_worst  = max_sink_delay_to_node[left]  + edge_delay(parent_candidate, left.loc)
right_worst = max_sink_delay_to_node[right] + edge_delay(parent_candidate, right.loc)

balance_skew = abs(left_worst - right_worst)
```

对多 child outer node：

```text
balance_skew = max(child_worst_delay_after_edge) - min(child_worst_delay_after_edge)
```

`estimated_edge_delay(parent, child)` 当前使用几何曼哈顿距离：

```text
estimated_edge_delay = L1(parent.loc, child.loc)
```

若后续需要加入 buffer / unit wire delay，可在这里统一替换，但 locer 当前不插 buffer。

对 cluster 内部 DME：

```text
cluster 内部优先使用 BU assigned edge_to_left/right 作为 parent-child delay；
若 assigned edge 不可用，再 fallback 到 L1(parent.loc, child.loc)。
```

对 outer nodes：

```text
bridge/top/global/source 的 delay profile 根据已经放好的 child loc 递推计算。
```

注意：

- profile 必须按 subtree sinks 收集，不要求记录 sink id，但 debug 时建议同时能输出 count/min/max/skew。
- 若 child loc 尚未确定，则不能计算该候选的完整 skew score，应使用 parent/child estimated loc 或跳过该候选。
- locer 不通过 profile 修改 topology，只用它给 loc candidate 打分。

---

# 9. preferred_side 决定（Stage 0 预判）

每个 cluster 的 preferred_side 在 Stage 0 中预判完成，不再在 access / bridge / top placement 阶段重新估计。

具体预判逻辑见 §5.5：优先使用 treer 传入的 parent/global/top node loc/cx/cy 作为 external anchor，通过 external anchor 和 cluster bbox center 的方向关系决定 preferred_side，tie 时按 congestion 辅助选择，最终确定 `cluster_preferred_side[cluster_id]`。

access / bridge / top 全部继承该值，不单独估计。

---

# 10. bridge / top congestion-aware placement

`bridge` 和 `top` 的 placement 逻辑必须彻底分开。`bridge` 不考虑 bbox center，避免 bridge 被吸到 cluster 中心导致 access -> bridge 线穿过 congestion 区域；`top` 作为 cluster 对外代表点，可以使用 bbox center 的 preferred-side 投影。

## 10.1 bridge placement

`bridge` 是多个 access / child subtree 的局部汇合点，不是 cluster 对外代表点。因此 bridge 不要求 outside bbox，也不应该被 bbox center 吸到 cluster 中心。

bridge placement 的原则：

```text
bridge 继承所在 cluster 的 Stage 0 preferred_side
bridge candidate 优先位于 selected_side corridor / bbox 浅层区域 / bbox 外侧
bridge 不使用 bbox center 作为候选坐标来源
bridge 不允许 deep-inside bbox，除非所有 corridor candidates 都非法，此时作为高 penalty fallback
access -> bridge 的第一段应沿 preferred_side 方向
```

bridge 的 candidate 坐标来源只允许来自：

```text
access.loc.x / access.loc.y
child loc x/y
sibling access/child loc x/y
parent/top estimated loc x/y
selected_side bbox boundary coordinate
selected_side outside coordinate with margin in {1, 2, 4, 8}
```

bridge 明确禁止把以下项作为 candidate anchor：

```text
bbox center x
bbox center y
```

bridge corridor 定义：

```text
corridor_width = max(2.0, 0.15 * max(bbox_width, bbox_height))

LEFT corridor:
    p.x <= xmin + corridor_width
RIGHT corridor:
    p.x >= xmax - corridor_width
BOTTOM corridor:
    p.y <= ymin + corridor_width
TOP corridor:
    p.y >= ymax - corridor_width

deep_inside_bbox(p):
    inside_or_on_bbox(p, bbox) && not in selected_side corridor
```

bridge candidate 过滤 / fallback：

```text
primary bridge candidates:
    candidates in selected_side corridor or selected_side outside bbox

fallback bridge candidates:
    candidates inside bbox but still close to selected_side corridor

deep-inside candidates:
    do not generate by default
    only enable if all primary/fallback candidates are illegal
    add large deep_inside_bbox_penalty
```

preferred-side monotonic constraint for access -> bridge：

```text
LEFT:   bridge.x <= child/access.x
RIGHT:  bridge.x >= child/access.x
BOTTOM: bridge.y <= child/access.y
TOP:    bridge.y >= child/access.y
```

违反该 monotonic constraint 的 bridge candidate 应直接过滤，或加入极大 penalty。

## 10.2 top placement

`top` 是 cluster 对外代表点，必须遵循 bbox outside 硬约束。top 可以使用 bbox center 的投影来帮助生成外侧候选点，因为 top 的目标是从 cluster bbox 的 preferred side 对外接入。

top candidate 坐标来源允许包括：

```text
bbox center x / bbox center y
access.loc.x / access.loc.y
bridge.loc.x / bridge.loc.y
child loc x/y
parent/global estimated loc x/y
selected_side bbox outside coordinate with margin in {1, 2, 4, 8}
```

top fallback 规则：

```text
top must first try candidates outside bbox on selected_side
if selected_side outside candidates are all illegal:
    fallback to outside bbox candidates on other sides
fallback top candidates still must be outside related bbox
```

```text
for each bridge/top node:
    related_bbox = subtree sinks bbox or owning cluster bbox
    preferred_side = inherited cluster_preferred_side[cluster_id] from Stage 0

    if node is bridge:
        generate_bridge_candidates_without_bbox_center(node, related_bbox, preferred_side)
        enforce selected_side corridor and access->bridge monotonic constraint
        do not use bbox center x/y as candidate anchors

    if node is top:
        generate_top_candidates_with_bbox_outside_constraint(node, related_bbox, preferred_side)
        bbox center projection is allowed for top only
```

并保留后续 primary_sides / fallback_sides 的逻辑，但要明确：

```text
bridge 的 primary candidates 来自 selected_side corridor / selected_side outside；
top 的 primary candidates 来自 selected_side outside bbox。
```

`margin` 建议从 1 开始，可根据 congestion 增大：

```text
margin in {1, 2, 4, 8}
```

过滤：

```text
[bridge] remove candidate outside grid if grid exists
[top]    remove candidate inside/on related bbox; remove candidate outside grid
remove duplicate candidates
```

评分（bridge）：

```text
score_bridge(p) =
      300.0  * skew_penalty_after_placing_node(p)
    + 200.0  * congestion_penalty(p)
    + 100.0  * lshape_forbidden_sink_penalty(parent/child, p)
    +  50.0  * estimated_crossing_penalty(p)
    +  20.0  * total_manhattan_to_neighbors(p)
    + 500.0  * deep_inside_bbox_penalty(p)
    + 300.0  * preferred_side_monotonic_violation_penalty(p)
    + 100.0  * side_switch_penalty(p)
```

评分（top）：

```text
score_top(p) =
      INF    * inside_or_on_related_bbox(p)
    + 300.0  * skew_penalty_after_placing_node(p)
    + 200.0  * congestion_penalty(p)
    + 100.0  * lshape_forbidden_sink_penalty(parent/child, p)
    +  50.0  * estimated_crossing_penalty(p)
    +  20.0  * total_manhattan_to_neighbors(p)
    +  50.0  * side_switch_penalty(p)
```

解释：

- bridge: 无 bbox 硬约束；`bbox_inside_penalty` 为 soft penalty，候选点在 bbox 内时会增大 score 但不会直接过滤。bridge 优先从 primary_sides（Stage 0 preferred_side）选点。
- top: `inside_or_on_related_bbox` 是硬约束。
- `side_switch_penalty` 避免 child access 从某一侧出来后 bridge/top 又绕到另一侧。
- `skew_penalty_after_placing_node` 根据 child subtree delay profiles 加上候选 edge delay，估计该 bridge/top 下所有 sinks 到该 node 的 skew。
- `congestion_penalty` 衡量候选点附近是否已有 bbox 边界、outer node、预计通道。
- `lshape_forbidden_sink_penalty` 只用于估计，不输出 route。
- `estimated_crossing_penalty` 估计 parent-child L-shape 是否会穿过其他 cluster bbox 或已有 outer connection corridor。
- `total_manhattan_to_neighbors` 保持线长合理。

选择 score 最小的 candidate。

---

# 11. global placement（order-constrained，不锁定 treer 坐标）

treer 已经生成了 source-aware global topology。locer 不再强依赖 treer 给出的具体 `Global` 坐标，而是只保留 global chain 在某条 primary axis 上的相对顺序。

```text
treer 决定 global topology 和 global order；
locer 负责在保持 order 的前提下重新选择 global loc。
```

也就是说：

```text
Global nodes are order-constrained, not anchor-locked.
```

## 11.1 提取 global chain order 与 primary axis

对所有 `Global` nodes，先从 topology 中提取 source-to-cluster 的 global chain 顺序。treer 给出的 `node.loc` 只允许用于推断顺序、primary axis、初始 debug 参考，不再作为强 anchor 约束。

primary axis 判断：

```text
global/top/source_points = source + all Global nodes + connected ClusterTop nodes
width  = max(x) - min(x)
height = max(y) - min(y)

if width >= height:
    primary_axis = X
else:
    primary_axis = Y
```

如果 treer/source-aware topology 中已有 sweep direction，也可以直接使用：

```text
source left/right sweep -> primary_axis = X
source up/down sweep    -> primary_axis = Y
```

## 11.2 Global order constraint

locer 必须保持 global nodes 在 primary axis 上的相对顺序。

对 horizontal chain：

```text
G0.x <= G1.x <= G2.x <= ... <= Gn.x
```

对 vertical chain：

```text
G0.y <= G1.y <= G2.y <= ... <= Gn.y
```

若 sweep 方向相反，则使用降序约束。建议加入最小间隔避免重叠：

```text
min_global_gap = 1.0
G(i+1).axis >= G(i).axis + min_global_gap   // ascending
G(i+1).axis <= G(i).axis - min_global_gap   // descending
```

该约束是 hard constraint。违反 global order 的 candidate 应直接过滤或给 `INF` penalty。

## 11.3 Global candidate 生成

global candidates 不再围绕 treer `node.loc` 小半径生成，而是在满足 order interval 的范围内生成。

对第 `i` 个 global node，先根据相邻 global / source / connected top 估计合法 interval：

```text
lower_axis_bound = previous_global_or_source_axis + min_global_gap
upper_axis_bound = next_global_or_connected_top_axis - min_global_gap
```

对 horizontal chain，candidate 的 `x` 必须位于 `[lower_axis_bound, upper_axis_bound]`；对 vertical chain，candidate 的 `y` 必须位于对应 interval。

candidate 坐标来源：

```text
source.x / source.y
source-aligned trunk coordinate
child/top loc x/y
parent/global loc x/y when already placed
connected ClusterTop bbox center projection
evenly-spaced reference positions along primary axis
bbox union boundary projection
```

其中 evenly-spaced global slots 推荐：

```text
ref[i] = source_axis + (i + 1) / (num_globals + 1) * (last_cluster_axis - source_axis)
```

对直接连接 source 的 global root，应该允许其靠近 source：

```text
G0.axis ≈ source.axis + direction * small_margin
```

这样可以避免 `SOURCE -> first Global` 过长，同时仍保持 global chain order。

## 11.4 Global monotonic / side-preserving constraint

locer 放置 global node 时，不能破坏 treer 阶段的 source-aware ordering。

实现时使用 primary axis 和 global order 判断，不再使用 treer anchor 作为硬约束：

```text
if primary_axis == X:
    candidate 必须满足当前 global 在 X 轴上的 order interval
    并尽量保持 y 靠近 source.y 或局部 trunk_y
else:
    candidate 必须满足当前 global 在 Y 轴上的 order interval
    并尽量保持 x 靠近 source.x 或局部 trunk_x
```

如果 source 本身位于 cluster union bbox 内部，连接 source 的 `Gsrc` 应保持靠近 source，但两侧 chain 仍然只需要保持各自在 primary axis 上的顺序。

## 11.5 不允许 Global 跳到无关位置

新规则：

```text
Global candidate 合法性 =
    不在任何 cluster bbox 内
  + 满足 primary-axis order interval
  + 不破坏 source-aware order
```

不能只用 `not inside any single cluster bbox` 作为 global 合法性条件。即使 candidate 不在 cluster bbox 内，只要它破坏 global order、跨到错误侧、或导致明显回绕连接，也应该直接判非法或给极大 penalty。

## 11.6 Source inside bbox 的特殊约束

如果 treer 中 source 位于 cluster tops union bbox 内部，并且 global topology 为左右链结构，locer 必须保持：

- `Gsrc` 靠近 `source.loc`
- left_chain 的 global nodes 保持在 source 左侧/下侧
- right_chain 的 global nodes 保持在 source 右侧/上侧

对 `Gsrc` 的候选点，应优先靠近 `source.loc`，并强惩罚：

```text
L1(candidate, source.loc)
```

不再强依赖 `treer Gsrc.loc`，只要求 `Gsrc` 满足左右/上下两侧 global chain 的 order constraint。

## 11.7 Global edge crossing 检查

对 global node 的候选位置，使用 parent-child 直线段做轻量 segment-intersection 检查：

```text
for each candidate p:
    temporarily set global.loc = p
    form straight segments to placed parent/children when available
    estimate intersections with existing outer/global/internal edges
    crossing 越多，penalty 越大
```

crossing optimization 不能压过 order constraint。

## 11.8 Global placement scoring

```text
score_global(p) =
      INF    * illegal(p)
    + INF    * global_order_violation(p)
    + 400.0  * skew_penalty_after_placing_node(p)
    + 300.0  * congestion_penalty(p)
    + 100.0  * estimated_crossing_penalty(p)
    + 1000.0  * global_top_attachment_penalty(p, attached_top_loc)
    +  80.0  * source_trunk_penalty(p)
    +  20.0  * total_manhattan_to_neighbors(p)
    +   5.0  * imbalance_penalty_to_children(p)
```

`global_order_violation_penalty(p)` 对违反 primary-axis order interval 的 candidate 返回 `INF` 或极大 penalty。

`source_trunk_penalty(p)` 用于让 horizontal chain 的 global node 尽量靠近 `source.y`，或让 vertical chain 的 global node 尽量靠近 `source.x`。

`attached_top_loc` 指的是该 global 的直接 `ClusterTop` child 的 loc。

`global_top_attachment_penalty(p, attached_top_loc)` 指的是该 global 到其绑定 top 的 manhattan 距离。不要退化成“该 global 的任意 child 的最小距离”；这里只能看绑定的 top。

## 11.9 与 cluster-internal placement 的关系

- 该修改只针对 `Global` nodes。
- `ClusterTop`、`ClusterBridge`、`ClusterAccess`、`ClusterInternal` 仍然按照原 locer 规则处理。
- global node 连接到 `ClusterTop` 时，不允许为了贴近某个 top 而破坏 primary-axis global order。

---

# 12. congestion_penalty

用轻量 heuristic，不做真实 routing。

```text
congestion_penalty(p) =
    bbox_proximity_penalty(p)
  + outer_node_density_penalty(p)
  + corridor_overlap_penalty(p)
  + narrow_channel_penalty(p)
```

建议实现：

- `bbox_proximity_penalty`：离任意 cluster bbox 边界太近则增加 penalty。
- `outer_node_density_penalty`：半径 R 内已有 bridge/top/global 越多，penalty 越高。
- `corridor_overlap_penalty`：候选点与 parent/child 的简单 L-shape 估计通道若和已有 outer 估计通道重叠，penalty 增大。
- `narrow_channel_penalty`：候选点落在两个 bbox 之间的窄缝区域时增加 penalty。

这些 penalty 只用于 loc 排序，不生成或锁定 route。

---

# 13. skew_penalty

skew-aware placement 的核心函数：

```text
skew_penalty_after_placing_node(candidate_loc, node):
    all_delays = []
    child_worst_delays = []

    for each child c of node:
        if child loc/profile is known:
            edge_delay = L1(candidate_loc, loc[c])

            // keep full profile for data/debug
            for d in sink_delays_to_node[c]:
                all_delays.push_back(d + edge_delay)

            // main skew-balance term uses worst sink delay of this child subtree
            child_worst_delays.push_back(max_sink_delay_to_node[c] + edge_delay)

        else if child has estimated loc/profile:
            use estimated loc/profile in the same way
        else:
            return large penalty

    if child_worst_delays empty:
        return 0

    worst_balance = max(child_worst_delays) - min(child_worst_delays)
    full_profile_skew = max(all_delays) - min(all_delays) if all_delays not empty else 0

    return worst_balance + 0.1 * full_profile_skew
```

对 binary outer node，主目标是比较左右 child subtree 的最大 sink delay：

```text
left_worst  = left.max_sink_delay_to_node  + edge_delay(candidate_loc, left.loc)
right_worst = right.max_sink_delay_to_node + edge_delay(candidate_loc, right.loc)

worst_balance = abs(left_worst - right_worst)
```

完整 `sink_delays_to_node` 仍然需要更新：

```text
sink_delays_to_node[node] = concat(child.sink_delays_to_node + edge_delay_to_child)
min/max/skew 再由完整 profile 统计得到
```

实现要求：

- 每次确定一个 node loc 后，立即更新该 node 的完整 `sink_delays_to_node / min / max / skew`。
- 每次给 loc candidate 打分时，必须先取每个 child 的 `max_sink_delay_to_node`，再加上候选 parent loc 到 child loc 的 edge delay，用这些 child worst delays 作为 skew balance 主项。
- bridge/top/global/source 都必须写入对应 delay profile。
- skew-aware 是 soft penalty，不允许为了降低 skew 违反 bbox hard constraint。
- locer 不在该阶段添加 detour；若 skew 不佳，只通过选择不同 loc 尽量改善。

---

# 14. source placement

source 处理规则：

```text
if problem has fixed source coordinate:
    use it directly
else:
    generate candidates around median/global center of top/global nodes
    choose candidate minimizing skew_penalty_after_placing_node + total_manhattan_to_global_neighbors + congestion_penalty
```

source 不需要满足 bbox outside 硬约束，但若多个候选近似等价，应优先选择不在 cluster bbox 内的点。

---

# 15. 几何 helper

在 `locer.cc` 匿名 namespace 中实现：

```cpp
struct BBox { double xmin, xmax, ymin, ymax; bool valid; };
static constexpr double EPS = 1e-9;
static constexpr double INF = 1e100;

double manhattan(SegmentPoint a, SegmentPoint b);
BBox compute_subtree_sink_bbox(...);
BBox union_bbox(...);
bool inside_or_on_bbox(SegmentPoint p, BBox b);
bool outside_bbox(SegmentPoint p, BBox b);
SegmentPoint bbox_center(BBox b);
std::vector<SegmentPoint> generate_bbox_side_candidates(...);
Side resolve_cluster_preferred_side(...);
SegmentPoint resolve_external_anchor_for_cluster_top(...);
SegmentPoint get_treer_node_guidance_loc(...);  // read node loc/cx/cy/estimated center from input topology tree
bool has_treer_node_guidance_loc(...);
Side get_inherited_preferred_side_for_node(...);
std::vector<SegmentPoint> dedup_candidates(...);
bool in_grid_if_grid_exists(SegmentPoint p, const Problem& problem);
double total_manhattan_to_neighbors(...);
double lshape_forbidden_sink_penalty(...);
double estimated_crossing_penalty(...);
double congestion_penalty(...);
double deep_inside_bbox_penalty(...);
bool in_selected_side_corridor(...);
double preferred_side_monotonic_violation_penalty(...);
double side_bonus(...);
enum class PrimaryAxis { X, Y };

PrimaryAxis infer_global_primary_axis(...);
std::vector<int> extract_global_chain_order(...); // 此函数可利用 treer 传入的 global的坐标进行排序。根据 globan bbox aspect ratio 推断 primary_axis，然后按照坐标轴排序。

double global_order_violation_penalty(...);
double source_trunk_penalty(...);
int find_attached_top_for_global(...);
double global_top_attachment_penalty(const SegmentPoint& p,
                                     const SegmentPoint& attached_top_loc);

std::vector<SegmentPoint> generate_global_order_constrained_candidates(...);
void update_delay_profile_for_node(...);
double skew_penalty_after_placing_node(...);
double delay_profile_skew(const std::vector<double>& delays);
double child_worst_delay_after_edge(...);
double worst_child_delay_balance_penalty(...);
```

最终语义必须是：

```text
sink_delays_to_node 保留所有 sinks delay；
max_sink_delay_to_node 是每个 subtree 的 worst sink delay；
loc candidate 打分时，先看每个 child subtree 的 max delay 加上 candidate 到 child 的 edge delay；
balance 的主目标是让这些 child worst delays 尽量相等；
完整 delay profile 仍然要维护，用于 debug 和辅助 penalty。
```

---

# 16. Debug 输出

`debug_output()` 打印：

```text
[LOCER] valid/error_msg/num_nodes
for each physical cluster / ClusterTop:
  cluster_id cluster_top_node_id bbox external_anchor anchor_source dx dy preferred_side tie_used tie_candidates
  access_nodes=[...] access_locs=[...]
  DME access-subtree count

for each node:
  node_id class cluster_id parent left right
  loc=(x,y) loc_mode candidate_count loc_score
  inside_related_bbox congestion_penalty lshape_penalty wire_est_to_parent
  sink_delay_count min_sink_delay max_sink_delay skew_to_node skew_penalty
```

outer node 额外打印：

```text
related_bbox
inherited_preferred_side
actual_side
fallback_side_used
nearest bbox distance
estimated crossing penalty
neighbor ids used for scoring
```

---

# 17. Debug loc 文件输出

`locer` 需要像 `treer` 一样支持 debug 文件输出。该输出只用于检查 loc assignment，不是最终 solution，不由 evaluator 读取。

开关：

```cpp
locer::debug_file_enable(true);
```

关闭时不写任何 `loc/*.txt` 文件。

文件路径：

```text
loc/sample<k>_loc.txt
```

`<k>` 从输入文件名推导：

```text
samples/sample1.txt  -> loc/sample1_loc.txt
sample2.txt          -> loc/sample2_loc.txt
```

若 `input_path` 为空或无法解析 sample 名，则使用：

```text
loc/loc_debug.txt
```

写文件前必须确保 `loc/` 目录存在；不存在则创建。

推荐格式：

```text
# LOCER_DEBUG_LOC v1
# valid=<0/1>
# num_nodes=<N>
# columns: node_id class cluster_id x y loc_mode parent left right candidate_count loc_score inside_related_bbox congestion_penalty lshape_penalty wire_est_to_parent sink_delay_count min_sink_delay max_sink_delay skew_to_node skew_penalty
node <node_id> <class> <cluster_id> <x> <y> <loc_mode> <parent> <left> <right> <candidate_count> <loc_score> <inside_related_bbox> <congestion_penalty> <lshape_penalty> <wire_est_to_parent> <sink_delay_count> <min_sink_delay> <max_sink_delay> <skew_to_node> <skew_penalty>
```

示例：

```text
node 12 access 0 34 18 DME_ACCESS_ROOT 20 7 8 1 0 0 0 0 0 4 0 12 12 0
node 20 bridge 0 34 10 BRIDGE_CONGESTION_AWARE 28 12 -1 24 132.5 0 0.2 0 12 4 12 24 12 12
```

要求：

- 按 `node_id` 从小到大输出。
- 每个 topology node 输出一行。
- `x/y` 输出最终确定的 `loc`。
- `class` 使用 `sink/internal/access/bridge/top/global/source`。
- parent/left/right 使用原始 topology node id；没有则输出 `-1`。
- 文件输出失败时返回 false，并设置 `error_msg`。
- debug 文件输出不得替代 stdout debug。

---

# 18. 合法性检查

至少检查：

- `problem.valid == true`
- `tree.valid == true`
- `tree.root` 合法
- 每个 node class 可识别
- 每个 sink/internal/access node 被 cluster-local DME 赋 loc
- 同一个 physical cluster / ClusterTop 下的所有 access 必须共享同一个 preferred_side
- cluster bbox 必须由该 physical cluster / ClusterTop 下所有 sinks 计算，不能由单个 access subtree 计算
- access loc 只需在 access.ms 上，不检查 outside cluster bbox
- 每个 bridge/top/global/source node 被 outer placement 赋 loc
- 每个 loc 是 finite number
- 每个 node 的 `sink_delays_to_node` 非空，除非该 node 不是任何 sink 的 ancestor
- `min_sink_delay_to_node <= max_sink_delay_to_node`
- `skew_to_node >= -EPS`
- locer 必须先执行 Stage 0，生成 cluster_preferred_side[cluster_id]
- Stage 0 的 external_anchor 必须优先来自 treer 输入 topology node 中已有的 loc/cx/cy/estimated center/debug center
- Stage 0 不允许依赖 locer 后续生成的 parent/global/top placement result 作为主要 anchor
- 同一个 physical cluster / ClusterTop 下的 access、bridge、top 的 inherited_preferred_side 必须一致
- access/bridge/top 不允许单独重新估计 preferred_side
- 非 tie 情况下，congestion / wirelength scoring 不允许推翻 Stage 0 的 anchor 主导方向
- top 若属于单 cluster，必须优先位于该 cluster preferred_side 对应的 bbox 外侧；只有该 side 全部候选非法时才允许 fallback
- bridge 不强制 outside bbox，但必须优先位于 inherited_preferred_side corridor；deep-inside bbox 只能作为高 penalty fallback
- bridge/top 的 preferred_side 必须与所属 physical cluster / ClusterTop 一致；bridge/top 不允许为每个 access 单独重新估计 preferred_side
- top 必须 outside related cluster bbox
- bridge 不要求 outside bbox
- global 不能 inside 任意单个 cluster bbox
- global placement 不再强依赖 treer-provided node.loc；treer loc 只可用于推断 global order / primary axis / debug reference
- global candidate 必须满足 primary-axis order interval，不允许破坏 global chain 在 primary axis 上的相对顺序
- global loc 不能破坏 treer 的 source-aware ordering（order-constrained / side-preserving constraint）
- source 若 problem 给定坐标，必须等于 source 坐标
- 每个 node result valid
- debug file enabled 时，`loc/sample<k>_loc.txt` 写入成功

失败时：

```cpp
result.valid = false;
result.error_msg = "清晰错误信息";
```

---

# 19. main.cc / 构建接入

`main.cc`：

```cpp
#include "locer.h"

locer::debug_enable(true);
locer::debug_file_enable(true);
common::LocerResult loc_result = locer::run(problem, tree, argv[1]);
if (!loc_result.valid) {
    std::cerr << "LOCER error: " << loc_result.error_msg << "\n";
    return 1;
}
```

构建文件：

```text
SRC += src/locer.cc
INC += include/locer.h
```
