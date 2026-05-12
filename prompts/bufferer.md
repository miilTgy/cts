# Bufferer Prompt

请实现 `bufferer` 阶段：在 `router/detourer` 已经生成的合法 Manhattan route 基础上，尝试在 topology node 上插入 buffer，以降低 skew、提高最终 score。

`bufferer` 的主要目标是 **skew optimization**，不是 fanout-driven insertion。fanout 只作为合法性检查：任何方案不能违反 fanout limit，但不能因为 fanout 超过某个阈值就无条件插 buffer。

bufferer 不负责重新生成 topology，不负责重新 route，不负责 locer/router/detourer 的职责。它只在已有 topology node 上选择是否放置 buffer，并更新 delay / fanout / cost / skew 统计。

---

## 0. 核心原则

### 0.1 Skew-first，不改善 skew 就不插

bufferer 必须遵守：

```text
如果插入 buffer 不能改善最终 skew / score，则不插入。
```

也就是说，每个候选 buffer 都必须先临时应用，然后重新计算 delay profile 和目标 skew。只有满足以下条件才允许 commit：

```text
after_skew < before_skew
```

更推荐使用 score-aware 判断：

```text
delta_score = 5000 * (before_skew - after_skew)
            - 200  * added_buffer_cost
            - 50   * added_wirelength
```

由于 buffer 不改变 wirelength，通常：

```text
delta_score = 5000 * skew_improvement - 200 * added_buffer_cost
```

只有：

```text
delta_score > 0
```

才 commit buffer。

如果作业评分权重不同，请将权重配置化；但核心规则不变：**skew/score 不改善，不插 buffer。**

### 0.2 Fanout 只做 legality check

fanout limit 不是插 buffer 的主触发条件。fanout 只用于判断某个候选方案是否合法：

```text
if any driver fanout > selected_buffer.fanout_limit:
    candidate illegal
```

不要写成：

```text
if downstream_sink_count > fanout_limit:
    必须插 buffer
```

除非作业明确规定 fanout violation 会导致非法解，否则不要为了 fanout 强行插 buffer。当前 bufferer 的主要目标是降低 skew。

### 0.3 Buffer 只插在 node 上，不插在 edge 中间

buffer 只能插在 topology node 上，不能插在 routed edge 的中间 grid point。

允许插 buffer 的 node：

```text
internal
access
bridge
top
global
```

禁止插 buffer 的位置：

```text
sink
source
edge midpoint
edge segment 上的普通 grid point
route turning point，如果它不是 topology node
```

因此 bufferer 不需要枚举 edge polyline 中点，也不需要修改 edge polyline。buffer placement 只是在已有 node loc 上标记该 node 使用某个 buffer type。

---

## 1. 输入与输出

输入：

```cpp
common::Problem problem;
common::TopologyTree tree;
common::LocerResult loc_result;
common::RouterResult route_result;
common::DetourerResult detour_result;  // 如果没有 detourer，可使用 router_result
```

输出：

```cpp
common::BuffererResult result;
```

核心输出：

```text
每个 node 是否插 buffer
每个 node 的 buffer type
每个 node 的 downstream sink count
每个 node 的 effective load / fanout legality
每个 node 的 delay profile
总 buffer cost
插 buffer 前后的 skew / score
```

要求：

- 不改变 topology。
- 不改变 node loc。
- 不重新 route。
- 不修改 edge polyline。
- buffer 只能标记在非 sink/source 的 topology node 上。
- buffer 不能插在 edge 中间。
- buffer 插入后必须重新计算 delay profile。
- buffer 插入后必须满足 fanout legality。
- 若 skew/score 不改善，必须 rollback，不 commit。

---

## 2. Buffer library / fanout model

需要支持一组 buffer 类型。若 problem 已提供 buffer library，则优先使用 problem 中的数据；否则可先使用默认抽象 library。

示例：

```cpp
struct BufferType {
    std::string name;
    int fanout_limit = 0;
    double delay = 0.0;
    double cost = 0.0;
    double input_cap = 1.0;
    double drive_strength = 1.0;
};
```

默认可设：

```text
BUF_X1: fanout_limit=4,  delay=1, cost=1
BUF_X2: fanout_limit=8,  delay=2, cost=2
BUF_X4: fanout_limit=16, delay=3, cost=4
```

具体数值应按作业要求/评测器定义调整。

fanout legality：

```text
node 如果插入 buffer，则该 buffer 驱动 node 的 children / downstream branches。
该 buffer type 的 fanout_limit 必须 >= 它实际驱动的 fanout。
```

如果 node 不插 buffer，则该 node 由上游 driver 直接看到，fanout 由上游候选方案检查。

---

## 3. Delay model with node buffers

buffer 插在 node 上时，buffer delay 计入从该 node 到其 parent / source 方向的 path delay，等价于所有经过该 node 的 sink-to-root path 增加该 buffer 的 delay。

推荐定义：

```text
path_delay(sink -> node) = wire delays along child edges + child subtree buffer delays
```

如果当前 node 插了 buffer：

```text
sink_delays_to_parent_through_node = sink_delays_to_node + buffer_delay[node] + edge_delay(parent-node)
```

也就是说，对当前 node 的所有下游 sinks，经过该 node 向上传播时都会额外增加 `buffer_delay[node]`。

叶子 sink：

```text
sink_delays_to_node[sink] = [0]
```

内部 node：

```text
sink_delays_to_node[node] = concat(
    for each child c:
        for d in sink_delays_to_node[c]:
            d + buffer_delay_if_inserted_at_child + route_delay(edge node-c)
)
```

然后：

```text
min_sink_delay_to_node = min(sink_delays_to_node[node])
max_sink_delay_to_node = max(sink_delays_to_node[node])
skew_to_node = max - min
```

注意：

- 保留完整 `sink_delays_to_node`，方便 debug。
- 优化时可以优先看每个 child subtree 的 max delay。
- buffer 不能插在 sink 上，所以 sink 的 buffer_delay 永远为 0。
- source 不作为可插 buffer node。

---

## 4. Downstream sink count / fanout legality

每个 node 必须 bottom-up 维护：

```text
downstream_sink_count[node]
```

递推：

```text
if node is sink:
    downstream_sink_count[node] = 1
else:
    downstream_sink_count[node] = sum(downstream_sink_count[child] for child in children)
```

不要直接依赖 `TopoNode::sink_indices.size()`，除非确认该字段已经是完整 subtree sinks。更稳妥是自己 bottom-up 汇总。

fanout legality 检查：

```text
fanout(node) = number of children / effective driven branches / downstream sinks according to problem definition
```

如果 buffer 插在 node 上：

```text
buffer_type.fanout_limit >= fanout_driven_by_this_node_buffer
```

如果问题定义 fanout 是 sink count，则用 `downstream_sink_count`；如果定义 fanout 是 child branch count，则用 children count。必须和作业/evaluator 一致。

---

## 5. Candidate node generation

buffer candidate 只来自 topology nodes。

候选 node：

```text
node.kind in {internal, access, bridge, top, global}
node is not sink
node is not source
node has at least one downstream sink
```

禁止候选：

```text
sink node
source virtual node
已经插过 buffer 且不允许替换的 node
没有下游 sink 的孤立 node
```

候选顺序建议：

```text
bottom-up order
then nodes with larger skew contribution first
then nodes on shorter-delay side of an imbalanced parent first
then deterministic node_id order
```

---

## 6. Skew-driven insertion decision

主流程不是“fanout 超限就插”，而是：枚举一个 node + buffer type，临时插入，评估 skew/score 改善。

对每个候选 node `n` 和 buffer type `buf`：

```text
if fanout_legality(n, buf) fails:
    reject candidate

before = current global/root/source skew and score
apply temporary buffer at node n
recompute delay profiles bottom-up
check fanout legality globally
after = new global/root/source skew and score

if after.score > before.score:
    candidate is beneficial
else:
    reject and rollback
```

选择：

```text
commit best beneficial candidate
```

可以多轮迭代：

```text
while exists beneficial buffer candidate:
    commit best candidate
    recompute profiles
```

停止条件：

```text
no candidate improves score
or max_buffer_count reached
or max_pass reached
```

---

## 7. 哪些 skew 作为优化目标

优先目标应是最终 source/root skew：

```text
objective_skew = skew at source virtual root if source has multiple children
               else skew at tree.root
```

如果 source 是虚拟点，且 `tree.source_children` 为空但 `tree.root` 有效，则 objective 使用 `tree.root`。

也可以同时考虑局部 skew，但不能牺牲 global/root skew：

```text
primary: reduce objective_skew
secondary: reduce sum of node skews
tertiary: reduce max local node skew
```

commit 条件必须至少满足：

```text
objective_skew decreases OR score increases
```

推荐使用 score 增量作为最终 commit criterion。

---

## 8. Score function

候选 buffer 的 score：

```text
score = 5000000
      - 5000 * objective_skew
      - 50   * wirelength
      - 200  * total_buffer_cost
```

wirelength 不因 node buffer 改变。若作业评分权重不同，应将权重配置化。

candidate commit rule：

```text
if new_score > old_score:
    commit
else:
    rollback
```

如果只看 skew，不看 score，则容易出现 skew 小幅改善但 buffer cost 过高导致总分下降。因此推荐 score-aware。

---

## 9. Buffer type selection

对每个合法 candidate node，枚举所有 buffer type：

```text
for node in candidate_nodes:
  for buf in buffer_library:
      if fanout_legality(node, buf):
          temporary_insert(node, buf)
          recompute profiles
          evaluate score
```

选择使 score 最大的候选。

tie-break：

```text
higher score first
then lower objective_skew
then lower buffer cost
then lower buffer delay
then smaller buffer type
then lower node_id
```

---

## 10. 多轮插入与 rollback

bufferer 应支持多轮 greedy improvement：

```text
initialize current_solution with no new buffers or existing buffers
compute current profiles / score

for pass in 0..max_pass:
    best_candidate = none

    for node in candidate_nodes:
        for buf in buffer_library:
            if node already has buffer:
                optionally try replacement instead of duplicate insertion
            if fanout illegal:
                continue
            apply temporary candidate
            recompute profiles
            if score improves best_candidate:
                save candidate
            rollback temporary candidate

    if best_candidate is none:
        break

    commit best_candidate
    recompute profiles / score
```

不要在同一个 node 上叠多个 buffer。若该 node 已经有 buffer，只允许：

```text
replace with another buffer type if score improves
or keep existing buffer
```

---

## 11. Fanout legality as global check

每次 candidate 临时插入后，必须做全局 fanout legality check。

检查内容：

```text
for each driver node or buffer:
    computed_fanout <= driver_fanout_limit
```

如果 problem 没有明确 fanout legality 要求，fanout check 可以只作为 debug warning，而不是强制插入理由。

关键点：

```text
fanout violation 可以 reject candidate；
但 fanout pressure 不能强迫 commit 一个 skew/score 变差的 candidate。
```

---

## 12. Buffer legality check

由于 buffer 只插在 node 上，不插在 edge 上，几何合法性大幅简化。

每个 buffer candidate 必须检查：

1. node 不是 sink。
2. node 不是 source。
3. node loc 在 grid/die boundary 内。
4. node loc 是已有 topology node loc。
5. node 没有被其它 buffer 重复占用，除非是合法 replacement。
6. node 所在位置本身已有 node，因此不需要额外检查压 edge；但不能把 buffer 放到非 node 的 route point 上。
7. 插 buffer 不改变任何 route polyline，不改变 endpoint，不改变 port。
8. fanout legality 通过。
9. score/skew 改善。

---

## 13. Delay profile recomputation

每次临时插入/rollback/commit buffer 后，应重新计算 delay profile。

为了简单可靠，可以每次从头 bottom-up recompute：

```text
for node in bottom_up_order:
    if sink:
        delays[node] = [0]
    else:
        delays[node] = []
        for child in children(node):
            child_buf_delay = buffer_delay[child] if child has buffer else 0
            edge_delay = current routed/detoured edge delay(node, child)
            for d in delays[child]:
                delays[node].push_back(d + child_buf_delay + edge_delay)
```

若要计算 source virtual root：

```text
source_delays = []
for child in source_children or tree.root:
    child_buf_delay = buffer_delay[child] if child has buffer else 0
    edge_delay = route_delay(source, child)
    source_delays += delays[child] + child_buf_delay + edge_delay
source_skew = max(source_delays) - min(source_delays)
```

注意：buffer 插在 node 上，影响的是经过该 node 往 parent/source 方向传播的所有下游 sink paths。因此在 parent 汇总 child delays 时加入 `buffer_delay[child]`。

---

## 14. Processing order

主处理顺序：bottom-up recomputation + greedy global candidate search。

```text
build_bottom_up_order(tree)
build_candidate_nodes(bottom_up_order)
compute_initial_profiles_and_score()

while true:
    best_candidate = none
    for node in candidate_nodes:
        for buffer_type in buffer_library:
            if illegal candidate:
                continue
            temporary apply
            recompute profiles and score
            rollback
            if score improves:
                update best_candidate
    if no best_candidate:
        break
    commit best_candidate
    recompute profiles and score
```

candidate_nodes 可以按 bottom-up 顺序遍历，但最终 commit 以 score improvement 为准。

---

## 15. Debug output

需要支持 debug 开关：

```cpp
namespace bufferer {
void debug_enable(bool enable);
void debug_file_enable(bool enable);
void debug_output(const BuffererResult& result,
                  const common::Problem& problem,
                  const common::TopologyTree& tree,
                  const common::LocerResult& loc_result,
                  const common::RouterResult& route_result,
                  const common::DetourerResult& detour_result);
bool write_debug_buffer_file(const BuffererResult& result,
                             const std::string& input_path,
                             std::string& error_msg);
}
```

stdout debug 输出：

```text
[BUFFERER] valid/error_msg/num_buffers/total_buffer_cost
before_skew after_skew before_score after_score
for each committed buffer:
  buffer_id type node_id loc
  buffer_cost buffer_delay
  skew_before skew_after score_delta

for each rejected candidate summary:
  node_id type reject_reason
  skew_before skew_after score_delta
```

reject_reason 建议包括：

```text
SINK_NODE
SOURCE_NODE
FANOUT_ILLEGAL
NO_SKEW_IMPROVEMENT
NO_SCORE_IMPROVEMENT
ALREADY_BUFFERED
```

---

## 16. Debug buffer file

文件路径：

```text
buffer/sample<k>_buffer.txt
```

推荐格式：

```text
# BUFFERER_DEBUG v1
# valid=<0/1>
# before_skew=<S0>
# after_skew=<S1>
# before_score=<P0>
# after_score=<P1>
# num_buffers=<B>
# total_buffer_cost=<C>

buffer <buffer_id> <type> <node_id> <x> <y> <cost> <delay> <skew_before> <skew_after> <score_delta>
node <node_id> <downstream_sink_count> <min_delay> <max_delay> <skew> <has_buffer> <buffer_type>
```

写文件前确保 `buffer/` 目录存在。

---

## 17. Final legality check

buffer 插入完成后必须检查：

- 所有 buffer 都位于合法 topology node。
- 没有 buffer 位于 sink/source。
- 没有 buffer 位于 edge midpoint 或普通 route point。
- 每个 node 最多一个 buffer。
- 所有 driver fanout <= 对应 fanout limit，如果 fanout 是硬约束。
- 所有 route endpoints 未改变。
- 所有 route polyline 未被修改。
- delay profile 与最终 buffer/wire delay 一致。
- final score 不低于 initial score；否则 rollback 到无 buffer 或最佳历史方案。

若失败：

```cpp
result.valid = false;
result.error_msg = "clear reason";
```

---

## 18. main.cc / 构建接入

```cpp
#include "bufferer.h"

bufferer::debug_enable(true);
bufferer::debug_file_enable(true);
common::BuffererResult buffer_result = bufferer::run(problem, tree, loc_result, route_result, detour_result, argv[1]);
if (!buffer_result.valid) {
    std::cerr << "BUFFERER error: " << buffer_result.error_msg << "\n";
    return 1;
}
```

构建文件：

```text
SRC += src/bufferer.cc
INC += include/bufferer.h
```