# HW3 Report: Wire Delay CTS — Algorithmic Strategies

## 流程总览

```
Parser → Partitioner → Partreer (cluster topology) → Treer (global tree)
→ Locer (DME BU/TD + congestion-aware placement) → Router (Manhattan routing)
→ Detourer (skew balancing) → Bufferer (buffer insertion for skew) → Writer (solution output)
```

优先级：合法可布线性 > skew 潜力 > buffer cost > wirelength

评分公式：

```text
score = 5000000 - 5000 × skew - 50 × wirelength - 200 × buffercost
```

---

## 1. Parser

### 策略：关键词驱动的结构化解析

- 按 `DIE` → `SOURCE` → `NUM_SINKS` → `SINK ...` → `NUM_BUFFERS` → `BUFFER_TYPE ...` 固定顺序读取
- 所有 node id 去重校验（source, sinks, buffer types）
- 所有坐标进行 die boundary 合法性检查（含边界点）
- 采用 **fail-early** 策略：遇到任何格式/语义错误立即设置 `valid=false` 并记录详细错误信息

---

## 2. Partitioner — Adaptive Multi-Gap Partition

### 问题定义

将平面上的 sink 集合递归划分为层次化空间的 cluster tree。每个 leaf cluster 后续由 partreer 构建局部拓扑。

### 核心算法策略

**2.1 自适应 big gap 检测**

对当前 sink set 分别沿 x 轴和 y 轴评估 split 质量。对某条轴：
1. 按坐标排序，计算所有相邻 sink 间的 Manhattan gap。
2. 收集所有正 gap，取 **中位数** 作为 `small_gap`。
3. big gap 判定条件：
   ```
   gap >= min_abs_gap (10)  AND  gap >= gap_ratio (4.0) × small_gap
   ```
   若没有任何 positive gap，则该轴不可 split。

**2.2 Multi-way split 与 outlier 处理**

- 用该轴的所有 big gaps 一次性切成多个连续 group（非二分的 multi-way split）。
- 每个 group 至少含 2 个 sink；若 group 仅 1 个 sink：
  - 若相邻 gap 满足 `outlier_gap_ratio = 3.5` 和 `outlier_min_abs_gap = 20`，保留为 single-sink outlier。
  - 否则，将该单 sink 合并入相邻较小 group（优先左侧，再右侧）。
- 若最终 group 数 ≤1，该轴候选无效。

**2.3 轴的选取**

对 x/y 两轴候选人打分：

```text
score = total_big_gap
      + (is_major_axis ? 0.5 : 0.0)   // 优先 bbox 长轴
      - num_tiny_groups × 0.3          // 惩罚 size=2 的 group
```

注：代码中实际使用的 `gap_ratio = 4.0`, `outlier_gap_ratio = 3.5`, `min_cluster_size = 4`，注释常量为建议初始值。

**2.4 递归终止**

- `n ≤ min_cluster_size`（4）→ leaf cluster
- 两轴均无合法 candidate → leaf cluster（代码中称为 "no axis"）

---

## 3. Partreer — Pair-first Access Tree

### 问题定义

在单个 cluster 内部，将 sinks 逐步配对为 binary topology，形成以 `ClusterAccess` 为接入点、`ClusterTop` 为对外的唯一根的局部树。

### 核心算法策略

**3.1 Multi-level candidate pairing**

每层执行：
1. **候选生成**：基于 x-sorted 和 y-sorted k-nearest（k=3）、Manhattan nearest 生成候选边对。
2. **pair cost**：
   ```text
   cost = manhattan(a.loc, b.loc)
        + 0.10 × bbox_penalty          // 合并后 bbox 周长
        + 0.50 × interleave_penalty    // 跨过其它 active node 的数量
        + 0.05 × skew_penalty          // subtree skew 差异
   ```
3. **Matching**：
   - 当 active nodes ≤ 22 时，用 **bitmask DP** 精确求解最小 cost 匹配。
   - 当 active nodes > 22 时，用 **贪心** 按 cost 顺序选择未使用的 pair。
   - 允许 **carry**（不匹配传递到下一层），避免强制恶 pair。
4. **Tapping node 位置**：取左右 child 的 midpoint，然后按 external_target 做 source-aware bias（±1 偏移）。

**3.2 CONNECTABLE 判定（线段相交法）**

新 pair 的 candidate segment `(a, b)` 必须不与已有 topology segment 非法相交：
- 使用 **cross product** 判断线段相交（`cross(a, b, c) * cross(a, b, d) < 0`）
- T-junction（端点落在另一条线段内部）也算非法
- 共线重叠判定：若同线且投影区间有交集，非法
- 唯一例外：两条线段只共享同一个 topology node 的端点

连接 parent 后，检查 `parent→left` 和 `parent→right` 的实际线段是否与已有 segment 合法。

**3.3 Tapping node 位置搜索**

不只用 midpoint，而是枚举多个 candidate 坐标（biased midpoint、midpoint、左右端点、external_target、bbox 四角、die 四角等），选择第一个满足所有 connectivity 约束的位置。

**3.4 Access tree 构造与 canonicalize**

- pairing 停止条件：active node 数 ≤ `target_access_points`（3）或达到 `max_pair_levels`（8）仍不收敛
- access tree 构造：确保 `ClusterAccess` → `ClusterBridge` → `ClusterTop` 的层次关系
- **Canonicalize**：反复吸收可比较的一元 parent-child pair（偏序：`ClusterTop > ClusterBridge > ClusterAccess > ClusterInternal`），消除冗余一元 wrapper
- `ClusterSink` 可直接替换 `ClusterAccess/ClusterInternal`

---

## 4. Treer — Source-aware Global Tree Construction

### 问题定义

将 partition tree 中的 leaf cluster 的 cluster topologies 组装为全局 binary tree，最后执行 source-aware 几何修正。

### 核心算法策略

**4.1 递归遍历 partition hierarchy**

- 对 leaf partition，调用 `partreer::build()` 生成 cluster 内部 topology。
- 对 internal partition，递归处理 children，然后将 child roots 汇集。
- **关键**：递归 merge 的结果只是中间产物；最终 global tree 会由 source-aware 几何重建覆盖。

**4.2 Source-aware 几何 Global Topology 修正**

所有 `ClusterTop` roots 收集完毕后，按 source 位置重建 global tree：

1. **判断 source 相对 cluster union bbox 的位置**：
   - source 在左侧 → 按 x 升序 sweep
   - source 在右侧 → 按 x 降序 sweep
   - source 在下方 → 按 y 升序 sweep
   - source 在上方 → 按 y 降序 sweep
   - source 在内部 → 取 spread 更大的轴作为 primary sweep axis

2. **Source 作为分界点**：当 source 在 union bbox 内部时，将 clusters 分为 source 负方向侧和正方向侧，分别构造 ordered chain，最后用一个靠近 source 的 Global 根连接两侧。

3. **Ordered chain-like binary tree**（非 balanced recursive merge）：
   ```text
   root = Cn-1
   for i = n-2 downto 0:
       root = new Global(left = Ci, right = root)
   ```
   确保空间访问顺序符合 sweep 方向，避免跨 source 回绕。

**4.3 Canonicalize 与 Source 吸收**

- 若 source 只有一个 Global child 且重合，由 source 吸收该 Global（source 不经中间节点直接连接 children）。
- 反复吸收可比较的一元 parent-child pair，消除冗余。

---

## 5. Locer — DME BU/TD + Congestion-Aware Placement

### 问题定义

为 topology 中每个 node 确定几何坐标。内部节点（sink/internal/access）用 DME 确定；外部节点（bridge/top/global）用 congestion-aware heuristic 确定。

### 5.1 Stage 0：Cluster 级 preferred_side 预判

- 对每个 `ClusterTop`，先从 treer topology 中寻找外部 anchor（优先 parent treer loc 或 ancestor global loc → source → die center 降级）。
- 计算 `dx = anchor.x - bbox_center.x`, `dy = anchor.y - bbox_center.y`。
- 主导方向判定：`abs(dx) >= abs(dy)` → prefer X side，否则 prefer Y side。
- **Tie 处理**：当 `abs(dx) - abs(dy)` ≤ tie_eps 时，利用 congestion 辅助选择（计算各 side 到其它 cluster bbox 的距离/穿越风险），若仍平局则按 `TOP, RIGHT, BOTTOM, LEFT` 顺序。
- 整个 cluster 内的所有 access/bridge/top 继承同一 preferred_side。

### 5.2 Cluster 内部 DME（BU/TD）

**(a) Bottom-Up (BU) — Buffered DME**

使用 **(u, v) rotated coordinate**：
```text
u = x + y,   v = x - y
Manhattan dist = max(|u1-u2|, |v1-v2|)
```
TRR（Tilted Rectangular Region）在 (u,v) 中是 axis-aligned rectangle。

核心流程：
1. Leaf sink：ms = point segment at sink loc。
2. Internal node：
   - 构造 base TRR from child merging segment。
   - 分配 edge length：
     ```text
     l_mid = avg(left delay), r_mid = avg(right delay)
     D = min distance between left.ms and right.ms
     raw_left = (D + r_mid - l_mid) / 2
     rL = clamp(raw_left, 0, D),  rR = D - rL
     ```
   - 若初始 radius 无合法 intersection，**先取 rL = rR** 初始，再用 **exponential repair**：`extra = 0, 1, 2, 4, 8, ...` 逐步增加双方 radius，直到 expand_trr 相交。
   - **Buffer 枚举**：对每个 child 枚举所有满足 `max_fanout >= child.sink_count` 的 buffer type + no_buffer。sink 节点不允许加 buffer。
   - 选择 `total_cost = 5000×skew + 50×wire_est + 200×buffer_cost` 最小的 (buffer_L, buffer_R) 组合。
3. MS 提取优先级：
   ```
   (1) BOUNDARY_INTERSECTION: 取 expand_left.boundary ∩ expand_right.boundary
   (2) INTER_BOUNDARY_FALLBACK: 取 intersection 自身边界
   (3) INTERIOR_FALLBACK: 取 intersection 内最长 axis-aligned segment
   ```
   边界候选按 segment 长度优先；并列用 midpoint 到 base 的距离对称性 tie-break。

**(b) Top-Down (TD) — Location Assignment**

从 access root 向下确定每个 node loc：
1. 对每个 child：`feasible_ms = child.ms ∩ TRR(parent_loc, assigned_edge)`
2. 若 feasible 非空：生成 candidate points（包括 feasible_ms 中点、端点、parent loc 投影、各 sink 坐标±1 的投影等），按 loc_score 选最优（最多 32 candidates）。
3. `loc_score = 10000×max(0, dist-assigned) + 100×lshape_penalty + |dist-assigned| + 0.001×dist`
4. 若 feasible 空：fallback 为 child.ms 上 nearest point to parent_loc。

### 5.3 Congestion-Aware Outer Placement

**(a) Bridge placement**

- 继承 cluster preferred_side；优先在 selected_side corridor（宽 15% of max bbox dimension）内选点
- 不使用 bbox center 作为 candidate anchor（避免桥被吸入 cluster 中心）
- 加入 preferred-side monotonic constraint（如 LEFT: bridge.x ≤ child.x）
- Scoring：
  ```
  score = 300×skew + 200×congestion + 100×crossing + 20×wire
        + 50×bbox_inside + 250×deep_inside + 300×mono_violation
        + 10×side_switch
  ```
- 若 preferred_side 所有 candidate 非法，fallback 到其它 side。

**(b) Top placement**

- 必须严格 outside 相关 cluster bbox（硬约束）
- 同样继承 cluster preferred_side；fallback 到其它 side outside candidates
- 允许使用 bbox center 的 preferred_side 投影作为 candidate

**(c) Global placement — Order-Constrained**

treer 给出的 global 坐标仅用于推断顺序，locer 重新选点：
- 提取 primary axis（X 或 Y）和 global chain 顺序
- 每个 global node 有 order interval：`[prev_axis + min_gap, next_axis - min_gap]`
- source-side 约束：global 不能跨到 source 的另一侧
- Candidate 来源：source 轴、child/top loc、bbox 投影、均匀 spacing 参考点
- Scoring：
  ```
  score = 400×skew + 250×congestion + 120×crossing + 20×wire
        + 1000×top_attachment_penalty + 50×imbalance + 220×trunk_penalty
        + INF×order_violation
  ```
- 全局直接连接 `ClusterTop` 的 global，加上强 `L1(global, top_loc)` 惩罚

### 5.4 Delay Profile 维护

每个 node 维护 `sink_delays_to_node`（所有sinks到该node的delay list）。
Scoring 时以 child worst delay balance 为主项：
```text
child_worst = max_sink_delay_to_node[child] + edge_delay(candidate, child.loc)
balance_penalty = max(child_worsts) - min(child_worsts)
```
最终输出前所有坐标 snap 到整数 grid 并 clamp 到 die 边界内。

---

## 6. Router — Two-Stage Bottom-Up Pattern-First Router

### 问题定义

根据 locer 已确定的固定坐标，为每条 parent-child edge 生成合法的 Manhattan polyline route。

### 核心策略

**6.1 Route Policy 分类**

| Policy | 适用边 | 路由策略 |
|--------|-------|---------|
| `LocalClusterPatternOnly` | sink↔internal↔access 等 cluster 内 | pattern-only (I/L/Z) |
| `ExternalAccessPatternThenMaze` | access↔bridge↔top | pattern-first, A* maze fallback |
| `GlobalPatternThenMaze` | global↔global↔top↔source | pattern-first, A* maze fallback |

**6.2 Two-Stage Bottom-Up Route Order**

Stage A（per-cluster）：先 route 每个 cluster 内部 sink→internal→access 边，再 route 该 cluster 的 access/bridge/top 边，直到 top 接通。按 cluster 内部深度排序，靠近 sink 的短边先走。

Stage B（global）：所有 cluster top 接通后，再从各 top 向 source 方向汇合 route global 主干。靠近 source 的边最后走。

**6.3 Directional Port / Pin Access**

每个 node 有 4 个方向 port（UP/DOWN/LEFT/RIGHT），每个方向最多被一条边占用。

**Tiered preferred direction penalty**（非 binary preference）：
- parent exit 方向等于 dominant geometric direction → penalty 0.0
- 垂直方向 → 0.5
- 反方向 → 1.0
- Child entry 对称处理（用 opposite of dominant_dir 作为最优）。

**6.4 Pattern Candidate Generation**

- I-shape：直接连接（若共线）
- L-shape：HV 和 VH 两种
- Z-shape：枚举 xm/ym 轨道，轨道来源包括 parent/child 坐标±偏移、bbox 边界±偏移、node 坐标附近 track 等
- **Z-shape 硬约束**：wirelength == Manhattan 距离，canonicalize 后 bends == 2

Pattern candidates 按 sorted parent_exit_dirs 和 child_entry_dirs（以 preferred factor 升序）迭代生成。

**6.5 Incremental Legality Check**

每选一条 route，必须实时检查：
- 不越 die 边界
- 不压非本 edge 端点的 node（source、sink、任何内部点都是障碍）
- 不穿 forbidden bbox（非所属 cluster 的 ClusterTop bbox）
- 不同已 commit route segment 非法交集/重叠
- 不自交
- Port 未占用
- 对 `Global → ClusterTop`：允许同 cluster 的已 commit Stage A route 被触碰（受限于 top 周围被局部接入线封口的情况，但不允许 overlap）

**6.6 A* Maze Fallback**

当 pattern candidate 全部失败时，使用 **双向 A\***（bidirectional A*）：
- 同时从 parent（正向）和 child（反向）搜索
- 状态 = (grid point, prev_dir)，使用 `best_g` 表记录
- 正向初始方向加 directional bias（preferred_dir_factor × bend_weight）
- 当双向 frontier 相遇时，拼接 path 并 canonicalize
- 对于 `LocalClusterPatternOnly`，搜索限定在 cluster bbox 扩张窗口内

**6.7 Scoring**

Pattern route：`score = wirelength + preferred_cost + detour_penalty + bbox_penalty + z_center_penalty + reserved_top_port_cost`

Maze route：`score = wirelength + bend_weight×bends + preferred_cost + ...`

关键在于：Z-shape 可以同时满足 parent exit 和 child entry 均为 preferred（cost=0+0），而 L-shape 最多一端 preferred，因此 Z-shape 在 score 上自然优先。

---

## 7. Detourer — Bottom-Up Skew Balancing via Detour Insertion

### 问题定义

不改变 topology、不改变 node loc、不重新 route，仅在已有 route polyline 上插入/升级小绕格来平衡左右子树 skew。

### 核心策略

**7.1 严格 bottom-up 处理顺序**

按照 sink→internal→access→bridge→top→global 的顺序，每次只处理当前 node 的左右 child worst delay 差异。

**7.2 Detour Unit：U-shape 小绕格**

对水平 segment 的 k-level 绕格：
```text
原始: ... → (a,y) → (b,y) → ...
k=1:  ... → (a,y) → (a,y±1) → (b,y±1) → (b,y) → ...
added_delay = 2k
```

**7.3 Detour 插入优先级**

1. 选择较短一侧中最长的 routed edge
2. 在该 edge 内，按 segment 长度降序 → segment 中心距离优先
3. 在每个 segment 上从中心 anchor 开始向两侧扩展
4. 对每个 anchor 尝试两侧方向（先选空旷侧）
5. 优先 k=1；若所有位置 k=1 后仍不够，升级已有 k=1 为 k=2，k=2 为 k=3（需已是 k×(k-1)），直到 `kMaxDetourLevel = 10`

**7.4 关键约束**

- **Endpoint orientation preservation**：距 parent/child endpoint L1 ≤ 1 的位置不插 detour，保证 parent_exit_dir 和 child_entry_dir 不变。
- **Adjacent detour 异向规则**：紧邻的两个 detour 必须朝相反方向，避免同向仅造成局部平移而不创造有效绕行空间。
- 每次插入/升级都必须做完整合法性检查（压 node、压 edge、交叉、重叠、穿 bbox、自交等）。

**7.5 Balancing Objective**

每个 node 只比较左右 child 的 worst delay：
```text
delta = max(left_delay + edge, right_delay + edge) - min(...)
```
选择 `abs(new_delta)` 最小的 detour；若并列则优先不过度超过长边，再优先更少 detour 数、更小 k。

**7.6 Candidate Edge 选择**

除了当前 node→short_child 的直接边，还会考虑 short child subtree 中更长的 routed edges（按 edge length 降序排列），优先在最长的边中插入 detour。

---

## 8. Bufferer — Skew-Driven Buffer Insertion

### 问题定义

在 route/detour 完成后，尝试在 topology node 上插入 buffer 以进一步降低 skew。buffer 不能插在 edge 中间，只能标记在非 sink/source 的 topology node 上。

### 核心策略

**8.1 Skew-first 原则**

bufferer 的主要目标是 skew optimization，不是 fanout-driven insertion。只有满足以下条件才 commit：

```text
new_score > old_score
score = 5000000 - 5000×skew - 50×wirelength - 200×buffer_cost
```

若 score 不改善，rollback 不插入。fanout 只做 legality check，不作为自动插 buffer 的触发器。

**8.2 Delay model with node buffers**

buffer 插在 node 上时，其 delay 计入从该 node 向上传播的所有 sink-to-root path：

```text
sink_delays_to_parent[node] = concat(
    for each child c:
        for d in sink_delays_to_node[c]:
            d + buffer_delay[c] + edge_delay(node, c)
)
```

叶子 sink delay=0。内部 node 汇总所有 child 的 delay + child 的 buffer delay（若有）+ edge delay。

**8.3 Fanout legality**

每个 node bottom-up 维护 `downstream_sink_count`：叶节点=1，内部节点=sum(children)。若 buffer 插在 node 上，必须满足：

```text
buffer_type.max_fanout >= downstream_sink_count[node]
```

不满足则 candidate 直接 reject。

**8.4 Greedy multi-pass selection**

算法流程（最多 `MAX_PASSES=20` 轮）：

1. 收集合法候选 node（非 sink、非 source、尚未插 buffer 的内部 node）
2. 对每个候选 node，枚举所有 buffer type：
   - fanout 合法性检查
   - 临时插入 buffer，recompute 全局 delay profile bottom-up
   - 计算 objective_skew 和 score
   - rollback 临时插入
3. 选择 score 改善最大的 candidate（同 score 时选 skew 更低的）
4. 若无可改善 candidate，停止
5. commit 最优 candidate，进入下一轮

**8.5 Objective skew 计算**

objective_skew 计算 source virtual root 的 skew：若 source 有多个 children（`tree.source_children`），汇总所有 source child 的 delay；若 `tree.root` 有效，使用 root 的 subtree skew。

**8.6 Buffer 位置约束**

- 允许：internal、access、bridge、top、global node
- 禁止：sink、source、edge midpoint、route turning point
- 每个 node 最多一个 buffer；已 commit buffer 的 node 不再尝试
- buffer 不改变 route polyline、endpoint、port

---

## 9. Writer

### 策略：Edge Route 拼接 + Buffer 输出

- 从 sink leaf node 沿 topology parent 链上溯到 source child
- 按 segment 拼接各条 edge polyline，去重追加
- 每个 committed buffer 输出为 `b <id> <type> <node_id> <x> <y>`
- 最终输出：`result/sample_k_solution.txt`

---

## 10. 未实现的功能（TODO）

1. **2D global distribution**：当前 global placement 是 order-constrained 一维链，尚待考虑更完整的 2D 全局分布。Buffer insertion 已完成（见 Section 8）。

---

## 11. 关键几何/数据架构决策

- **DME 几何全部使用 rotated coordinate (u, v)**：`u=x+y, v=x-y`，Manhattan 距离 = `max(Δu, Δv)`。TRR 在 (u,v) 中为 axis-aligned rectangle。
- **全局使用整数坐标**：locer 输出前 snap 到整数 grid，router 用 scale=1 的整数 grid routing。
- **Delay model**：全程用 Manhattan 距离 = delay（1 edge = 1 delay），buffer delay 来自 `BufferType.delay`。
- **Deterministic**：所有排序使用稳定的比较函数，tie-break 通常为 node id。
