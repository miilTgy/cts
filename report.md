# HW3 Report: Wire Delay CTS — Algorithmic Strategies

## Pipeline Overview

```
Parser → Partitioner → Partreer (cluster topology) → Treer (global tree)
→ Locer (DME BU/TD + congestion-aware placement) → Router (Manhattan routing)
→ Detourer (skew balancing) → Bufferer (buffer insertion for skew) → Writer (solution output)
```

Priority: legal routability > skew potential > buffer cost > wirelength

Scoring formula:

```text
score = 5000000 - 5000 × skew - 50 × wirelength - 200 × buffercost
```

---

## 1. Parser

### Strategy: Keyword-Driven Structured Parsing

- Reads in fixed order: `DIE` → `SOURCE` → `NUM_SINKS` → `SINK ...` → `NUM_BUFFERS` → `BUFFER_TYPE ...`
- Deduplication and validation of all node IDs (source, sinks, buffer types)
- All coordinates checked for die boundary legality (including boundary points)
- **Fail-early** strategy: any format/semantic error immediately sets `valid=false` and records a detailed error message

---

## 2. Partitioner — Adaptive Multi-Gap Partition

### Problem Definition

Recursively partition the set of sinks on the plane into a hierarchical spatial cluster tree. Each leaf cluster is later used by Partreer to build local topology.

### Core Algorithm Strategy

**2.1 Adaptive Big Gap Detection**

For the current sink set, evaluate split quality along the X and Y axes respectively. For each axis:
1. Sort by coordinate, compute the Manhattan gap between all adjacent sinks.
2. Collect all positive gaps and use the **median** as `small_gap`.
3. Big gap qualification:
   ```
   gap >= min_abs_gap (10)  AND  gap >= gap_ratio (4.0) × small_gap
   ```
   If there are no positive gaps, the axis cannot be split.

**2.2 Multi-Way Split and Outlier Handling**

- Use all big gaps on the chosen axis to cut into multiple contiguous groups (multi-way split, not binary).
- Each group must contain at least 2 sinks; if a group has only 1 sink:
  - If adjacent gaps satisfy `outlier_gap_ratio = 3.5` and `outlier_min_abs_gap = 20`, keep it as a single-sink outlier.
  - Otherwise, merge the single sink into the smaller adjacent group (left first, then right).
- If the final group count ≤ 1, the axis candidate is invalid.

**2.3 Axis Selection**

Score X and Y axis candidates:

```text
score = total_big_gap
      + (is_major_axis ? 0.5 : 0.0)   // prefer the bbox long axis
      - num_tiny_groups × 0.3          // penalize size=2 groups
```

Note: the code actually uses `gap_ratio = 4.0`, `outlier_gap_ratio = 3.5`, `min_cluster_size = 4`; the comment constants are suggested initial values.

**2.4 Recursion Termination**

- `n ≤ min_cluster_size` (4) → leaf cluster
- Neither axis produces a valid candidate → leaf cluster (referred to in the code as "no axis")

---

## 3. Partreer — Pair-first Access Tree

### Problem Definition

Inside a single cluster, progressively pair sinks into a binary topology, forming a local tree with `ClusterAccess` as the entry point and `ClusterTop` as the unique external root.

### Core Algorithm Strategy

**3.1 Multi-Level Candidate Pairing**

At each level:
1. **Candidate generation**: generate candidate edge pairs from X-sorted and Y-sorted k-nearest (k=3) and Manhattan-nearest neighbors.
2. **Pair cost**:
   ```text
   cost = manhattan(a.loc, b.loc)
        + 0.10 × bbox_penalty          // perimeter of merged bbox
        + 0.50 × interleave_penalty    // number of other active nodes crossed
        + 0.05 × skew_penalty          // subtree skew difference
   ```
3. **Matching**:
   - When active nodes ≤ 22, use **bitmask DP** to compute the exact minimum-cost matching.
   - When active nodes > 22, use a **greedy** approach, selecting unused pairs in cost order.
   - Allow **carry** (unmatched nodes propagate to the next level) to avoid forcing bad pairs.
4. **Tapping node location**: take the midpoint of the left and right children, then apply a source-aware bias (±1 offset) based on `external_target`.

**3.2 CONNECTABLE Check (Segment Intersection Method)**

The candidate segment `(a, b)` of a new pair must not illegally intersect existing topology segments:
- Use **cross product** to detect segment intersection (`cross(a, b, c) * cross(a, b, d) < 0`)
- T-junctions (an endpoint lying on another segment) are also illegal
- Collinear overlap: illegal if the two segments are on the same line and their projected intervals overlap
- The only exception: two segments sharing exactly one topology node endpoint

After connecting the parent, check that the actual segments for `parent→left` and `parent→right` are legal against existing segments.

**3.3 Tapping Node Location Search**

Instead of using only the midpoint, enumerate multiple candidate coordinates (biased midpoint, midpoint, left/right endpoints, `external_target`, bbox corners, die corners, etc.) and select the first location that satisfies all connectivity constraints.

**3.4 Access Tree Construction and Canonicalize**

- Pairing stop condition: active node count ≤ `target_access_points` (3) or reaching `max_pair_levels` (8) without convergence
- Access tree construction: ensure the hierarchy `ClusterAccess` → `ClusterBridge` → `ClusterTop`
- **Canonicalize**: repeatedly absorb comparable unary parent-child pairs (partial order: `ClusterTop > ClusterBridge > ClusterAccess > ClusterInternal`), eliminating redundant unary wrappers
- `ClusterSink` can directly replace `ClusterAccess/ClusterInternal`

---

## 4. Treer — Source-aware Global Tree Construction

### Problem Definition

Assemble the cluster topologies from leaf partitions into a global binary tree, then apply source-aware geometric corrections.

### Core Algorithm Strategy

**4.1 Recursive Partition Hierarchy Traversal**

- For leaf partitions, call `partreer::build()` to generate the cluster-internal topology.
- For internal partitions, recursively process children, then collect the child roots.
- **Key**: the recursively merged result is only intermediate; the final global tree is overwritten by source-aware geometric reconstruction.

**4.2 Source-Aware Geometric Global Topology Correction**

After collecting all `ClusterTop` roots, rebuild the global tree according to the source position:

1. **Determine source position relative to the cluster union bbox**:
   - Source on the left → sweep by X ascending
   - Source on the right → sweep by X descending
   - Source below → sweep by Y ascending
   - Source above → sweep by Y descending
   - Source inside → use the axis with larger spread as the primary sweep axis

2. **Source as a dividing point**: when the source is inside the union bbox, split clusters into those on the negative side of the source and those on the positive side, build ordered chains separately, and finally connect both sides with a Global root near the source.

3. **Ordered chain-like binary tree** (not balanced recursive merge):
   ```text
   root = Cn-1
   for i = n-2 downto 0:
       root = new Global(left = Ci, right = root)
   ```
   This ensures the spatial access order follows the sweep direction and avoids cross-source back-tracking.

**4.3 Canonicalize and Source Absorption**

- If the source has only one Global child and they coincide, absorb that Global into the source (the source connects directly to the children without an intermediate node).
- Repeatedly absorb comparable unary parent-child pairs to eliminate redundancy.

---

## 5. Locer — DME BU/TD + Congestion-Aware Placement

### Problem Definition

Determine geometric coordinates for every node in the topology. Internal nodes (sink/internal/access) use DME; external nodes (bridge/top/global) use a congestion-aware heuristic.

### 5.1 Stage 0: Cluster-Level preferred_side Pre-Evaluation

- For each `ClusterTop`, first find an external anchor from the treer topology (prefer parent treer loc → ancestor global loc → source → die center, falling back accordingly).
- Compute `dx = anchor.x - bbox_center.x`, `dy = anchor.y - bbox_center.y`.
- Dominant direction: `abs(dx) >= abs(dy)` → prefer X side, otherwise prefer Y side.
- **Tie handling**: when `abs(dx) - abs(dy)` ≤ tie_eps, use congestion to assist selection (compute distance/crossing risk from each side to other cluster bboxes); if still tied, use the order `TOP, RIGHT, BOTTOM, LEFT`.
- All access/bridge/top nodes within the cluster inherit the same preferred_side.

### 5.2 Cluster-Internal DME (BU/TD)

**(a) Bottom-Up (BU) — Buffered DME**

Uses **(u, v) rotated coordinates**:
```text
u = x + y,   v = x - y
Manhattan dist = max(|u1-u2|, |v1-v2|)
```
TRR (Tilted Rectangular Region) is an axis-aligned rectangle in (u, v).

Core flow:
1. Leaf sink: ms = point segment at the sink location.
2. Internal node:
   - Construct base TRR from child merging segments.
   - Allocate edge lengths:
     ```text
     l_mid = avg(left delay), r_mid = avg(right delay)
     D = min distance between left.ms and right.ms
     raw_left = (D + r_mid - l_mid) / 2
     rL = clamp(raw_left, 0, D),  rR = D - rL
     ```
   - If the initial radii yield no legal intersection, **first try rL = rR**, then use **exponential repair**: `extra = 0, 1, 2, 4, 8, ...` to gradually increase both radii until expand_trr intersects.
   - **Buffer enumeration**: for each child, enumerate all buffer types satisfying `max_fanout >= child.sink_count` plus no_buffer. Sink nodes cannot have buffers.
   - Select the `(buffer_L, buffer_R)` combination minimizing `total_cost = 5000×skew + 50×wire_est + 200×buffer_cost`.
3. MS extraction priority:
   ```
   (1) BOUNDARY_INTERSECTION: take expand_left.boundary ∩ expand_right.boundary
   (2) INTER_BOUNDARY_FALLBACK: take the intersection's own boundary
   (3) INTERIOR_FALLBACK: take the longest axis-aligned segment within the intersection
   ```
   Boundary candidates are prioritized by segment length; ties are broken by the symmetry of the midpoint distance to the base.

**(b) Top-Down (TD) — Location Assignment**

Traverse from the access root downward to determine each node location:
1. For each child: `feasible_ms = child.ms ∩ TRR(parent_loc, assigned_edge)`
2. If feasible is non-empty: generate candidate points (including feasible_ms midpoint, endpoints, parent_loc projections, each sink coordinate ±1 projections, etc.) and select the best by `loc_score` (at most 32 candidates).
3. `loc_score = 10000×max(0, dist-assigned) + 100×lshape_penalty + |dist-assigned| + 0.001×dist`
4. If feasible is empty: fallback to the nearest point on child.ms to parent_loc.

### 5.3 Congestion-Aware Outer Placement

**(a) Bridge placement**

- Inherit cluster preferred_side; prefer candidate points within the selected_side corridor (width = 15% of max bbox dimension)
- Do not use bbox center as a candidate anchor (avoids bridges being drawn into the cluster center)
- Apply preferred-side monotonic constraint (e.g., LEFT: bridge.x ≤ child.x)
- Scoring:
  ```
  score = 300×skew + 200×congestion + 100×crossing + 20×wire
        + 50×bbox_inside + 250×deep_inside + 300×mono_violation
        + 10×side_switch
  ```
- If all candidates on the preferred_side are illegal, fallback to other sides.

**(b) Top placement**

- Must be strictly outside the related cluster bbox (hard constraint)
- Likewise inherit cluster preferred_side; fallback to other-side outside candidates
- Allow using the bbox center's preferred_side projection as a candidate

**(c) Global placement — Order-Constrained**

The global coordinates from treer are only used to infer ordering; the locer re-selects points:
- Extract the primary axis (X or Y) and global chain order
- Each global node has an order interval: `[prev_axis + min_gap, next_axis - min_gap]`
- Source-side constraint: globals must not cross to the other side of the source
- Candidate sources: source axis, child/top loc, bbox projections, uniform spacing reference points
- Scoring:
  ```
  score = 400×skew + 250×congestion + 120×crossing + 20×wire
        + 1000×top_attachment_penalty + 50×imbalance + 220×trunk_penalty
        + INF×order_violation
  ```
- Globals directly connecting to `ClusterTop` receive a strong `L1(global, top_loc)` penalty

### 5.4 Delay Profile Maintenance

Each node maintains `sink_delays_to_node` (the list of delays from every sink to this node).
Scoring uses child worst-delay balance as the primary term:
```text
child_worst = max_sink_delay_to_node[child] + edge_delay(candidate, child.loc)
balance_penalty = max(child_worsts) - min(child_worsts)
```
Before final output, all coordinates are snapped to integer grid and clamped within die boundaries.

---

## 6. Router — Two-Stage Bottom-Up Pattern-First Router

### Problem Definition

Given the fixed coordinates determined by the locer, generate legal Manhattan polyline routes for each parent-child edge.

### Core Strategy

**6.1 Route Policy Classification**

| Policy | Applicable Edges | Routing Strategy |
|--------|-----------------|-----------------|
| `LocalClusterPatternOnly` | sink↔internal↔access (intra-cluster) | pattern-only (I/L/Z) |
| `ExternalAccessPatternThenMaze` | access↔bridge↔top | pattern-first, A* maze fallback |
| `GlobalPatternThenMaze` | global↔global↔top↔source | pattern-first, A* maze fallback |

**6.2 Two-Stage Bottom-Up Route Order**

Stage A (per-cluster): first route the edges sink→internal→access within each cluster, then route the access/bridge/top edges of that cluster until the top is connected. Process in order of cluster-internal depth; shorter edges closer to sinks go first.

Stage B (global): after all cluster tops are connected, route the global trunk from each top toward the source. Edges closer to the source go last.

**6.3 Directional Port / Pin Access**

Each node has 4 directional ports (UP/DOWN/LEFT/RIGHT); each direction can be occupied by at most one edge.

**Tiered preferred direction penalty** (not binary preference):
- parent exit direction equals the dominant geometric direction → penalty 0.0
- Perpendicular direction → 0.5
- Opposite direction → 1.0
- Child entry is handled symmetrically (using opposite of dominant_dir as optimal).

**6.4 Pattern Candidate Generation**

- I-shape: direct connection (if collinear)
- L-shape: HV and VH variants
- Z-shape: enumerate xm/ym track values derived from parent/child coordinates ± offsets, bbox boundaries ± offsets, tracks near node coordinates, etc.
- **Z-shape hard constraint**: wirelength == Manhattan distance, canonicalized bends == 2

Pattern candidates iterate through sorted parent_exit_dirs and child_entry_dirs (by ascending preferred factor).

**6.5 Incremental Legality Check**

After each route is selected, it must be checked in real time:
- Does not cross the die boundary
- Does not hit any node that is not an endpoint of this edge (source, sinks, and all internal points are obstacles)
- Does not enter a forbidden bbox (ClusterTop bboxes of non-owned clusters)
- Does not illegally intersect/overlap with already-committed route segments
- No self-intersection
- Port not already occupied
- For `Global → ClusterTop`: allows touching already-committed Stage A routes of the same cluster (to handle cases where the top's surrounding is sealed by local access lines, but no overlap is allowed)

**6.6 A* Maze Fallback**

When all pattern candidates fail, use **bidirectional A***:
- Search simultaneously from parent (forward) and child (backward)
- State = (grid point, prev_dir), tracked with a `best_g` table
- Apply directional bias to the forward initial direction (`preferred_dir_factor × bend_weight`)
- When the two frontiers meet, splice the path and canonicalize
- For `LocalClusterPatternOnly`, the search is confined within the cluster bbox expanded window

**6.7 Scoring**

Pattern route: `score = wirelength + preferred_cost + detour_penalty + bbox_penalty + z_center_penalty + reserved_top_port_cost`

Maze route: `score = wirelength + bend_weight×bends + preferred_cost + ...`

The key insight: a Z-shape can simultaneously satisfy both parent exit and child entry as preferred (cost=0+0), whereas an L-shape can have at most one end preferred, so Z-shapes naturally have a scoring advantage.

---

## 7. Detourer — Bottom-Up Skew Balancing via Detour Insertion

### Problem Definition

Without changing the topology, node locations, or re-routing, insert/upgrade small detours on existing route polylines to balance left/right subtree skew.

### Core Strategy

**7.1 Strict Bottom-Up Processing Order**

Process in the order: sink → internal → access → bridge → top → global. For each node, only handle the worst-delay difference between its left and right children.

**7.2 Detour Unit: U-Shape Bump**

For a k-level detour on a horizontal segment:
```text
Original: ... → (a,y) → (b,y) → ...
k=1:      ... → (a,y) → (a,y±1) → (b,y±1) → (b,y) → ...
added_delay = 2k
```

**7.3 Detour Insertion Priority**

1. Choose the longest routed edge on the shorter side
2. Within that edge, sort segments by descending length → proximity to segment center
3. On each segment, start from the center anchor and expand outward
4. For each anchor, try both directions (prefer the side with more free space)
5. Prefer k=1; if all positions are filled at k=1 and still insufficient, upgrade existing k=1 to k=2, k=2 to k=3 (must already be at k×(k-1)), up to `kMaxDetourLevel = 10`

**7.4 Key Constraints**

- **Endpoint orientation preservation**: do not insert detours within L1 ≤ 1 of parent/child endpoints, to keep parent_exit_dir and child_entry_dir unchanged.
- **Adjacent detour alternating rule**: two tightly adjacent detours must face opposite directions, to avoid merely translating a segment without creating effective routing space.
- Every insertion/upgrade requires a full legality check (hitting nodes, hitting edges, crossing, overlap, bbox penetration, self-intersection, etc.).

**7.5 Balancing Objective**

At each node, only compare the worst delays of the left and right children:
```text
delta = max(left_delay + edge, right_delay + edge) - min(...)
```
Select the detour that minimizes `abs(new_delta)`; in case of ties, prefer not overshooting the longer side, then prefer fewer detours, then prefer smaller k.

**7.6 Candidate Edge Selection**

In addition to the direct edge from the current node to short_child, also consider longer routed edges within the short child's subtree (sorted by descending edge length), prioritizing detour insertion on the longest edges.

---

## 8. Bufferer — Skew-Driven Buffer Insertion

### Problem Definition

After routing and detouring, attempt to insert buffers on topology nodes to further reduce skew. Buffers cannot be inserted in the middle of an edge; they can only be placed on non-sink, non-source topology nodes.

### Core Strategy

**8.1 Skew-First Principle**

The bufferer's primary goal is skew optimization, not fanout-driven insertion. A candidate is committed only if:

```text
new_score > old_score
score = 5000000 - 5000×skew - 50×wirelength - 200×buffer_cost
```

If the score does not improve, rollback without inserting. Fanout is used only for legality checks, not as a trigger for automatic buffer insertion.

**8.2 Delay Model with Node Buffers**

When a buffer is placed on a node, its delay is added to all sink-to-root paths that pass upward through that node:

```text
sink_delays_to_parent[node] = concat(
    for each child c:
        for d in sink_delays_to_node[c]:
            d + buffer_delay[c] + edge_delay(node, c)
)
```

Leaf sink delay = 0. Internal nodes aggregate all children's delays + each child's buffer delay (if any) + edge delay.

**8.3 Fanout Legality**

Each node maintains `downstream_sink_count` bottom-up: leaf node = 1, internal node = sum(children). If a buffer is placed on a node, it must satisfy:

```text
buffer_type.max_fanout >= downstream_sink_count[node]
```

Candidates that violate this condition are directly rejected.

**8.4 Greedy Multi-Pass Selection**

Algorithm flow (at most `MAX_PASSES=20` passes):

1. Collect legal candidate nodes (non-sink, non-source, internal nodes not yet buffered)
2. For each candidate node, enumerate all buffer types:
   - Check fanout legality
   - Temporarily insert buffer, recompute the global delay profile bottom-up
   - Compute objective_skew and score
   - Rollback the temporary insertion
3. Select the candidate with the greatest score improvement (ties broken by lower skew)
4. If no beneficial candidate exists, stop
5. Commit the best candidate and proceed to the next pass

**8.5 Objective Skew Computation**

Objective skew is computed at the source virtual root: if the source has multiple children (`tree.source_children`), aggregate delays across all source children; if `tree.root` is valid, use the root's subtree skew.

**8.6 Buffer Placement Constraints**

- Allowed: internal, access, bridge, top, global nodes
- Forbidden: sink, source, edge midpoint, route turning point
- At most one buffer per node; already-committed buffer nodes are not retried
- Buffers do not change route polylines, endpoints, or ports

---

## 9. Writer

### Strategy: Edge Route Concatenation + Buffer Output

- Trace from each sink leaf node up the topology parent chain to a source child
- Concatenate edge polylines segment by segment, deduplicating consecutive identical points
- Each committed buffer is output as `b <id> <type> <node_id> <x> <y>`
- Final output: `result/sample_k_solution.txt`

---

## 10. Unimplemented Features (TODO)

1. **2D global distribution**: the current global placement is an order-constrained 1D chain; a more complete 2D global distribution remains to be considered. Buffer insertion is already complete (see Section 8).

---

## 11. Key Geometric / Data Architecture Decisions

- **All DME geometry uses rotated coordinates (u, v)**: `u = x + y`, `v = x - y`. Manhattan distance = `max(Δu, Δv)`. TRRs are axis-aligned rectangles in (u, v).
- **Global integer coordinates**: the locer snaps to integer grid before output; the router uses scale=1 integer-grid routing.
- **Delay model**: Manhattan distance = delay throughout (1 edge = 1 delay). Buffer delay comes from `BufferType.delay`.
- **Deterministic**: all sorting uses stable comparison functions; tie-breaks are typically by node id.
