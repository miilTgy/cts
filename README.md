# Wire delay

1 edge 1 delay

# Buffered-DME (Deferred-Merge Embedding)

topology generation -> DME bottom-up with optional buffer -> DME top-down

## DME Bottom-up with Optional Buffer

This version follows the original DME bottom-up flow, but each subtree keeps both the minimum and maximum sink delay exposed to its root. Therefore, the algorithm can still evaluate non-zero-skew approximate merges correctly. When merging two child subtrees, it enumerates whether to insert a buffer at the left/right child branching node, and selects the case with the minimum estimated global objective cost.

In this homework, wire delay follows the evaluator definition: every unit Manhattan tree edge contributes 1 delay. Therefore, `edge_to_a` and `edge_to_b` are unit-grid path lengths, not the number of branch points.

```text
Input:
    set of sinks S
    binary topology G(S, Top)
    buffer types B

Output:
    merging segments ms(v)
    edge lengths |ev|
    selected buffer choices at internal branching nodes
    min/max subtree delays

foreach node v in G, in bottom-up order:
    if v is a sink node:
        ms[v] = PL(v)
        min_delay[v] = 0
        max_delay[v] = 0
        skew[v] = 0
        wire_est[v] = 0
        buffer_cost[v] = 0
        total_cost[v] = 0
        sink_count[v] = 1
    else:
        (a, b) = CHILDREN(v)
        sink_count[v] = sink_count[a] + sink_count[b]

        best_total_cost = INF
        best_case = NONE

        for each buffer option Ba at branching node a:
            // Ba can be no_buffer, or one legal buffer type inserted at node a.
            // Since a is an internal branching node, it is not a sink coordinate.
            if Ba is not no_buffer and Ba.max_fanout < sink_count[a]:
                continue

            for each buffer option Bb at branching node b:
                // Bb can be no_buffer, or one legal buffer type inserted at node b.
                // Since b is an internal branching node, it is not a sink coordinate.
                if Bb is not no_buffer and Bb.max_fanout < sink_count[b]:
                    continue

                ba_delay = delay(Ba)
                bb_delay = delay(Bb)
                added_buffer_cost = cost(Ba) + cost(Bb)

                // For exact zero-skew DME estimation, first try to choose |ea| and |eb|
                // so that the two child delay intervals are aligned as well as possible.
                // When both child subtrees are already zero-skew, this reduces to:
                //     |ea| + ba_delay + min_delay[a]
                //       = |eb| + bb_delay + min_delay[b]
                CALC_EDGE_LENGTH(ea, eb, min_delay[a], max_delay[a], ba_delay,
                                              min_delay[b], max_delay[b], bb_delay)

                trr[a][core]   = ms[a]
                trr[a][radius] = |ea|
                trr[b][core]   = ms[b]
                trr[b][radius] = |eb|

                candidate_ms = trr[a] intersection trr[b]
                if candidate_ms is empty:
                    // Exact merging is impossible for this buffer choice.
                    // Fall back to the closest approximate merging candidate.
                    candidate_ms = APPROX_MERGING_SEGMENT(trr[a], trr[b])
                    edge_to_a = APPROX_EDGE_LENGTH(candidate_ms, ms[a])
                    edge_to_b = APPROX_EDGE_LENGTH(candidate_ms, ms[b])
                else:
                    edge_to_a = |ea|
                    edge_to_b = |eb|

                a_min = min_delay[a] + edge_to_a + ba_delay
                a_max = max_delay[a] + edge_to_a + ba_delay
                b_min = min_delay[b] + edge_to_b + bb_delay
                b_max = max_delay[b] + edge_to_b + bb_delay

                candidate_min_delay = min(a_min, b_min)
                candidate_max_delay = max(a_max, b_max)
                candidate_skew = candidate_max_delay - candidate_min_delay

                candidate_wire_est = wire_est[a] + wire_est[b]
                                   + edge_to_a + edge_to_b

                candidate_buffer_cost = buffer_cost[a] + buffer_cost[b]
                                      + added_buffer_cost

                candidate_total_cost = 5000 * candidate_skew
                                     + 50   * candidate_wire_est
                                     + 200  * candidate_buffer_cost

                if candidate_total_cost < best_total_cost:
                    best_total_cost = candidate_total_cost
                    best_case = (Ba, Bb,
                                 edge_to_a, edge_to_b,
                                 candidate_ms,
                                 candidate_min_delay,
                                 candidate_max_delay,
                                 candidate_skew,
                                 candidate_wire_est,
                                 candidate_buffer_cost,
                                 candidate_total_cost)

        select best_case
        ms[v] = best_case.candidate_ms
        edge_to_a = best_case.edge_to_a
        edge_to_b = best_case.edge_to_b
        record selected buffers Ba, Bb if they are not no_buffer

        min_delay[v] = best_case.candidate_min_delay
        max_delay[v] = best_case.candidate_max_delay
        skew[v] = best_case.candidate_skew
        wire_est[v] = best_case.candidate_wire_est
        buffer_cost[v] = best_case.candidate_buffer_cost
        total_cost[v] = best_case.candidate_total_cost
```

Notes:
- `no_buffer` is explicitly included as one buffer option, so the algorithm enumerates both inserting and not inserting a buffer on each child branching node.
- A real buffer option is legal only when `buffer.max_fanout >= sink_count(child)`.
- A selected buffer is placed at the corresponding internal child branching node, not at a sink. This is valid because a branching node cannot be a sink coordinate in this construction.
- Buffer delay shifts the entire child delay interval, so it must be added to both `min_delay[child]` and `max_delay[child]` before evaluating skew.
- The selected case minimizes the estimated global objective cost of the subtree rooted at `v`, rather than only the incremental cost of the current merge:

```text
total_cost = 5000 * subtree_skew
           + 50   * estimated_subtree_wirelength
           + 200  * subtree_buffer_cost
```

- Subtree skew is defined using the full delay interval of all downstream sinks:

```text
subtree_skew = max_delay[v] - min_delay[v]
```

- For a candidate merge, the two child delay intervals are:

```text
child_a_delay_interval = [min_delay[a] + edge_to_a + delay(Ba),
                          max_delay[a] + edge_to_a + delay(Ba)]

child_b_delay_interval = [min_delay[b] + edge_to_b + delay(Bb),
                          max_delay[b] + edge_to_b + delay(Bb)]
```

- Therefore, the candidate merge skew is:

```text
candidate_skew = max(child_a_max, child_b_max)
               - min(child_a_min, child_b_min)
```

- If exact zero-skew merging is available, `candidate_skew` may remain 0. Otherwise an approximate merging candidate can still be kept, but it is penalized by the skew term.
- Shared trunk length above node `v` is common to all sinks below `v`, so it does not change `skew[v]`. However, every unit Manhattan edge still contributes to the absolute arrival delay from the source to each sink in the final evaluator.
- `wire_est` is a bottom-up estimate. The final wirelength should still be recomputed after top-down embedding by expanding all routed paths into unit Manhattan edges and counting shared edges only once.
