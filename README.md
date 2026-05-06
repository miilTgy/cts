# Buffered-DME (Deferred Merged Embeddeing)

topology generation -> DME buttom-up with optional buffer -> DME top-down

## DME Bottom-up with Optional Buffer

This version follows the original DME bottom-up flow, but when calculating the merging segment of an internal node, it enumerates buffer insertion on the left/right child subtrees. The selected case is the one with the minimum incremental cost.

```text
Input:
    set of sinks S
    binary topology G(S, Top)
    buffer types B

Output:
    merging segments ms(v)
    edge lengths |ev|
    selected buffer choices on child subtrees

foreach node v in G, in bottom-up order:
    if v is a sink node:
        ms[v] = PL(v)
        delay[v] = 0
        cost[v] = 0
        sink_count[v] = 1
    else:
        (a, b) = CHILDREN(v)
        sink_count[v] = sink_count[a] + sink_count[b]

        best_delta_cost = INF
        best_case = NONE

        for each buffer option Ba on child subtree a:
            // Ba can be no_buffer, or one legal buffer type inserted at a
            if Ba is not no_buffer and Ba.max_fanout < sink_count[a]:
                continue

            for each buffer option Bb on child subtree b:
                // Bb can be no_buffer, or one legal buffer type inserted at b
                if Bb is not no_buffer and Bb.max_fanout < sink_count[b]:
                    continue

                delay_a = delay[a] + delay(Ba)
                delay_b = delay[b] + delay(Bb)
                buffer_cost = cost(Ba) + cost(Bb)

                CALC_EDGE_LENGTH(ea, eb, delay_a, delay_b)
                    // choose |ea| and |eb| so that:
                    //     |ea| + delay_a = |eb| + delay_b
                    // and |ea| + |eb| is minimum

                trr[a][core]   = ms[a]
                trr[a][radius] = |ea|
                trr[b][core]   = ms[b]
                trr[b][radius] = |eb|

                candidate_ms = trr[a] intersection trr[b]
                if candidate_ms is empty:
                    continue

                delta_cost = 50 * (|ea| + |eb|)
                           + 200 * buffer_cost

                if delta_cost < best_delta_cost:
                    best_delta_cost = delta_cost
                    best_case = (Ba, Bb, |ea|, |eb|, candidate_ms)

        select best_case
        ms[v] = best_case.candidate_ms
        |ea| = best_case.|ea|
        |eb| = best_case.|eb|
        record selected buffers Ba, Bb if they are not no_buffer

        delay[v] = |ea| + delay[a] + delay(Ba)
                 = |eb| + delay[b] + delay(Bb)
        cost[v] = cost[a] + cost[b] + cost(Ba) + cost(Bb)
```

Notes:
- `no_buffer` is included as one buffer option.
- A real buffer option is legal only when `buffer.max_fanout >= sink_count(child)`.
- Buffer delay changes the child subtree delay exposed to the parent, so it must be considered before computing the merging segment.
- The selected case minimizes only the incremental cost of this merge:

```text
incremental_cost = 50 * added_wirelength + 200 * added_buffer_cost
```

- More accurate versions may keep multiple states per node, but this simplified version keeps only the locally best case.
