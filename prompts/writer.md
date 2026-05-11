# writer: router result solution output generator

请实现 Writer 模块：把当前工程实际链路

```text
parser -> partitioner -> treer -> locer -> router -> writer
```

产出的最终 routing 结果写入 `result/<basename>_solution.txt`，使该 solution 能被 `evaluate.cpp` 读取评估。

Writer 只负责格式化输出和一致性检查，不负责选择 node 位置、不负责生成 parent-child route、不负责补 detour、不负责绕障、不负责 skew balancing。所有具体布线几何必须来自 `common::RouterResult::edge_debugs[*].polyline`。如果 writer 产出的结果被 evaluator 报错，不要为了通过 evaluator 改 writer 的几何输出逻辑，而是报告 evaluator 的原始错误信息。

## 1. 输出格式

作业输出格式为：

```text
NUM_BUFS <K>
BUF <buf_id> <type> <x> <y>
...
NUM_ROUTES <N>
ROUTE <sink_id> <P>
<x_0> <y_0>
<x_1> <y_1>
...
```

当前没有最终 buffer placement stage，因此 writer 必须输出：

```text
NUM_BUFS 0
```

除非后续有独立 stage 提供最终 buffer placement，否则 writer 不得凭 BU/TD/估计信息输出 buffer。

`NUM_ROUTES` 必须等于 `problem.sinks.size()`。route 输出顺序必须与 input sink 顺序一致。每条 route 第一个点必须是 source 坐标，最后一个点必须是对应 sink 坐标；相邻点必须形成 Manhattan segment。

输出路径：

```text
result/<basename>_solution.txt
```

例如输入 `samples/sample1.txt`，输出 `result/sample1_solution.txt`。

## 2. API

`include/writer.h`：

```cpp
#pragma once

#include <string>
#include "common.h"

namespace writer {

struct WriterResult {
    bool valid = false;
    std::string error_msg;
    std::string output_path;
};

void debug_enable(bool enable);

WriterResult write_solution(const std::string& input_path,
                            const common::Problem& problem,
                            const common::TopoTree& tree,
                            const common::LocerResult& loc_result,
                            const common::RouterResult& router_result);

}  // namespace writer
```

Writer 不再接收 `BottomUpResult` 或 `TopDownResult`，也不得读取 `final_route_to_parent`、`route_to_parent`、`compensation_snake_to_parent` 来构造 solution。

## 3. main.cc 接入

在 `router::run()` 成功之后调用：

```cpp
writer::WriterResult writer_result =
    writer::write_solution(argv[1], problem, tree, loc_result, route_result);

if (!writer_result.valid) {
    std::cerr << "Writer error: " << writer_result.error_msg << "\n";
    return 1;
}

std::cout << "Wrote solution: " << writer_result.output_path << "\n";
```

构建文件应包含 `src/writer.cc` 和 `include/writer.h`。

## 4. Route 构造规则

Writer 的唯一 routing 几何来源是：

```cpp
router_result.edge_debugs[*].polyline
```

实现步骤：

1. 为所有成功 router edge 建立 map：

```text
key = (parent, child)
source edge key = (-1, child)
value = rounded router polyline
```

2. 每条 router edge 必须满足：
   - `routed == true`
   - `failure_reason == "OK"`
   - `polyline.size() >= 2`
   - `parent` / `child` id 合法，source edge 的 parent 为 `-1`
   - polyline 起点等于 parent loc，source edge 起点等于 source loc
   - polyline 终点等于 child loc
   - 所有点 round 后在 die 内
   - 所有相邻点是 Manhattan segment
   - round 后允许丢弃连续重复点；这只用于消除整数化产生的 zero-length serialization 点，不得重排、插入或绕线
   - 丢弃连续重复点后仍必须至少有两个点

3. 对每个 sink：
   - 找到 `tree.nodes[node_id].is_sink && sink_index == input sink index` 的 leaf。
   - 沿 parent 指针回溯到 source child/root，再反转为 source-side 到 leaf 的 node path。
   - 若 `tree.source_children` 非空，path 第一个节点必须属于 `source_children`。
   - 若 `tree.source_children` 为空但 `tree.root` 合法，path 第一个节点必须是 `tree.root`，并使用 `(-1, tree.root)` source edge。
   - 先追加 `(-1, path.front())` 的 router polyline。
   - 再按 path 中每个 `parent -> child` 原样追加对应 router polyline。
   - 拼接时只跳过相邻 edge 共享的第一个点；除此之外只允许丢弃 round 后连续重复的 serialization 点，不允许插入、重排或改动任何 routing 中间点。

4. Writer 不允许使用 minimal connector，不允许从 loc 自己生成 L-shape，不允许补 source-to-root 或 leaf-to-sink。

## 5. 坐标离散化

solution 只能输出整数坐标。

实现：

```cpp
static bool is_near_integer(double v);
static int round_to_int(double v);
static IntPoint round_point(common::SegmentPoint p);
```

规则：

- 若 `abs(v - round(v)) <= 1e-6`，直接 round。
- 若不是近似整数，也允许 round 到最近整数，但 debug warning。
- round 是唯一允许的几何变化；round 后相邻点相同可以删除重复点以满足整数输出格式，但不得借此修补非 Manhattan、端点错误或缺失 edge。

## 6. 合法性检查

`write_solution()` 至少检查：

- `problem.valid == true`
- `tree.valid == true`
- `loc_result.valid == true`
- `router_result.valid == true`
- `tree.nodes` 非空
- `loc_result.node_results.size() == tree.nodes.size()`
- source/sink 坐标在 die 内
- 每个用于 route 的 locer node valid，loc finite，round 后在 die 内
- 每个 input sink 恰好输出一条 route
- 每条 route 起点是 source，终点是对应 sink
- 每条 route 所有点在 die 内
- 每条 route 相邻点不相同，且 x 相同或 y 相同

Writer 不检查 route union 是否成树，不检查是否穿过其它 sink，不检查 buffer fanout，不做 evaluator 的替代实现。若 evaluator 报错，保留 writer/router 输出并报告 evaluator stdout/stderr。

## 7. Debug

`debug_enable(bool)` 控制 debug。

开启 debug 时打印：

```text
[Writer] output_path=result/sample<k>_solution.txt
[Writer] num_buffers=0
[Writer] num_routes=<N>
[Writer] route <sink_id> P=<P> length=<manhattan polyline length>
```

若 round 非整数 loc/router point，打印 warning。

## 8. 推荐主流程

```text
write_solution(input_path, problem, tree, loc_result, router_result):
    output_path = result/<basename>_solution.txt
    validate global inputs
    ensure result directory
    round locer node locs
    build edge map from router_result.edge_debugs
    for sink in input order:
        find sink leaf
        build source-side-to-leaf topology path
        append source edge router polyline
        append each topology edge router polyline
        validate route
    write NUM_BUFS 0
    write NUM_ROUTES and ROUTE blocks
    debug print
    return valid
```

## 9. 注意事项

- 不要修改 `evaluate.cpp`。
- 不要修改 router polyline 以迎合 evaluator。
- 不要输出额外注释行。
- 不要输出浮点坐标。
- 不要把 `P` 写成 segment 数；`P` 是 polyline point 数。
- 文件输出失败时返回 `WriterResult.valid=false` 和明确 `error_msg`。
