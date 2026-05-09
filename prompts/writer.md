请实现 Writer 模块：参考作业 PDF 的输出格式，将已有 parser / treer / bu / td 的最终结果写入 `result/sample<k>_solution.txt`。Writer 只负责格式化输出和合法性检查，不负责选择 node 位置、不负责生成 parent-child route、不负责补 detour、不负责绕障、不负责 skew balancing。所有具体布线几何必须由 TD 模块提前确定。只实现 writer 相关文件和必要的 main/build 接入；不要修改 parser、treer、bu、td 的接口和算法；不要重新实现 evaluator。

# writer: solution output generator

## 1. 目标与输入输出

Writer 的职责是把 `common::Problem`、`common::TopologyTree`、`common::BottomUpResult`、`common::TopDownResult` 中已经确定好的最终 buffer placement 和最终 route geometry，忠实转换为作业要求的 solution 文件。Writer 不得推导、改变或重新生成任何布线几何。

作业 PDF 要求输出格式为：

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

其中：

- 先列出所有 buffer。
- 再为每个 sink 输出一条从 SOURCE 到该 sink 的 rectilinear route。
- route 中每相邻两个点必须满足 `x` 相同或 `y` 相同。
- 每条 route 第一个点必须是 source 坐标，最后一个点必须是对应 sink 坐标。
- 输出文件路径为：

```text
result/sample<k>_solution.txt
```

若输入文件名为：

```text
samples/sample1.txt
```

输出应为：

```text
result/sample1_solution.txt
```

更一般地，去掉输入路径和扩展名，取 basename，再拼接：

```text
result/<basename>_solution.txt
```

核心边界：

- TD 负责根据 BU 结果确定每个 tree node 的具体位置，以及每条 parent-child branch 的最终合法 rectilinear route。
- TD 负责把 assigned edge length、detour、绕障 extra、sibling compensation/skew balancing 全部落实为具体 polyline。
- Writer 只读取 TD 的最终 route 字段并写文件。
- Writer 不允许调用任何会“选择路径”的逻辑，也不允许根据 assigned length 自行补绕线。

## 2. 文件 / namespace / API

新增：

- `writer.h`
- `writer.cc`

所有 writer 函数放在：

```cpp
namespace writer
```

`writer.h`：

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
                            const common::TopologyTree& tree,
                            const common::BottomUpResult& bu_result,
                            const common::TopDownResult& td_result);

}  // namespace writer
```

`writer.cc` 内部维护：

```cpp
static bool g_debug_enabled = false;
```

错误处理规则：

- 不用 exception 作为主错误处理。
- 失败返回 `WriterResult{false, error_msg, output_path}`。
- main 中若失败，打印：

```cpp
std::cerr << "Writer error: " << writer_result.error_msg << "\n";
return 1;
```

## 3. main.cc 接入

在 `main.cc` 中接入：

```cpp
#include "writer.h"
```

在 parser、treer、bu、td 成功之后调用：

```cpp
writer::debug_enable(true);

writer::WriterResult writer_result =
    writer::write_solution(argv[1], problem, tree, bu_result, td_result);

if (!writer_result.valid) {
    std::cerr << "Writer error: " << writer_result.error_msg << "\n";
    return 1;
}

std::cout << "Wrote solution: " << writer_result.output_path << "\n";
```

构建文件也要加入：

```text
SRC += src/writer.cc
INC += include/writer.h
```

## 4. 输出数据模型

`writer.cc` 内部定义辅助结构，不必加入 `common.h`：

```cpp
struct IntPoint {
    int x = 0;
    int y = 0;
};

struct OutputBuffer {
    std::string id;              // B0, B1, ...
    int type_index = -1;
    std::string type_name;
    IntPoint loc;
    int tree_node_id = -1;
};

struct OutputRoute {
    std::string sink_id;
    std::vector<IntPoint> points;
};
```

## 5. 坐标离散化原则

作业输入是 2D integer plane，输出 route 和 buffer 坐标也必须是整数。

TD 中 `loc` 是 double，因此 writer 需要将 tree node location 转换为 integer grid point。

实现：

```cpp
static bool is_near_integer(double v);
static int round_to_int(double v);
static IntPoint round_point(common::SegmentPoint p);
```

规则：

- 若 `abs(v - round(v)) <= 1e-6`，直接 round。
- 若不是近似整数，也允许 round 到最近整数，但 debug warning：

```text
[Writer][WARN] non-integer TD loc at node <id>, rounded from (...) to (...)
```

- round 后必须检查在 die 内：

```text
0 <= x <= problem.die_width
0 <= y <= problem.die_height
```

如果越界，返回 invalid。

注意：不要为了追求浮点几何精确而输出小数；最终文件必须全部输出整数。

## 6. buffer 输出规则

TD 已经将 BU 选择的 buffer 标记在 child node 上：

```cpp
td_result.node_results[node_id].has_buffer
td_result.node_results[node_id].buffer_type_index
```

Writer 扫描所有 tree node，收集 `has_buffer=true` 的 node。

规则：

- buffer id 按扫描顺序命名为 `B0`, `B1`, `B2`, ...。
- type 名称来自：

```cpp
problem.buffer_types[buffer_type_index].name
```

- buffer 坐标为该 tree node 的 TD placement location round 后的整数坐标。
- buffer 坐标必须唯一。
- buffer 坐标不能等于 SOURCE 坐标。
- buffer 坐标不能等于任何 SINK 坐标。
- buffer type index 必须合法。
- 对每个 buffer，检查 fanout：

```text
tree.nodes[node_id].sink_count <= problem.buffer_types[type_index].max_fanout
```

若违反，返回 invalid。

- leaf node 上如果有 buffer，也按同样规则处理；但如果 buffer 坐标与 sink 坐标冲突，会自然报错。不要擅自移动 buffer。

输出格式：

```text
NUM_BUFS <K>
BUF B0 <type_name> <x> <y>
BUF B1 <type_name> <x> <y>
...
```

## 7. route 生成策略

Writer 不生成布线，只展开 TD 已经确定的最终 branch route，组合成每个 sink 的 source-to-sink polyline。

对每个 sink leaf：

1. 从 leaf node 沿 `parent` 指针回溯到 root，得到 node path：

```text
root -> ... -> leaf
```

2. 将 source 坐标作为 route 第一个点。
3. 从 source 到 root TD loc 只做必要连接：
   - 如果 TD root loc 等于 source，直接进入下一步。
   - 如果 TD 已经提供 source-to-root 的 route 字段，则使用 TD 提供的 route。
   - 如果现有 TD 数据结构没有 source-to-root route 字段，则允许用一个最简单的 rectilinear connector 连接 source 和 root loc；该 connector 只是为了满足输出格式起点要求，不参与 BU/TD branch length 或 skew 语义。
4. 对 node path 中的每条 parent -> child tree edge，直接追加 `td_result.node_results[child].final_route_to_parent`。这是 TD 新接口提供的最终 branch polyline，已经包含 base route、绕障 extra 和 compensation；Writer 不得读取或拼接 `route_to_parent` / `compensation_snake_to_parent`。
5. 从 leaf TD loc 到 sink 坐标：
   - 如果 TD leaf loc 等于 sink，直接结束。
   - 如果 TD 已经提供 leaf-to-sink 的 route 字段，则使用 TD 提供的 route。
   - 如果现有 TD 数据结构没有 leaf-to-sink route 字段，则允许用一个最简单的 rectilinear connector 连接 leaf loc 和 sink 坐标；该 connector 只是为了保证 route 终点是 sink，不得用于补偿 delay 或 detour。
6. `append_point` 已保证不产生连续重复点。

route 输出顺序必须与 `problem.sinks` 的输入顺序一致，即：

```text
for sink_index in [0, problem.sinks.size())
```

需要找到对应 `tree.nodes[node_id].is_leaf && tree.nodes[node_id].sink_index == sink_index` 的 leaf node。

若找不到，返回 invalid。

重要：Writer 不得根据 parent loc 和 child loc 自行选择 horizontal-first / vertical-first / DAG path / detour path。parent-child branch 的几何结果必须来自 TD。

## 8. rectilinear 连接 helper

实现：

```cpp
static void append_point(std::vector<IntPoint>& pts, IntPoint p);
static bool append_td_branch_polyline(std::vector<IntPoint>& pts,
                                      const std::vector<common::SegmentPoint>& final_route_to_parent,
                                      IntPoint expected_from,
                                      IntPoint expected_to,
                                      std::string& err);
static void append_minimal_connector(std::vector<IntPoint>& pts,
                                     IntPoint from,
                                     IntPoint to);
```

`append_point()`：

- 若 `pts` 为空，push。
- 若新点与最后一个点相同，不 push。
- 否则 push。

`append_td_branch_polyline()`：

- 用于追加 TD 新接口提供的 `final_route_to_parent`。
- 输入类型是 `std::vector<common::SegmentPoint>`，Writer 只负责 round 成 `IntPoint` 后追加。
- `final_route_to_parent` 不能为空。
- round 后的第一个点必须等于 `expected_from`，即 rounded parent TD loc。
- round 后的最后一个点必须等于 `expected_to`，即 rounded child TD loc。
- 当前 `pts.back()` 必须等于 `expected_from`。
- 每相邻两个点必须是水平/垂直段。
- 只做顺序追加，不改变 polyline 的走向和中间点。
- 如果发现端点不匹配、空 route、非 rectilinear 段或越界，返回 invalid。

`append_minimal_connector()`：

- 只允许用于 source -> root 或 leaf -> sink 这类输出格式连接。
- 不允许用于 parent -> child tree edge。
- 如果 `from == to`，只追加 `to`。
- 如果 `from.x == to.x || from.y == to.y`，直接追加 `to`。
- 否则追加一个固定的 L-shape connector：`(to.x, from.y)` 再到 `to`。
- 这个 helper 只是输出适配，不承担布线优化、detour、绕障或 skew balancing 语义。

## 9. route branch 使用规则

Writer 不使用 `assigned_edge_to_parent` 生成几何路径。`assigned_edge_to_parent`、`detour_to_parent`、绕障 extra、compensation snake 等语义必须已经由 TD 体现在 `final_route_to_parent` 中。

### 9.1 source -> root

作业要求每条 route 从 source 坐标开始，但 TD root loc 不一定等于 source 坐标。

对于 `source -> root_loc`：

- 如果 TD 提供 source-to-root route，直接使用 TD route。
- 如果 TD 没有提供该字段，使用 `append_minimal_connector()`。
- 不使用 BU/TD assigned edge。
- 不插 detour。

### 9.2 internal parent -> child

对于 tree edge `parent -> child`：

- from = rounded TD parent loc
- to = rounded TD child loc
- branch route 必须直接来自：

```cpp
td_result.node_results[child].final_route_to_parent
```

- `final_route_to_parent` 必须以 parent loc 开始，以 child loc 结束。
- `final_route_to_parent` 已经包含 TD 的 base obstacle-aware route、绕障 extra 和 sibling compensation。
- Writer 只允许调用 `append_td_branch_polyline()` 追加它，不得调用 `append_minimal_connector()` 或任何 detour helper。
- Writer 不得读取、拼接或解释：

```cpp
td_result.node_results[child].route_to_parent
td_result.node_results[child].compensation_snake_to_parent
```

- 如果 `final_route_to_parent` 为空或端点不匹配，返回 invalid，错误信息说明：`TD result missing/invalid final_route_to_parent for node <child>`。

### 9.3 leaf -> sink

leaf TD loc 理论上应等于或靠近 sink 坐标。为了保证 route 最后到达 sink：

- 如果 TD 提供 leaf-to-sink route，直接使用 TD route。
- 如果 TD 没有提供该字段，使用 `append_minimal_connector()`。
- 不使用 assigned edge。
- 不插 detour。

## 10. 输出文件生成

实现：

```cpp
static std::string basename_without_ext(const std::string& input_path);
static std::string make_output_path(const std::string& input_path);
static bool ensure_result_dir(std::string& err);
```

`make_output_path()`：

```text
base = basename_without_ext(input_path)
return "result/" + base + "_solution.txt"
```

`ensure_result_dir()`：

- 使用 C++17 `<filesystem>`：

```cpp
std::filesystem::create_directories("result");
```

- 若失败，返回 invalid。

写文件时使用 `std::ofstream`。

文件内容严格如下：

```text
NUM_BUFS <K>
BUF <buf_id> <type> <x> <y>
...
NUM_ROUTES <N>
ROUTE <sink_id> <P>
<x0> <y0>
<x1> <y1>
...
```

建议在每个 route 之间不要额外空行，避免 evaluator 解析问题。

每行末尾加 `\n`。

## 11. 合法性检查

`write_solution()` 开始至少检查：

- `problem.valid == true`
- `tree.valid == true`
- `bu_result.valid == true`
- `td_result.valid == true`
- `tree.root` 合法
- `tree.nodes` 非空
- `td_result.node_results.size() == tree.nodes.size()`
- `bu_result.node_results.size() == tree.nodes.size()`
- `problem.sinks` 非空
- 每个 sink 坐标在 die 内
- source 坐标在 die 内

对每个 TD node 检查：

- 如果 tree node 会被输出路径使用，则 `td_result.node_results[node_id].valid == true`
- rounded loc 在 die 内
- `min_delay <= max_delay + EPS`
- `skew >= -EPS`

对 route 检查：

- 非空且 `points.size() >= 2`
- 第一项等于 source 坐标
- 最后一项等于对应 sink 坐标
- 所有点在 die 内
- 任意相邻点不相同
- 任意相邻点满足 same x 或 same y
- 每条 parent-child branch route 必须来自 `td_result.node_results[child].final_route_to_parent`。
- 每条 `final_route_to_parent` 的起点必须等于 parent TD loc，终点必须等于 child TD loc。
- 每条 `final_route_to_parent` 必须是合法 rectilinear polyline。
- Writer 不检查 branch route 是否为最优路径，也不解释 compensation 语义；这些已经由 TD 完成。

## 12. debug 输出

`debug_enable(bool)` 控制 debug。

开启 debug 时，在 writer 中打印：

```text
[Writer] output_path=result/sample<k>_solution.txt
[Writer] num_buffers=<K>
[Writer] buffer B0 type=<type> node=<node_id> loc=(x,y) fanout=<sink_count>
[Writer] num_routes=<N>
[Writer] route <sink_id> P=<P> length=<manhattan polyline length>
```

实现 helper：

```cpp
static int polyline_length(const std::vector<IntPoint>& pts);
```

## 13. 推荐 write_solution() 主流程

```text
write_solution(input_path, problem, tree, bu_result, td_result):
    result.output_path = make_output_path(input_path)

    validate global inputs
    ensure result directory

    build rounded_loc[node_id]
    collect buffers from td_result
    validate buffers

    for each sink_index in input order:
        find leaf node with matching sink_index
        build node path root -> leaf
        pts = []
        append source
        append source -> root_loc using TD source route if available, otherwise minimal connector
        for every adjacent node pair parent -> child in node path:
            read td_result.node_results[child].final_route_to_parent
            append_td_branch_polyline(final_route_to_parent, parent_loc, child_loc)
        append leaf_loc -> sink_loc using TD leaf route if available, otherwise minimal connector
        validate route
        save OutputRoute

    open output file
    write NUM_BUFS and BUF lines
    write NUM_ROUTES and ROUTE blocks
    close file

    debug print
    return valid
```

## 14. 注意事项

- Writer 只负责输出，不要改变 BU/TD 的结果，也不要重新布线。
- 不要修改 `common.h`，除非现有数据结构完全缺失；目前 `TopDownResult` 和 `BottomUpResult` 已经在 common 中，应直接使用。
- 不要把 buffer 插到 edge 中间；buffer 位置就是 TD 标记的 child node 位置。
- 不要输出额外注释行。
- 不要输出浮点坐标。
- 不要把 route 的 `P` 写成 segment 数量；`P` 是 polyline point 数量。
- 输出 route 顺序必须跟 input sink 顺序一致，方便 debug 和 evaluator 对照。
- 如果 TD 没有提供 `final_route_to_parent`，Writer 应返回 invalid，而不是自行构造 route，也不是拼接 `route_to_parent + compensation_snake_to_parent`。
<!--
Ensure all mentions of `route_to_parent` and `compensation_snake_to_parent` say Writer must not consume them, except in forbidden-field lists. Remove any fallback logic allowing Writer to concatenate those fields.
-->