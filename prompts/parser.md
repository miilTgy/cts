

请只实现 `parser.h` 和 `parser.cc`；如需对外暴露输入数据结构和 API，可适当新建/修改 `common.h`。不要实现 CTS 算法、输出文件生成器或 main 逻辑。目标是在 `main.cc` 中只需要调用一次：

```cpp
parser::Problem problem = parser::parse(in_path);
```

即可读入 HW3 输入文件，并把 DIE、SOURCE、SINK、BUFFER_TYPE 信息存入结构体。

## 输入格式

严格支持 PDF 中的 HW3 输入格式：

```text
DIE <width> <height>
SOURCE <id> <x> <y>
NUM_SINKS <N>
SINK <id> <x> <y>
...
NUM_BUFFERS <B>
BUFFER_TYPE <name> <delay> <max_fanout> <cost>
...
```

需要能正确读取 `samples/sample1.txt` 这种文件，其中允许空行，例如：

```text
DIE 130 120

SOURCE SRC 64 60

NUM_SINKS 15
SINK L0 20 94
...
NUM_BUFFERS 3
BUFFER_TYPE BUF_SMALL 2 3 3
BUFFER_TYPE BUF_MED 4 5 6
BUFFER_TYPE BUF_BIG 7 7 10
```

## 文件与 namespace 要求

- 所有 parser 相关内容放在 `namespace parser` 中。
- `parser.h`：声明对外 API。
- `parser.cc`：实现具体解析逻辑。
- `common.h`：放需要被其他模块使用的数据结构。
- 不要使用 `try / catch / exception` 作为主要错误处理方式，尽量使用 `if-else` 检查并返回错误状态/打印错误。
- 可使用 C++ 标准库的 `std::ifstream`、`std::getline`、`std::istringstream`、`std::vector`、`std::string`、`std::cerr` 等。

## 数据结构建议放在 `common.h`

```cpp
#pragma once

#include <string>
#include <vector>

namespace common {

struct Point {
    int x = 0;
    int y = 0;
};

struct Sink {
    std::string id;
    Point loc;
};

struct BufferType {
    std::string name;
    int delay = 0;
    int max_fanout = 0;
    int cost = 0;
};

struct Problem {
    int die_width = 0;
    int die_height = 0;
    Sink source;
    std::vector<Sink> sinks;
    std::vector<BufferType> buffer_types;
    bool valid = false;
    std::string error_msg;
};

}  // namespace common
```

## parser.h API 要求

```cpp
#pragma once

#include <string>
#include "common.h"

namespace parser {

void debug_enable(bool enable);
void debug_output(const Problem& problem);
Problem parse(const std::string& in_path);

}  // namespace parser
```

## parser.cc 实现要求

实现以下逻辑：

1. 内部维护一个 `static bool g_debug_enabled = false;`。
2. `debug_enable(bool enable)` 用于设置 debug 开关。
3. `debug_output(const Problem& problem)`：
   - 当 debug 关闭时直接 return。
   - 当 debug 开启时，打印：die size、source id/坐标、sink 数量和列表、buffer type 数量和列表、valid/error_msg。
4. `parse(const std::string& in_path)`：
   - 打开输入文件；如果失败，返回 `Problem{.valid=false}` 并写入 `error_msg`。
   - 逐 token 读取，自动跳过空白和空行。
   - 按顺序解析：`DIE`、`SOURCE`、`NUM_SINKS`、N 行 `SINK`、`NUM_BUFFERS`、B 行 `BUFFER_TYPE`。
   - 每一步都检查 keyword 是否正确；若不正确，设置 `valid=false` 和清晰的 `error_msg` 后返回。
   - 检查数量非负；坐标建议检查在 die 范围内：`0 <= x <= die_width`，`0 <= y <= die_height`。如果你认为边界应为 `< width/height`，请在注释中说明；本 HW 图示和样例更适合使用闭区间边界。
   - 检查 source/sink 坐标合法。
   - 检查 buffer delay、max_fanout、cost 非负，且 max_fanout 最好大于 0。
   - 成功读完后设置 `problem.valid = true`。
   - 若 debug 开启，parse 结束前调用 `debug_output(problem)`。

## 解析风格要求

使用类似下面的模式，不要写复杂 parser framework：

```cpp
std::ifstream fin(in_path);
if (!fin) {
    problem.valid = false;
    problem.error_msg = "Cannot open input file: " + in_path;
    return problem;
}

std::string keyword;
fin >> keyword;
if (keyword != "DIE") {
    problem.error_msg = "Expected DIE, got " + keyword;
    return problem;
}
fin >> problem.die_width >> problem.die_height;
```

对 `SOURCE`、`SINK`、`BUFFER_TYPE` 建议写小的 helper function，例如：

```cpp
static bool read_sink(std::istream& in, const std::string& expected_keyword, Sink& sink, std::string& err);
static bool read_buffer_type(std::istream& in, BufferType& buf, std::string& err);
static bool check_point_in_die(const Problem& problem, const Point& p, std::string& err);
```

但 helper 函数只放在 `parser.cc` 的匿名 namespace 或 `static` 函数中，不暴露到外部。

## main.cc 预期调用方式

最终其他模块应能这样使用：

```cpp
#include "parser.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        return 1;
    }

    parser::debug_enable(true);
    parser::Problem problem = parser::parse(argv[1]);

    if (!problem.valid) {
        return 1;
    }

    // 后续 CTS 算法直接使用 problem.source、problem.sinks、problem.buffer_types
    return 0;
}
```

请保持实现简洁、稳健、易读，不要过度设计。