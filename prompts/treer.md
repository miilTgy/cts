请只实现 topology generation 模块，不要实现 DME bottom-up、DME top-down、buffer insertion、router、evaluator 或输出文件生成器。目标是根据 `common::Problem` 中的 source/sinks 生成一棵供 Buffered-DME 使用的抽象二叉拓扑树。

建议文件名为 `treer.h` 和 `treer.cc`；如需对外暴露树节点和树结构，可适当放在 `common.h` 中。最终在 `main.cc` 或后续 CTS flow 中只需要调用一次：

```cpp
treer::TopologyTree tree = treer::build(problem, argv[1]);
```

即可得到一棵二叉拓扑树。debug 开启时，还需要在 `./tree/` 目录下额外输出一个以输入文件名命名的树结构记录文件，方便后续写脚本可视化。

# Topology Generation: Greedy RGM (Recursive Geometric Matching)

## 算法目标

输入 source 和多个 sinks，输出一棵 abstract binary topology。该模块只决定 sink 之间的二叉合并关系，不决定最终内部节点坐标，不插 buffer，不输出 routing path。

DME 后续会在给定 topology 上做 bottom-up merging segment 和 top-down embedding。因此 `treer` 的职责是让拓扑尽量满足：

- 几何上相近的 sinks / clusters 优先合并；
- 左右子树 sink 数量尽量平衡；
- 左右子树估计 delay 尽量接近；
- 输出结构严格为二叉树，每个 internal node 恰好有两个 child。

## 文件与 namespace 要求

- 所有 topology generation 相关内容放在 `namespace treer` 中。
- `treer.h`：声明对外 API。
- `treer.cc`：实现 Greedy RGM 具体逻辑。
- `common.h`：放需要被其他模块使用的树节点/拓扑树数据结构。
- 不要使用 `try / catch / exception` 作为主要错误处理方式，尽量使用 `if-else` 检查并返回 `valid=false` 和 `error_msg`。
- 可使用 C++ 标准库的 `std::vector`、`std::string`、`std::limits`、`std::cmath`、`std::cerr` 等。

## 数据结构建议放在 `common.h`

如果 `common.h` 已经有 `common::Problem`、`common::Sink`、`common::Point`，请在同一个 `namespace common` 中补充：

```cpp
namespace common {

struct TreeNode {
    int id = -1;

    bool is_leaf = false;
    int sink_index = -1;   // valid only when is_leaf == true

    int parent = -1;
    int left = -1;
    int right = -1;

    int sink_count = 0;

    // Abstract cluster center used only for topology generation.
    // DME top-down will decide the real embedding coordinate later.
    double cx = 0.0;
    double cy = 0.0;

    int bbox_lx = 0;
    int bbox_ly = 0;
    int bbox_ux = 0;
    int bbox_uy = 0;

    // Estimated maximum downstream delay from this cluster root to its sinks.
    // This is only a topology-generation heuristic, not final evaluator delay.
    double est_delay = 0.0;
};

struct TopologyTree {
    std::vector<TreeNode> nodes;
    int root = -1;
    bool valid = false;
    std::string error_msg;
};

}  // namespace common
```

`treer.h` 中可以通过 type alias 暴露：

```cpp
namespace treer {
using TopologyTree = common::TopologyTree;
using TreeNode = common::TreeNode;
}
```

## treer.h API 要求

```cpp
#pragma once

#include "common.h"
#include <string>

namespace treer {

using TopologyTree = common::TopologyTree;
using TreeNode = common::TreeNode;

void debug_enable(bool enable);
void debug_output(const TopologyTree& tree, const common::Problem& problem);
void debug_output_file(const TopologyTree& tree,
                       const common::Problem& problem,
                       const std::string& input_path);
TopologyTree build(const common::Problem& problem, const std::string& input_path = "");

}  // namespace treer
```

## treer.cc 实现要求

实现以下逻辑：

1. 内部维护一个 `static bool g_debug_enabled = false;`。
2. `debug_enable(bool enable)` 用于设置 debug 开关。
3. `debug_output(const TopologyTree& tree, const common::Problem& problem)`：
   - debug 关闭时直接 return。
   - debug 开启时打印：valid/error_msg、root id、node 数量、每个 node 的 id/parent/left/right/is_leaf/sink_index/sink_count/center/bbox/est_delay。
   - leaf node 额外打印对应 sink id 和坐标。
4. `debug_output_file(const TopologyTree& tree, const common::Problem& problem, const std::string& input_path)`：
   - debug 关闭时直接 return。
   - `input_path` 为空时使用默认文件名 `tree_debug.txt`。
   - 根据 `input_path` 提取 basename。例如输入 `samples/sample1.txt`，输出文件为 `./tree/sample1.txt`。
   - 创建 `./tree/` 目录；如果工程使用 C++17，可用 `std::filesystem::create_directories("tree")`；如果不想依赖 C++17，可用 `system("mkdir -p tree")`，但优先使用 `std::filesystem`。
   - 用 `std::ofstream` 写出树结构；如果无法打开文件，向 `std::cerr` 打印 warning，但不要让 `build()` 失败。
5. `build(const common::Problem& problem, const std::string& input_path = "")`：
   - 如果 `problem.valid == false`，返回 `TopologyTree{.valid=false}`，并写入 error_msg。
   - 如果 sink 数量为 0，返回 invalid。
   - 如果 sink 数量为 1，生成只有一个 leaf node 的 tree，root 指向该 leaf，valid=true。
   - 如果 sink 数量大于 1，使用 Greedy RGM 合并生成二叉树。
   - build 成功后，如果 debug 开启，调用 `debug_output(tree, problem)` 和 `debug_output_file(tree, problem, input_path)`。

## Greedy RGM 主流程

```text
Input:
    common::Problem problem

Output:
    common::TopologyTree tree

Initialize:
    for each sink i:
        create one leaf TreeNode
        node.id = nodes.size()
        node.is_leaf = true
        node.sink_index = i
        node.parent = -1
        node.left = -1
        node.right = -1
        node.sink_count = 1
        node.cx = sink[i].loc.x
        node.cy = sink[i].loc.y
        node.bbox = sink point itself
        node.est_delay = 0
        push node into tree.nodes
        push node.id into active_clusters

while active_clusters.size() > 1:
    best_cost = INF
    best_i = -1
    best_j = -1

    enumerate all unordered pairs active_clusters[i], active_clusters[j]:
        ci = active_clusters[i]
        cj = active_clusters[j]
        cost = pair_cost(tree.nodes[ci], tree.nodes[cj], problem.source.loc)
        if cost < best_cost:
            best_cost = cost
            best_i = i
            best_j = j

    left_id = active_clusters[best_i]
    right_id = active_clusters[best_j]

    create parent node p
    p.id = tree.nodes.size()
    p.is_leaf = false
    p.sink_index = -1
    p.left = left_id
    p.right = right_id
    p.parent = -1
    p.sink_count = nodes[left_id].sink_count + nodes[right_id].sink_count
    p.cx = weighted average center of left/right by sink_count
    p.cy = weighted average center of left/right by sink_count
    p.bbox = union bbox of left/right

    D = manhattan(center(left), center(right))
    dl = nodes[left_id].est_delay
    dr = nodes[right_id].est_delay

    if abs(dl - dr) <= D:
        p.est_delay = (dl + dr + D) / 2.0
    else:
        p.est_delay = max(dl, dr)

    set nodes[left_id].parent = p.id
    set nodes[right_id].parent = p.id
    push p into tree.nodes

    remove the two selected clusters from active_clusters
    insert p.id into active_clusters

After loop:
    tree.root = active_clusters[0]
    tree.valid = true
    return tree
```

删除 active clusters 时注意先删 index 较大的，再删 index 较小的，避免 vector erase 后下标变化。

## 奇数 sink / cluster 数量处理

当前算法是 Greedy RGM-style pairwise clustering，而不是严格的 batch matching RGM。它每次只选择一对 active clusters 合并，因此 active cluster 数量每轮减少 1。

所以该算法天然支持奇数 sink 数量，例如：

```text
15 -> 14 -> 13 -> ... -> 2 -> 1
```

不需要额外写 unmatched cluster carry-over 逻辑。只要 `while active_clusters.size() > 1` 持续执行，最后一定会得到唯一 root。

## pair_cost 设计

第一版使用简单、可调、稳定的启发式：

```text
pair_cost(ci, cj) =
      alpha * manhattan(center(ci), center(cj))
    + beta  * abs(sink_count(ci) - sink_count(cj))
    + gamma * abs(est_delay(ci) - est_delay(cj))
    + delta * source_bias(ci, cj)
```

推荐默认参数：

```text
alpha = 1.0
beta  = 5.0
gamma = 1.0
delta = 0.05
```

其中：

```text
manhattan(center(ci), center(cj)) = abs(ci.cx - cj.cx) + abs(ci.cy - cj.cy)

source_bias(ci, cj) = abs(manhattan(source, center(ci))
                       - manhattan(source, center(cj)))
```

解释：

- `alpha`：鼓励几何距离近的 clusters 优先合并，减少未来 wirelength。
- `beta`：鼓励左右子树 sink 数量平衡，避免退化成长链。
- `gamma`：鼓励左右子树估计 delay 接近，降低后续 DME 调 skew 的压力。
- `delta`：轻微鼓励两个 cluster 到 source 的距离相近，避免 root 附近拓扑过度偏斜。该项权重应较小，不要盖过局部几何匹配。

## est_delay 定义

`est_delay[node]` 表示从该 cluster 的抽象入口点到其所有下游 sinks 的估计最大路径 delay。它只用于 topology generation 阶段的 pair selection，不是最终 evaluator delay，也不是 DME bottom-up 的精确 delay。

leaf node：

```text
est_delay[leaf] = 0
```

internal node 合并时使用 DME-like 的估计。设：

```text
D  = manhattan(center(left), center(right))
dl = est_delay[left]
dr = est_delay[right]
```

如果两边 delay 差可以通过几何距离调平：

```text
abs(dl - dr) <= D
```

则估计合并后的最大 delay 为：

```text
est_delay[parent] = (dl + dr + D) / 2.0
```

否则说明几何距离不足以补偿两边已有 delay 差，保守估计为：

```text
est_delay[parent] = max(dl, dr)
```

等价写法：

```cpp
double d = manhattan(left.cx, left.cy, right.cx, right.cy);
double dl = left.est_delay;
double dr = right.est_delay;
parent.est_delay = std::max(dl, dr) + std::max(0.0, (d - std::abs(dl - dr)) / 2.0);
```

## 合法性与一致性检查

在 `build()` 结束前可以做轻量检查：

- `tree.root >= 0`。
- `tree.nodes[tree.root].parent == -1`。
- leaf 数量等于 `problem.sinks.size()`。
- 每个 non-root node 的 parent 有效。
- 每个 internal node 的 left/right 都有效且不同。
- 每个 internal node 的 `sink_count = left.sink_count + right.sink_count`。

如果检查失败，设置 `tree.valid=false` 和清晰的 `error_msg`。

## Debug tree file 输出格式

当 `treer::debug_enable(true)` 时，`build(problem, input_path)` 除了向 stdout/stderr 打印 debug 信息，还需要在 `./tree/` 下生成一个树结构文件：

```text
./tree/<input basename>.txt
```

例如：

```text
input_path = samples/sample1.txt
output     = ./tree/sample1.txt
```

如果 `input_path` 为空，则输出：

```text
./tree/tree_debug.txt
```

建议输出纯文本、脚本友好的格式：

```text
TREE_VALID 1
ROOT <root_id>
NUM_NODES <num_nodes>
NUM_SINKS <num_sinks>

NODE <id> <parent> <left> <right> <is_leaf> <sink_index> <sink_count> <cx> <cy> <bbox_lx> <bbox_ly> <bbox_ux> <bbox_uy> <est_delay>
...

LEAF <node_id> <sink_index> <sink_id> <x> <y>
...

EDGE <parent_id> <child_id>
...
```

其中：

- `NODE` 行记录完整树节点信息。
- `LEAF` 行方便可视化脚本直接找到 sink 坐标。
- `EDGE` 行方便脚本直接画 parent-child 连线。
- internal node 的真实物理坐标尚未由 DME top-down 决定，所以可视化时可以先用 `cx/cy` 作为抽象位置。

`debug_output_file()` 可以使用如下 helper：

```cpp
static std::string get_basename(const std::string& input_path);
static bool ensure_tree_dir();
static bool write_tree_file(const common::TopologyTree& tree,
                            const common::Problem& problem,
                            const std::string& output_path);
```

如果使用 C++17，建议包含：

```cpp
#include <filesystem>
#include <fstream>
```

并使用：

```cpp
std::filesystem::create_directories("tree");
std::ofstream fout(output_path);
```

## main.cc / flow 预期调用方式

最终其他模块应能这样使用：

```cpp
#include "parser.h"
#include "treer.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        return 1;
    }

    parser::debug_enable(true);
    treer::debug_enable(true);

    common::Problem problem = parser::parse(argv[1]);
    if (!problem.valid) {
        return 1;
    }

    treer::TopologyTree tree = treer::build(problem, argv[1]);
    if (!tree.valid) {
        return 1;
    }

    // 后续 Buffered-DME 直接使用 tree.root 和 tree.nodes
    return 0;
}
```