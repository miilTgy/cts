请只实现 topology generation 模块，不要实现 DME bottom-up、DME top-down、buffer insertion、router、evaluator 或输出文件生成器。目标是根据 `common::Problem` 中的 source/sinks 生成一棵供 Buffered-DME 使用的抽象二叉拓扑树。

建议文件名为 `treer.h` 和 `treer.cc`；如需对外暴露树节点和树结构，可适当放在 `common.h` 中。最终在 `main.cc` 或后续 CTS flow 中只需要调用一次：

```cpp
treer::TopologyTree tree = treer::build(problem, argv[1]);
```

即可得到一棵二叉拓扑树。debug 开启时，还需要在 `./tree/` 目录下额外输出一个以输入文件名命名的树结构记录文件，方便后续写脚本可视化。

# Topology Generation: Top-down MMM-style Recursive Geometric Bisection

## 算法目标

DME 后续会在给定 topology 上做 bottom-up merging segment、buffer insertion 和 top-down embedding。因此 `treer` 的职责不是在 topology 阶段追求 skew=0，而是优先生成 single-metal-friendly、planar-like、稳定可布线的二叉 slicing topology。

`treer` 使用类似 MMM（Method of Means and Medians）的 top-down recursive geometric bisection：

- 对当前 sink set 计算 center of gravity，也就是 COG / mean，作为当前 abstract cluster center。
- 根据当前 sink set 的 bbox 选择切分轴。
- 按该轴排序，并用 median 附近的连续切分把 sink set 分成两个大小接近、几何上相邻的子集。
- 对两个子集递归构造 left / right subtree。
- 输出结构严格为二叉树，每个 internal node 恰好有两个 child。

核心优先级：

```text
合法可布线性 / planar-like topology > sink_count balance > wirelength heuristic > source/delay tie-break
```

不要为了 topology 阶段的估计 skew 破坏几何连续划分；后续 BU/TD/buffer 会负责 skew 修正。

重要限制：普通 MMM 只保证 sink set 是连续几何划分，但如果 internal node 直接使用 COG，并用直线连接 parent COG 到 child COG，仍然可能在几何可视化上出现交叉。因此本 treer 采用 slicing-region-aware MMM：递归时同时维护每个 subtree 的 owned region，并把 internal node 的 abstract center 放在当前 split separator 附近，而不是无条件放在 sink set COG。这样生成的 abstract topology 更接近 H-tree / slicing tree，后续 TD 更容易布成无交叉 single-metal CTS。

## 文件与 namespace 要求

- 所有 topology generation 相关内容放在 `namespace treer` 中。
- `treer.h`：声明对外 API。
- `treer.cc`：实现 top-down MMM-style recursive geometric bisection 具体逻辑。
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

    // Optional slicing region owned by this subtree. Used by treer debug / heuristic only.
    // It is not the final physical routing region and may be ignored by BU/TD.
    int region_lx = 0;
    int region_ly = 0;
    int region_ux = 0;
    int region_uy = 0;

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
   - 如果 sink 数量大于 1，使用 top-down MMM-style recursive geometric bisection 构造二叉树。
   - build 成功后，如果 debug 开启，调用 `debug_output(tree, problem)` 和 `debug_output_file(tree, problem, input_path)`。

## Top-down MMM 主流程

该算法是 top-down partitioning，不是 bottom-up clustering。

整体流程：

```text
Input:
    common::Problem problem

Output:
    common::TopologyTree tree

build(problem):
    create indices = [0, 1, ..., num_sinks-1]
    root_region = whole die rectangle from problem
    root_id = build_subtree(indices, root_region, problem, tree)
    tree.root = root_id
    tree.valid = true
    return tree
```

递归函数：

```text
build_subtree(indices, region, problem, tree):
    if indices.empty():
        return invalid

    if indices.size() == 1:
        return create_leaf(indices[0], region, problem, tree)

    bbox = compute_bbox(indices)
    cog  = compute_cog(indices)

    axis = choose_split_axis(region, bbox, indices, problem)
    sorted = sort indices by axis, then by the other axis, then by sink_index

    k = choose_best_median_split(sorted, axis, problem)

    left_indices  = sorted[0:k]
    right_indices = sorted[k:n]

    split_coord = choose_split_coordinate(sorted, k, axis)
    left_region, right_region = split_region(region, axis, split_coord)

    left_id  = build_subtree(left_indices, left_region, problem, tree)
    right_id = build_subtree(right_indices, right_region, problem, tree)

    abstract_center = choose_separator_center(region, axis, split_coord, bbox, cog)
    parent_id = create_internal(left_id, right_id, bbox, region, abstract_center, tree)
    tree.nodes[left_id].parent = parent_id
    tree.nodes[right_id].parent = parent_id

    return parent_id
```

注意：

- `left_indices` 和 `right_indices` 必须是排序后数组的连续前缀 / 后缀。
- 不允许用交错集合，例如 `{第1, 第3, 第5}` vs `{第2, 第4, 第6}`。
- 这样生成的 topology 是 slicing-like 的，更适合 single-metal routing。
- internal node 的 `cx/cy` 不是简单 COG，而是 slicing separator-aware abstract center，仅用于 topology debug 和 heuristic；DME TD 后续会决定真实 embedding coordinate。
- 每个 subtree 维护一个 `region`。left/right region 由当前 split separator 切开，理论上互不重叠。debug 可视化时优先画 region-aware center，有助于观察 topology 是否接近无交叉 slicing tree。

## leaf / internal node 创建规则

### create_leaf

```text
create_leaf(sink_index, region):
    node.id = tree.nodes.size()
    node.is_leaf = true
    node.sink_index = sink_index
    node.parent = -1
    node.left = -1
    node.right = -1
    node.sink_count = 1
    node.cx = sink[sink_index].loc.x
    node.cy = sink[sink_index].loc.y
    node.bbox = sink point itself
    node.region = input region
    node.est_delay = 0
    push node into tree.nodes
    return node.id
```

### create_internal

```text
create_internal(left_id, right_id, bbox, region, abstract_center):
    p.id = tree.nodes.size()
    p.is_leaf = false
    p.sink_index = -1
    p.parent = -1
    p.left = left_id
    p.right = right_id
    p.sink_count = nodes[left_id].sink_count + nodes[right_id].sink_count
    p.cx = abstract_center.x
    p.cy = abstract_center.y
    p.bbox = bbox
    p.region = region

    D  = manhattan(center(left), center(right))
    dl = nodes[left_id].est_delay
    dr = nodes[right_id].est_delay
    p.est_delay = max(dl, dr) + max(0, (D - abs(dl - dr)) / 2)

    push p into tree.nodes
    return p.id
```

## slicing region 与 abstract center 规则

普通 MMM 会把 internal node 放在当前 sink set 的 COG。但是 COG 可能落在某个子区域内部，导致 parent-to-child 直线在 debug grid 上穿过其他 subtree，甚至形成交叉。因此这里使用 separator-aware abstract center。

### region 定义

每次递归调用拥有一个 axis-aligned rectangular region：

```text
Region = [lx, ly, ux, uy]
```

root region 是 die rectangle。

当当前节点按 X 切分时：

```text
left_region  = [region.lx, region.ly, split_coord, region.uy]
right_region = [split_coord, region.ly, region.ux, region.uy]
```

当当前节点按 Y 切分时：

```text
left_region  = [region.lx, region.ly, region.ux, split_coord]
right_region = [region.lx, split_coord, region.ux, region.uy]
```

这里的 left/right 只是排序后的前缀/后缀，不一定代表物理“左”和“右”；但 region 必须按切分轴连续划分。

### split_coord

对 sorted list 的切分位置 k：

```text
last_left  = sorted[k-1]
first_right = sorted[k]
```

若 axis == X：

```text
split_coord = floor((x[last_left] + x[first_right]) / 2)
```

若 axis == Y：

```text
split_coord = floor((y[last_left] + y[first_right]) / 2)
```

然后 clamp 到当前 region 内部：

```text
axis == X: split_coord in [region.lx, region.ux]
axis == Y: split_coord in [region.ly, region.uy]
```

如果两个坐标相同，允许 split_coord 等于该坐标；region 可能零宽/零高，但递归仍通过 sink_count 下降终止。

### choose_separator_center

当前 internal node 的 abstract center 不直接使用 COG，而是放在 split separator 上：

若 axis == X：

```text
cx = split_coord
cy = clamp(cog.y, region.ly, region.uy)
```

若 axis == Y：

```text
cx = clamp(cog.x, region.lx, region.ux)
cy = split_coord
```

这样 parent node 位于左右/上下子区域之间的 separator 附近，debug 连接线更像 slicing tree / H-tree，而不是任意 COG-to-COG 斜穿。

### child center 建议

leaf 的 `cx/cy` 仍然是 sink 坐标。

internal child 的 `cx/cy` 由它自己的 split separator 决定。

这并不能数学保证最终 TD 一定无交叉，因为最终 physical loc 仍由 BU/TD 决定；但它比普通 COG-MMM 更适合 single-metal-friendly topology。

## 切分轴选择

对当前 `indices` 计算 bbox，同时有当前递归的 owned region：

```text
width  = bbox_ux - bbox_lx
height = bbox_uy - bbox_ly
```

region_width  = region.ux - region.lx  
region_height = region.uy - region.ly

默认：

```text
if region_width >= region_height:
    axis = X
else:
    axis = Y
```

含义：

- 区域更宽时，按 x 排序并左右切分。
- 区域更高时，按 y 排序并上下切分。

优先用 owned region 的长宽选择切分轴，而不是只用 sink bbox。这样递归区域会更像规范 slicing floorplan，减少 COG 连线交叉。若 region 退化为零宽/零高，再 fallback 到 sink bbox 的 width/height。

如果 `width == 0 && height == 0`，说明所有点坐标完全重合，仍然可以按 `sink_index` 稳定切分。

排序规则必须 deterministic：

```text
axis == X: sort by (x, y, sink_index)
axis == Y: sort by (y, x, sink_index)
```

## median 附近 split 选择

为了保证树高度稳定和左右 sink 数量均衡，不枚举所有 `k=1..n-1` 作为强 wirelength 优化，而是只枚举 median 附近的小窗口。

推荐：

```text
n = sorted.size()
mid = n / 2
candidate_k = all k in [mid - 2, mid + 2] clipped to [1, n-1]
```

如果 n 很小，该规则自动退化为所有合法 k。

对每个候选 k：

```text
left  = sorted[0:k]
right = sorted[k:n]
score = split_cost(left, right, axis, problem)
```

选择 score 最小的 k。若 score 相同，选择更接近 `mid` 的 k；若仍相同，选择更小的 k，保证 deterministic。

## split_cost 设计

第一版使用稳定、强 balance、弱 heuristic 的 cost：

```text
split_cost(left, right) =
      A * abs(left_count - right_count)
    + B * (bbox_hpwl(left) + bbox_hpwl(right))
    + C * source_bias(left, right)
    + D * est_delay_bias(left, right)
```

推荐默认权重：

```text
A = 1000000.0
B = 1.0
C = 0.05
D = 0.01
```

解释：

- `A` 非常大，强制优先保持左右 sink_count 接近，避免 topology 退化成长链。
- `B` 鼓励子区域 bbox 紧凑，降低后续 wirelength。
- `C` 是很弱的 source distance tie-break，避免 root 附近拓扑极端偏斜。
- `D` 是极弱的 est_delay tie-break，不允许它破坏 median balance 和几何连续划分。

其中：

```text
bbox_hpwl(S) = (bbox_ux(S) - bbox_lx(S)) + (bbox_uy(S) - bbox_ly(S))

center(S) = COG(S)

source_bias(left, right) =
    abs(manhattan(source, center(left)) - manhattan(source, center(right)))
```

`est_delay_bias` 可以用子集合 bbox 粗略估计：

```text
est_delay_bias(left, right) = abs(estimate_subtree_delay(left) - estimate_subtree_delay(right))
```

为了简化，也允许第一版直接令：

```text
est_delay_bias = 0
```

重要：`split_cost` 只在 median 附近候选 k 中选择，不允许产生非连续划分。

## 奇数 sink 数量处理

MMM-style top-down bisection 天然支持奇数 sink 数量。

例如 n = 15：

```text
mid = 7
candidate k around 7
最终可能切成 7/8 或 8/7
```

递归继续处理每个子集，直到每个 leaf 只包含一个 sink。

不需要 unmatched carry-over，也不需要 active cluster pair matching。

## 为什么不再使用 pair_cost / Greedy RGM

旧版 Greedy RGM 的逻辑是从所有 active clusters 中反复选择一对 `pair_cost` 最小的 cluster 进行 bottom-up 合并。它容易生成几何上交错的拓扑，例如矩形四角点中错误地合并对角线 pair，导致后续 single-metal routing 出现交叉、物理 loop 或非法 CTS route。

由于本作业中 routing 不合法会直接失败，而 topology 阶段的估计 skew 后续可以由 BU / TD / buffer 修正，因此 treer 不再使用全局 pair matching，也不再实现 `pair_cost()` 作为主算法。

新的 top-down MMM 算法只允许连续几何划分：

```text
sorted by x or y
left  = sorted[0:k]
right = sorted[k:n]
```

这牺牲了一些 topology 阶段局部 wirelength 灵活性，但显著提高了 single-metal routing 的稳定性。

## est_delay 定义

`est_delay[node]` 表示从该 cluster 的抽象入口点到其所有下游 sinks 的估计最大路径 delay。它只用于 topology generation 阶段的 debug / weak tie-break，不是最终 evaluator delay，也不是 DME bottom-up 的精确 delay。

leaf node：

```text
est_delay[leaf] = 0
```

internal node 创建时使用 DME-like 的估计。设：

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
- 每个 internal node 的 left/right 子树 sink set 不重叠，且 union 等于该 node 的 sink set。
- 每次递归切分产生的 left/right 子集都非空。
- 每个 node 的 region 合法：`region_lx <= region_ux` 且 `region_ly <= region_uy`。
- 每个 internal node 的 left/right region 应来自该 node region 的一次 X 或 Y slicing split。

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

NODE <id> <parent> <left> <right> <is_leaf> <sink_index> <sink_count> <cx> <cy> <bbox_lx> <bbox_ly> <bbox_ux> <bbox_uy> <region_lx> <region_ly> <region_ux> <region_uy> <est_delay>
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
- internal node 的真实物理坐标尚未由 DME top-down 决定，所以可视化时可以先用 `cx/cy` 作为 separator-aware MMM abstract center。
- `region_*` 字段用于 debug slicing partition；可视化脚本可以画出每个 subtree 的 owned region，帮助检查交叉风险。

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