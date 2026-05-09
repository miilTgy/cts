请只实现 topology generation 模块，不要实现 DME bottom-up、DME top-down、buffer insertion、router、evaluator 或输出文件生成器。目标是根据 `common::Problem` 中的 source/sinks 生成一棵供 Buffered-DME 使用的抽象二叉拓扑树。

建议文件名为 `treer.h` 和 `treer.cc`；如需对外暴露树节点和树结构，可适当放在 `common.h` 中。最终在 `main.cc` 或后续 CTS flow 中只需要调用一次：

```cpp
treer::TopologyTree tree = treer::build(problem, argv[1]);
```

即可得到一棵二叉拓扑树。debug 开启时，还需要在 `./tree/` 目录下额外输出一个以输入文件名命名的树结构记录文件，方便后续写脚本可视化。

# Topology Generation: Plain RGM-style Recursive Geometric Matching

## 算法目标

DME 后续会在给定 topology 上做 bottom-up merging segment、buffer insertion 和 top-down embedding。因此 `treer` 的职责不是在 topology 阶段追求 skew=0，而是优先生成局部性好、少交叉、稳定可布线的二叉 topology。

`treer` 使用普通 RGM（Recursive Geometric Matching）风格的 bottom-up topology generation：

- 初始 active clusters 是所有 sink leaf。
- 每一轮在 active clusters 上做 min-cost geometric matching。
- 被匹配的两个 cluster 合并成一个 internal node。
- 若 active cluster 数量为奇数，允许留下一个 unmatched cluster，直接 carry 到下一轮。
- 重复直到只剩一个 cluster，作为 root。

核心优先级：

```text
少交叉 / 局部相邻优先 > wirelength heuristic > tree balance
```

不要为了强制 balance 牺牲几何局部性。树稍微不平衡可以接受，因为后续 BU/TD/buffer 会负责 skew 修正；但 topology 如果一开始就大量交叉，后续 single-metal routing 很容易失败。

## 文件与 namespace 要求

- 所有 topology generation 相关内容放在 `namespace treer` 中。
- `treer.h`：声明对外 API。
- `treer.cc`：实现 RGM-style recursive geometric matching 具体逻辑。
- `common.h`：放需要被其他模块使用的树节点/拓扑树数据结构。
- 不要使用 `try / catch / exception` 作为主要错误处理方式，尽量使用 `if-else` 检查并返回 `valid=false` 和 `error_msg`。
- 可使用 C++ 标准库的 `std::vector`、`std::string`、`std::limits`、`std::cmath`、`std::cerr` 等。

## 数据结构建议放在 `common.h`

如果 `common.h` 已经有 `common::Problem`、`common::Sink`、`common::Point`，请在同一个 `namespace common` 中补充或复用：

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

    // Abstract cluster center used only for topology generation/debug.
    // It may be fractional. DME top-down will decide the real embedding coordinate later.
    double cx = 0.0;
    double cy = 0.0;

    int bbox_lx = 0;
    int bbox_ly = 0;
    int bbox_ux = 0;
    int bbox_uy = 0;

    // Region fields are optional for compatibility with old debug tools.
    // RGM does not rely on slicing regions, so each node may store its subtree bbox
    // or the whole die region here.
    int region_lx = 0;
    int region_ly = 0;
    int region_ux = 0;
    int region_uy = 0;

    // Estimated maximum downstream delay from this cluster root to its sinks.
    // This is only a topology-generation heuristic/debug value, not final evaluator delay.
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
   - 如果 sink 数量大于 1，使用 RGM-style recursive geometric matching 构造二叉树。
   - build 成功后，如果 debug 开启，调用 `debug_output(tree, problem)` 和 `debug_output_file(tree, problem, input_path)`。

## RGM 主流程

该算法是 bottom-up matching，不是 top-down bisection。

整体流程：

```text
Input:
    common::Problem problem

Output:
    common::TopologyTree tree

build(problem):
    active = []
    for each sink:
        leaf_id = create_leaf(sink)
        active.push(leaf_id)

    while active.size() > 1:
        active = rgm_round(active, problem, tree)

    tree.root = active[0]
    tree.nodes[tree.root].parent = -1
    tree.valid = true
    return tree
```

一轮 RGM：

```text
rgm_round(active, problem, tree):
    pairs = enumerate all unordered pairs (active[i], active[j])
    compute pair_cost for each pair
    sort pairs by cost, then deterministic tie-break

    used = empty set
    selected_pairs = []
    selected_segments = []

    for each pair in sorted pairs:
        if either endpoint already used: continue
        if pair segment conflicts with selected_segments: continue
        if pair segment passes through forbidden sink/cluster center: continue

        select this pair
        mark both endpoints used
        add pair segment to selected_segments

    new_active = []
    for each selected pair (a,b):
        parent = create_internal_from_pair(a,b)
        set nodes[a].parent = parent
        set nodes[b].parent = parent
        new_active.push(parent)

    for each active node not used:
        new_active.push(active node)   // odd-node or blocked-node carry

    if selected_pairs is empty:
        fallback_select_one_best_pair_without_crossing_check(active)
        // must guarantee progress

    return new_active
```

要求：

- 每轮必须保证 `new_active.size() < active.size()`，否则设置 error。
- 正常情况下一个奇数 active node 会 carry 到下一轮。
- 如果因为 crossing/obstacle 过滤导致多个 node unmatched，也允许 carry，但必须至少合并一对。
- 不要强制每轮完美 matching；普通 RGM 可以有 unmatched carry。
- 不要用 MMM 的 `choose_split_axis` / `sort by axis` / `median split` 作为主算法。

## Active cluster 表示

直接用 `tree.nodes[node_id]` 作为 active cluster。

每个 cluster 的几何代表点是：

```text
center(node) = (node.cx, node.cy)
```

leaf center 是 sink 坐标。

internal center 是两个 child 的 tapping point 近似位置。由于这里仅负责 topology，不需要求严格物理 tap point，允许使用小数坐标。

每个 cluster 的 bbox 是所有 descendant sinks 的 bbox，用于 pair_cost 和 debug。

## Tap point / internal center 规则

RGM 课件里的 tapping point 是两个 subtree 连接边上使 skew 最小的点。这里不需要求最终真实物理位置，只需要一个 topology/debug 代表中心，因此用曼哈顿距离的一维参数来近似。

设两个 child cluster 为 `a` 和 `b`：

```text
A = center(a) = (ax, ay)
B = center(b) = (bx, by)
dl = est_delay[a]
dr = est_delay[b]
D  = |ax - bx| + |ay - by|
```

如果 `D == 0`：

```text
tap = A
parent.est_delay = max(dl, dr)
```

否则，理想 tap point 到 A 的曼哈顿距离为：

```text
t = (D + dr - dl) / 2
```

将 `t` clamp 到 `[0, D]`：

```text
t = clamp(t, 0, D)
```

然后只需生成一个抽象坐标，不要求真实 routing。为了 deterministic，沿一条固定 Manhattan path 从 A 走向 B：先走 x，再走 y。

```text
remaining = t
sx = sign(bx - ax)
sy = sign(by - ay)
dx = |bx - ax|
dy = |by - ay|

if remaining <= dx:
    tap.x = ax + sx * remaining
    tap.y = ay
else:
    tap.x = bx
    tap.y = ay + sy * (remaining - dx)
```

该 tap 坐标可以是 double。

parent 的估计 delay：

```text
parent.est_delay = max(dl + t, dr + (D - t))
```

如果 `abs(dl - dr) <= D`，这个值等价于 `(dl + dr + D) / 2`。如果无法完全平衡，clamp 后会退化为更保守的一侧。

注意：

- treer 阶段不需要把 tap point 修到 grid integer。
- treer 阶段不需要生成 parent-child route。
- DME top-down 会在后续重新决定真实 embedding coordinate。
- 这里的 `cx/cy` 只是 topology/debug/heuristic 的代表点。

## pair_cost 设计

目标是优先局部连接、减少 topology 交叉，而不是强 balance。

候选 pair `(a,b)` 的基础 cost：

```text
base_cost(a,b) =
      1.0 * L1(center(a), center(b))
    + 0.15 * bbox_hpwl(union_bbox(a,b))
    + 0.05 * abs(est_delay[a] - est_delay[b])
    + 0.001 * source_tie_break(a,b)
```

其中：

```text
L1(A,B) = |Ax-Bx| + |Ay-By|
bbox_hpwl(box) = (box.ux - box.lx) + (box.uy - box.ly)
source_tie_break(a,b) = L1(source, midpoint(center(a), center(b)))
```

解释：

- L1 距离是主项，鼓励最近邻优先。
- union bbox HPWL 鼓励合并紧凑 cluster，避免跨越式合并。
- est_delay 差只是弱项，不要让它破坏几何局部性。
- source tie-break 非常弱，只用于 deterministic 和轻微偏向。

不要加入强 sink-count balance penalty。可以保留极弱项：

```text
+ 0.0001 * abs(sink_count[a] - sink_count[b])
```

但不要让它影响局部几何匹配。

## 非交叉优先规则

RGM 每轮选择 pair 时，优先保证新选择的 pair segment 不与本轮已选择的 pair segment 交叉。

定义候选 pair segment：

```text
segment(a,b) = straight line between center(a) and center(b)
```

用于 topology 交叉检查，不代表最终 routing。

选择 pair 时：

```text
if segment(a,b) properly intersects any selected segment:
    reject this pair
```

建议实现：

- 使用 double 坐标做 2D orientation test。
- proper intersection 指两个线段在内部相交。
- 如果只是在端点接触，通常不算冲突。
- 由于同一轮 selected pairs 没有共享 endpoint，所以端点接触极少发生。

还要避免明显穿过已有 active cluster center：

```text
if segment(a,b) passes through center(c) for some active node c not equal a,b:
    add huge penalty or reject
```

这里用近似判断即可：如果 `distance_point_to_segment(center(c), segment(a,b)) < eps` 且投影在线段内部，则认为穿过。

优先策略：

```text
1. 先尝试不交叉、不穿 active center 的 pair。
2. 如果这样导致本轮无法合并任何 pair，则 fallback 到 cost 最小的一对，保证算法前进。
3. fallback 时仍应优先避开 active center；如果实在没有合法 pair，才允许纯 cost 最小 pair。
```

## 奇数 sinks / unmatched carry 规则

普通 RGM 不要求每一轮所有 cluster 都被匹配。

如果 `active.size()` 是奇数：

```text
本轮最多匹配 floor(active.size()/2) 对
剩余 1 个 unmatched cluster carry 到下一轮
```

如果由于非交叉规则导致不止一个 unmatched，也允许：

```text
所有 unmatched active node 原样 push 到 new_active
```

但必须保证本轮至少有一个 pair 被合并。若没有 pair 被合并：

```text
选择 pair_cost 最小的一对强制合并
```

这样可以保证 while loop 终止。

## create_leaf

```text
create_leaf(sink_index):
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
    node.region = whole die region or sink bbox
    node.est_delay = 0
    push node into tree.nodes
    return node.id
```

## create_internal_from_pair

```text
create_internal_from_pair(left_id, right_id):
    left  = tree.nodes[left_id]
    right = tree.nodes[right_id]

    p.id = tree.nodes.size()
    p.is_leaf = false
    p.sink_index = -1
    p.parent = -1
    p.left = left_id
    p.right = right_id
    p.sink_count = left.sink_count + right.sink_count

    p.bbox = union(left.bbox, right.bbox)
    p.region = p.bbox or whole die region

    tap = compute_tap_point_by_manhattan_distance(left, right)
    p.cx = tap.x
    p.cy = tap.y
    p.est_delay = tap.parent_est_delay

    push p into tree.nodes
    return p.id
```

为了输出 deterministic，可以在 pair `(a,b)` 内部固定 child 顺序：

```text
if center(a).x < center(b).x: left=a,right=b
else if center(a).x > center(b).x: left=b,right=a
else if center(a).y < center(b).y: left=a,right=b
else smaller node_id first
```

这只是树文件稳定性，不代表物理左右。

## 为什么不再使用 MMM / median split

旧版 top-down MMM-style recursive geometric bisection 会先把 sink set 切成两个连续子集，再在每个子集中心附近创建 abstract center。它的优点是树高度稳定，但缺点是容易过早创建大 cluster center：

```text
先创建一个四点 cluster 的中心，再连接上下/左右子 cluster
```

在一些布局中，这会导致 parent-to-child debug edge 或后续 L-shape route 穿越其他局部 cluster，产生交叉 topology。

本版改用普通 RGM：

```text
先连接局部最近的 sink/cluster，再逐层合并
```

这样更容易得到：

```text
局部 pair -> 小 cluster -> 大 cluster
```

例如左下角 4 个点会更倾向于先形成上面两点 pair、下面两点 pair，再把两个 pair 合并，而不是先在四点中间创建一个总 CG。

## est_delay 定义

`est_delay[node]` 表示从该 cluster 的抽象入口点到其所有下游 sinks 的估计最大路径 delay。它只用于 topology generation 阶段的 debug / weak tie-break，不是最终 evaluator delay，也不是 DME bottom-up 的精确 delay。

leaf node：

```text
est_delay[leaf] = 0
```

internal node 使用 tap point 的曼哈顿距离参数估计：

```text
D  = L1(center(left), center(right))
dl = est_delay[left]
dr = est_delay[right]
t  = clamp((D + dr - dl) / 2, 0, D)
est_delay[parent] = max(dl + t, dr + (D - t))
```

## 合法性与一致性检查

在 `build()` 结束前做轻量检查：

- `tree.root >= 0`。
- `tree.nodes[tree.root].parent == -1`。
- leaf 数量等于 `problem.sinks.size()`。
- 每个 non-root node 的 parent 有效。
- 每个 internal node 的 left/right 都有效且不同。
- 每个 internal node 的 `sink_count = left.sink_count + right.sink_count`。
- 每个 internal node 的 bbox 等于左右 child bbox 的 union。
- 每个 sink 只出现在一个 leaf 中。
- 从 root DFS 可以访问所有 node，且不存在 cycle。

RGM 不需要检查 slicing region，也不要求 child regions 来自一次 X/Y slicing split。请移除或禁用旧版 MMM 的 slicing-region validation，例如：

```text
child_regions_match_slicing(...)
```

否则 RGM 生成的合法 topology 会被错误判 invalid。

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
- internal node 的真实物理坐标尚未由 DME top-down 决定，所以可视化时可以先用 `cx/cy` 作为 RGM tap/debug center。
- RGM 允许 `cx/cy` 是小数，输出时不要强制转成 int。

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