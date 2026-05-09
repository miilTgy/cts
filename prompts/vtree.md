请只实现一个 Python 可视化脚本 `vtree.py`，不要修改 C++ 代码，不要实现 CTS/DME/router/evaluator。目标是读取 `treer` debug 输出的树结构文件，并把它可视化为左右两个子图：左侧显示二叉树拓扑结构，右侧在 die grid 上显示 tree 的几何连线。同时，根据所有 leaf/sink 节点的 `est_delay` 计算全树 `max_est_skew`，并在终端和图标题中显示。

## 输入文件

### 1. 树结构文件

树结构文件位于：

```text
tree/sample<k>.txt
```

格式由 `treer.md` 规定，示例如下：

```text
TREE_VALID 1
ROOT 28
NUM_NODES 29
NUM_SINKS 15

NODE <id> <parent> <left> <right> <is_leaf> <sink_index> <sink_count> <cx> <cy> <bbox_lx> <bbox_ly> <bbox_ux> <bbox_uy> <region_lx> <region_ly> <region_ux> <region_uy> <est_delay>
...

LEAF <node_id> <sink_index> <sink_id> <x> <y>
...

EDGE <parent_id> <child_id>
...
```

需要解析：

- `TREE_VALID`
- `ROOT`
- `NUM_NODES`
- `NUM_SINKS`
- 所有 `NODE` 行
- 所有 `LEAF` 行
- 所有 `EDGE` 行

其中 `NODE` 行字段含义为：

```text
NODE id parent left right is_leaf sink_index sink_count cx cy bbox_lx bbox_ly bbox_ux bbox_uy region_lx region_ly region_ux region_uy est_delay
```

注意：

- `cx/cy` 是 topology generation 阶段的抽象 cluster center。
- 新版 treer 的 `cx/cy` 对 internal node 是 separator-aware MMM abstract center。
- `region_*` 是该 subtree 的 slicing-owned debug region，用于可视化/检查区域切分。
- 为了兼容旧 debug 文件，`vtree.py` 可以同时接受不含 `region_*` 的旧版 `NODE` 行；旧版行解析时 region 字段可填 0。
- leaf node 的真实 sink 坐标可从 `LEAF` 行读取。
- internal node 的真实 DME 坐标尚未生成，右侧几何图中暂时使用 `cx/cy` 作为该 internal node 的可视化位置。

### 2. 原始 sample 输入文件

原始输入文件位于：

```text
samples/sample<k>.txt
```

需要从中读取 die grid 大小：

```text
DIE <width> <height>
```

例如：

```text
DIE 130 120
```

右侧 grid 图的 x 范围为 `[0, width]`，y 范围为 `[0, height]`。

## 依赖要求

优先只使用 Python 标准库 + `matplotlib`。不要依赖 networkx。原因是本脚本只需要画一棵二叉树，手写布局更可控。

允许使用：

```python
import sys
import os
import matplotlib.pyplot as plt
from dataclasses import dataclass
```

可选使用：

```python
from typing import Dict, List, Tuple
```

如果运行环境没有交互式图形窗口，脚本应能使用非交互 matplotlib backend（例如 `Agg`），并把图片保存到：

```text
tree/sample<k>_vtree.png
```

如果用户显式设置了交互式 `MPLBACKEND`，可以使用 `plt.show()` 打开窗口。

## 数据结构建议

使用 dataclass：

```python
@dataclass
class Node:
    id: int
    parent: int
    left: int
    right: int
    is_leaf: bool
    sink_index: int
    sink_count: int
    cx: float
    cy: float
    bbox_lx: int
    bbox_ly: int
    bbox_ux: int
    bbox_uy: int
    region_lx: int
    region_ly: int
    region_ux: int
    region_uy: int
    est_delay: float
```

另外维护：

```python
nodes: Dict[int, Node]
leaf_info: Dict[int, Tuple[int, str, int, int]]
edges: List[Tuple[int, int]]
root: int
num_nodes: int
num_sinks: int
```

其中 `leaf_info[node_id] = (sink_index, sink_id, x, y)`。

## 文件解析函数

请实现：

```python
def parse_tree_file(path: str):
    ...
```

返回：

```python
root, nodes, leaf_info, edges
```

解析规则：

- 跳过空行。
- 使用 `line.split()`。
- 根据首 token 判断行类型。
- `TREE_VALID` 必须为 1，否则打印错误并退出。
- `NODE` 行解析成 `Node`。
- `NODE` 行应优先解析新版 18 个值（不含开头 `NODE` token）：`... bbox_uy region_lx region_ly region_ux region_uy est_delay`。
- 若遇到旧版 14 个值（不含开头 `NODE` token）：`... bbox_uy est_delay`，也应兼容解析，并把 `region_*` 设为 0。
- `LEAF` 行解析成 `leaf_info`。
- `EDGE` 行解析成 `(parent, child)`。

请实现：

```python
def parse_die_size(path: str):
    ...
```

返回：

```python
width, height
```

解析规则：

- 打开 `samples/sample<k>.txt`。
- 逐 token 或逐行读取。
- 找到 `DIE <width> <height>` 后返回。
- 如果找不到 DIE，打印错误并退出。

## max_est_skew 计算

请实现：

```python
def compute_max_est_skew(nodes, leaf_info):
    ...
```

含义：

- `NODE` 行中的 `est_delay` 是 treer 阶段记录的估计 delay。
- 全树 `max_est_skew` 只用 leaf/sink 节点计算，不用 internal node 计算。
- 对所有 leaf node 取 `est_delay` 的最大值和最小值：

```text
max_est_skew = max(est_delay[leaf]) - min(est_delay[leaf])
```

实现规则：

```python
leaf_delays = []
for node_id in leaf_info:
    if node_id in nodes:
        leaf_delays.append(nodes[node_id].est_delay)
if not leaf_delays:
    return 0.0
return max(leaf_delays) - min(leaf_delays)
```

注意：

- 这里的 `max_est_skew` 只是 topology generation debug 指标，不是最终 evaluator skew。
- 由于当前 treer 输出的 leaf `est_delay` 可能全为 0，因此这个值可能也是 0；这不代表最终 CTS skew 为 0。
- 如果以后 treer/debug 文件改为输出 leaf 的 accumulated root-to-leaf estimated delay，则本函数仍可直接用于估计全树 skew。

## 左侧：二叉树拓扑结构可视化

左侧子图显示纯拓扑，不使用真实坐标。

重要：当 sink 数量较多时，左侧二叉树不能固定画在一个小 subplot 中，否则 node 和文字会挤在一起。请使用自适应画布和自适应字体策略：

- 根据 leaf 数量和树深度动态增大 figure 宽度和高度。
- 左侧 topology subplot 的宽度比例要大于右侧 grid subplot，例如 `gridspec_kw={"width_ratios": [1.4, 1.0]}`，因为拓扑树横向更容易拥挤。
- 不要使用 `matplotlib.patches.Circle` 按 data coordinate 半径画 node。因为左侧拓扑图需要 `ax.set_aspect("auto")`，data-coordinate circle 会被 x/y 轴缩放拉成长椭圆。请改用 `ax.scatter()` 或 `ax.text(..., bbox=dict(boxstyle="circle", ...))` 这类 screen-space marker/text bbox，使圆片在屏幕上始终保持圆形。
- node marker 大小、字体大小需要随 leaf 数量缩小，但要设置下限，避免文字完全不可读。
- 不要强制 `ax.set_aspect("equal")` 导致拓扑图被压扁；左侧 topology 图建议使用 `ax.set_aspect("auto")`，并手动设置 x/y margin。

基础显示要求：

- 每个 node 用圆片表示。
- internal node 的圆片中间显示 node 的 `<id>`。
- sink/leaf node 的圆片中间显示 `<sink_id>`，例如 `L0`、`R3`。
- parent-child edge 用黑线连接。
- root 在最上方，leaf 在最下方。
- 二叉树左右顺序按 `left/right` 字段。
- 不要显示坐标轴。

建议实现一个递归 layout：

```python
def compute_topology_layout(root, nodes):
    ...
```

返回：

```python
pos: Dict[int, Tuple[float, float]]
```

推荐布局方法：

1. 对树做 DFS。
2. leaf 从左到右依次分配 x 坐标：`0, 1, 2, ...`。
3. internal node 的 x 坐标取左右 child x 坐标平均值。
4. y 坐标使用负的 depth，root depth = 0，child depth = parent depth + 1。

伪代码：

```text
next_leaf_x = 0

def dfs(node_id, depth):
    if node is leaf:
        x = next_leaf_x
        next_leaf_x += 1
    else:
        dfs(left, depth + 1)
        dfs(right, depth + 1)
        x = (pos[left].x + pos[right].x) / 2
    y = -depth
    pos[node_id] = (x, y)
```

另外实现树深度计算函数，用于动态决定 figure 高度：

```python
def compute_tree_depth(root, nodes):
    def dfs(node_id):
        node = nodes[node_id]
        if node.is_leaf:
            return 0
        return 1 + max(dfs(node.left), dfs(node.right))
    return dfs(root)
```

画图函数：

```python
def draw_topology(ax, root, nodes, leaf_info, edges):
    ...
```

绘制要求：

- 先画 edge，再画 node。
- edge 用黑色线，linewidth 可设为 1.0 到 1.5。
- node 不要用 `patches.Circle`。请使用 `ax.scatter()` 画圆形 marker，因为 scatter 的 marker size 使用 points²，是屏幕空间大小，不会因为 `ax.set_aspect("auto")` 被拉成长椭圆。
- 根据 leaf 数量设置自适应参数，例如：

```python
num_leaves = max(1, len(leaf_info))
marker_size = max(18, min(90, 2200 / num_leaves))
font_size = max(4, min(8, int(150 / max(num_leaves, 1))))
```

- 推荐画 node 的方式：先用 `ax.scatter([x], [y], s=marker_size, facecolors="white", edgecolors="black", linewidths=1.0, zorder=3)` 画圆片，再用 `ax.text(x, y, label, ha="center", va="center", fontsize=font_size, zorder=4)` 在圆片中心写 id 或 sink_id。
- 如果 label 仍然溢出，优先缩小 `font_size`；不要为了容纳文字把圆片变成椭圆。
- 左侧 topology 图继续使用 `ax.set_aspect("auto")`，否则节点多时整棵树会被压得很小。但正因为使用 `auto`，所以 node 必须用 screen-space marker，而不能用 data-coordinate circle patch。
- 根据布局结果设置边界，例如：

```python
xs = [p[0] for p in pos.values()]
ys = [p[1] for p in pos.values()]
ax.set_xlim(min(xs) - 1.0, max(xs) + 1.0)
ax.set_ylim(min(ys) - 1.0, max(ys) + 1.0)
```

- `ax.axis("off")`。
- 标题：`Binary Topology`。

- 不要使用如下写法：

```python
circle = plt.Circle((x, y), radius=node_radius, ...)
ax.add_patch(circle)
```

这种写法在 `ax.set_aspect("auto")` 时会把圆片拉成长椭圆。

## 右侧：grid 上画 tree 的几何连线

右侧子图显示 tree 在原始 die grid 中的几何近似位置。

要求：

- grid 大小从 `samples/sample<k>.txt` 的 `DIE width height` 读取。
- x 范围 `[0, width]`，y 范围 `[0, height]`。
- 坐标系和输入坐标一致：x 向右，y 向上。
- 画出网格线。
- edge 一律用粗黑线。
- node 和 sink 全部用黑圆点。
- 右侧图中不要标 node 名字、sink 名字或编号。
- internal node 暂时使用 `NODE` 行中的 `cx/cy` 作为位置；新版 treer 中它是 region-aware separator center。
- leaf/sink node 使用 `LEAF` 行中的真实 sink 坐标。

请实现：

```python
def get_geo_pos(node_id, nodes, leaf_info):
    ...
```

规则：

```text
if node_id in leaf_info:
    return sink x/y from LEAF line
else:
    return nodes[node_id].cx, nodes[node_id].cy
```

请实现：

```python
def draw_grid_tree(ax, root, nodes, leaf_info, edges, width, height):
    ...
```

绘制要求：

- 对每条 `EDGE parent child`：
  - 取 parent 和 child 的几何位置。
  - 用 `ax.plot([x1, x2], [y1, y2], color="black", linewidth=2.5)` 画粗黑线。
- 对所有 node：
  - 取几何位置。
  - 用 `ax.scatter(xs, ys, color="black", s=20, zorder=3)` 画黑圆点。
- 设置：

```python
ax.set_xlim(0, width)
ax.set_ylim(0, height)
ax.set_aspect("equal", adjustable="box")
ax.grid(True, linewidth=0.5, alpha=0.4)
ax.set_title("Tree on Die Grid")
```

- 不要给点加文字 label。

注意：这里的几何图只是 topology generation 的调试视图，不是最终 DME routing。edge 可以直接画 parent-child 直线，不需要画 Manhattan 折线。

## 主函数要求

实现：

```python
def main():
    ...
```

行为：

1. 检查命令行参数。
2. 如果不是一个 sample index，打印用法：

```text
Usage: python3 vtree.py <sample_index>
Example: python3 vtree.py 1
```

3. 构造路径：

```python
tree_path = f"tree/sample{idx}.txt"
sample_path = f"samples/sample{idx}.txt"
```

4. 检查文件是否存在。如果不存在，打印清晰错误并退出。
5. 调用：

```python
root, nodes, leaf_info, edges = parse_tree_file(tree_path)
width, height = parse_die_size(sample_path)
```

之后计算并打印：

```python
max_est_skew = compute_max_est_skew(nodes, leaf_info)
print(f"max_est_skew = {max_est_skew:.3f}")
```

6. 根据 leaf 数量和树深度创建自适应大小的左右两个子图：

```python
num_leaves = max(1, len(leaf_info))
max_depth = compute_tree_depth(root, nodes)

fig_width = max(14, min(36, 0.32 * num_leaves + 10))
fig_height = max(7, min(18, 0.70 * max_depth + 4))

fig, (ax1, ax2) = plt.subplots(
    1,
    2,
    figsize=(fig_width, fig_height),
    layout="constrained",
    gridspec_kw={"width_ratios": [1.4, 1.0]},
)
```

7. 调用：

```python
draw_topology(ax1, root, nodes, leaf_info, edges)
draw_grid_tree(ax2, root, nodes, leaf_info, edges, width, height)
```

8. 设置总标题：

```python
fig.suptitle(f"Topology Tree Visualization: sample{idx} | max_est_skew={max_est_skew:.3f}")
```

9. 根据 matplotlib backend 输出：

```python
backend = plt.get_backend().lower()
if "agg" in backend or "pdf" in backend or "svg" in backend:
    output_path = f"tree/sample{idx}_vtree.png"
    fig.savefig(output_path, dpi=160)
    print(f"saved visualization: {output_path}")
else:
    plt.show()
```

## 输出效果要求

运行：

```bash
python3 vtree.py 1
```

应弹出一个 matplotlib 窗口；如果当前环境使用非交互 backend，则应保存图片到 `tree/sample1_vtree.png`：

- 左侧：二叉树拓扑结构；圆片中 internal node 显示 node id，sink leaf 显示 sink id。
- 右侧：die grid 上的 tree 几何连接；所有 edge 是粗黑线，所有 node/sink 是黑圆点，不显示文字。
- 终端输出 `max_est_skew = ...`。
- 非交互 backend 下终端额外输出 `saved visualization: tree/sample<k>_vtree.png`。
- figure 总标题中显示 `max_est_skew=...`。
- 当 sample 的 sink 数量较多时，figure 会自动变大，左侧 topology subplot 会获得更多宽度，node 圆片和字体会自动缩小，尽量避免文字溢出和节点互相遮挡。
