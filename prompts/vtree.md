下面这版提示词把右侧逻辑改成：internal tap 点就是两个 child 的几何中点，并按 `node_kind` 使用不同颜色/marker；不再使用 NODE.cx/cy 作为右侧 tap 点位置，也不再引入额外的 B 分叉点。

请实现/修改 `vtree.py`，它是一个 Python 可视化脚本。不要修改 C++ 代码，不要实现 CTS/DME/router/evaluator。目标是读取 `tree/sample<k>_vtree.txt` 中 treer 输出的树结构，以及 `samples/sample<k>.txt` 中的 die/source 信息，画出左右两个子图：
1. 左侧：二叉树拓扑结构图。
2. 右侧：在 die grid 上显示 tree 的几何连接。
同时计算 source 到每个 sink 沿树路径的曼哈顿距离，并输出：
```text
max_est_skew = max(delay) - min(delay)

注意：max_est_skew 不要使用 NODE 行中的 `subtree_skew_to_node` 字段。

⸻

输入文件

树文件：

tree/sample<k>_vtree.txt

格式示例：

TREE_VALID 1
ROOT 28
NUM_NODES 29
NUM_SINKS 15
NODE <id> <parent> <left> <right> <is_leaf> <sink_index> <sink_count> <cx> <cy> <bbox_lx> <bbox_ly> <bbox_ux> <bbox_uy> <region_lx> <region_ly> <region_ux> <region_uy> <subtree_skew_to_node> <node_kind>
LEAF <node_id> <sink_index> <sink_id> <x> <y>
EDGE <parent_id> <child_id>

需要解析：

* TREE_VALID
* ROOT
* NODE
* LEAF
* EDGE

NODE 行有 19 个值，不含开头 NODE token：

id parent left right is_leaf sink_index sink_count cx cy bbox_lx bbox_ly bbox_ux bbox_uy region_lx region_ly region_ux region_uy subtree_skew_to_node node_kind

`node_kind` 是以下字符串之一：`SINK`、`CLUSTER_INTERNAL`、`CLUSTER_ACCESS`、`GLOBAL`。不需要兼容旧版 tree 文件格式。

sample 文件：

samples/sample<k>.txt

需要读取：

DIE <width> <height>
SOURCE <id> <x> <y>

⸻

数据结构建议

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
    subtree_skew_to_node: float
    node_kind: str

维护：

nodes: Dict[int, Node]
leaf_info: Dict[int, Tuple[int, str, int, int]]
edges: List[Tuple[int, int]]
root: int

其中：

leaf_info[node_id] = (sink_index, sink_id, x, y)

⸻

基础坐标函数

请实现：

def get_leaf_pos(node_id, leaf_info):
    return leaf_info[node_id][2], leaf_info[node_id][3]

请实现：

def get_node_debug_pos(node_id, nodes, leaf_info):
    if node_id in leaf_info:
        return leaf_info[node_id][2], leaf_info[node_id][3]
    return nodes[node_id].cx, nodes[node_id].cy

含义：

* get_leaf_pos() 只用于 leaf/sink，返回 LEAF 行真实坐标。
* get_node_debug_pos() 用于计算 max_est_skew，internal node 使用 NODE.cx/cy。
* 右侧 grid 可视化不要直接使用 NODE.cx/cy 作为红色 tap 点位置，右侧红色 tap 点需要按两个 child 的几何中点重新计算。

⸻

max_est_skew 计算

请实现：

def manhattan(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

compute_max_est_skew(root, nodes, leaf_info, source_pos) 规则：

* 不使用 NODE.subtree_skew_to_node。
* 对每个 leaf：
    * 从 leaf 沿 parent 指针向上走到 root。
    * 每一段距离用 get_node_debug_pos(current) 到 get_node_debug_pos(parent) 的曼哈顿距离。
    * 最后加上 root 的 NODE.cx/cy 到 source_pos 的曼哈顿距离。
* 收集所有 leaf delay。
* 返回 max(delay) - min(delay)。
* 如果没有 leaf，返回 0。

注意：这个 skew 是基于 treer debug center 的粗略估计，不是最终 routing skew。

⸻

左侧：二叉树拓扑结构图

左侧是纯 topology，不使用真实坐标。

布局要求：

* 用 DFS 生成二叉树 layout。
* leaf 从左到右依次分配 x 坐标：0, 1, 2, ...。
* internal node 的 x 坐标取左右 child 的平均值。
* y 坐标使用负 depth，root depth = 0。
* root 在最上方，leaf 在最下方。

左侧绘制要求：

* parent-child edge 用黑色直线连接。
* 注意：左侧 topology 图不要画 Manhattan 折线，不要画 elbow polyline，不要画共享 trunk。左侧就是普通二叉树拓扑。
* 每个 node 用圆片表示。
* internal node 圆片中显示 node id。
* leaf node 圆片中显示 sink_id，例如 L0、R3。
* 不同 `node_kind` 使用不同颜色，并添加 legend：`SINK` 黑色、`CLUSTER_INTERNAL` 黄色、`CLUSTER_ACCESS` 蓝色、`GLOBAL` 绿色。
* root node 用金色填充、深橙色边框，并在下方标注 ROOT。
* node 不要用 matplotlib.patches.Circle，因为 ax.set_aspect("auto") 时会被拉成长椭圆。请使用 ax.scatter() 画 screen-space 圆形 marker，再用 ax.text() 写 label。
* 当 sink 数量多时，自适应缩小 marker 和 font。
* 左侧建议 ax.set_aspect("auto")，并 ax.axis("off")。

示例自适应参数：

num_leaves = max(1, len(leaf_info))
marker_size = max(18, min(90, 2200 / num_leaves))
font_size = max(4, min(8, int(150 / max(num_leaves, 1))))

⸻

右侧：die grid 几何树图

右侧显示 tree 在 die grid 中的几何近似位置。

右侧目标效果：

* 黑色粗线构成连续树状结构。
* 按 `node_kind` 用不同颜色/marker 显示 node：`SINK` 黑色圆点、`CLUSTER_INTERNAL` 黄色圆点、`CLUSTER_ACCESS` 蓝色方块、`GLOBAL` 绿色菱形。
* source 用红色三角形表示。
* internal tap 点必须位于它的两个 child 的几何中点，并按 `node_kind` 上色。
* 如果 child 是 sink，则 child 坐标就是 LEAF 坐标。
* 如果 child 是 internal node，则 child 坐标是它自己递归算出来的 visual tap 坐标。

重要禁止项：

* 不要把每条 EDGE parent child 直接画成两点直线。
* 不要把每条 edge 各自独立画成 L 形折线。
* 不要引入额外的 trunk point B。
* 不要使用 P -> B -> L/R 的画法。
* 不要使用 NODE.cx/cy 作为右侧红色 tap 点位置。
* 不要出现红色 tap 圆圈在一处、黑线实际分叉/连接点在另一处的情况。
* 右侧绘制的基本单位不是单条 EDGE，而是 internal node 的二叉合并结构。

⸻

右侧 visual position 计算

右侧需要重新计算每个 node 的可视化位置 visual_pos。

规则：

* leaf/sink 的 visual_pos 是 LEAF 行真实坐标。
* internal node 的 visual_pos 是它的两个 child 的 visual_pos 的几何中点。
* root 作为 tree root 也有自己的 visual_pos，同样由它的两个 child 的 visual_pos 中点决定。
* source 不属于 tree node visual position；source 只额外连到 root 的 visual_pos。

请实现：

def compute_grid_visual_pos(root, nodes, leaf_info):
    visual_pos = {}
    def dfs(u):
        node = nodes[u]
        if node.is_leaf:
            x, y = get_leaf_pos(u, leaf_info)
            visual_pos[u] = (float(x), float(y))
            return visual_pos[u]
        L_id = node.left
        R_id = node.right
        L = dfs(L_id)
        R = dfs(R_id)
        ux = (L[0] + R[0]) / 2.0
        uy = (L[1] + R[1]) / 2.0
        visual_pos[u] = (ux, uy)
        return visual_pos[u]
    dfs(root)
    return visual_pos

说明：

* 这个 visual_pos 只用于右侧 grid 可视化。
* 它不会修改 nodes[u].cx/cy。
* 它不影响 max_est_skew 计算。
* 右侧 internal node 标记必须画在 internal node 的 visual_pos[u] 上，并按 `node_kind` 使用对应样式。

⸻

右侧 edge 绘制方法

请实现：

def draw_grid_tree(ax, root, nodes, leaf_info, edges, width, height, source_pos):
    visual_pos = compute_grid_visual_pos(root, nodes, leaf_info)
    def draw_branch(u):
        node = nodes[u]
        if node.is_leaf:
            return
        P = visual_pos[u]
        L_id = node.left
        R_id = node.right
        L = visual_pos[L_id]
        R = visual_pos[R_id]
        # 红色 tap 点 P 就是两个 child 的中点。
        # 黑线必须直接从 P 连到两个 child。
        ax.plot([P[0], L[0]], [P[1], L[1]],
                color="black", linewidth=2.8, zorder=1)
        ax.plot([P[0], R[0]], [P[1], R[1]],
                color="black", linewidth=2.8, zorder=1)
        draw_branch(L_id)
        draw_branch(R_id)
    # 先画 tree branch
    draw_branch(root)
    # 再画 source 到 root 的连接
    root_pos = visual_pos[root]
    ax.plot([source_pos[0], root_pos[0]], [source_pos[1], root_pos[1]],
            color="black", linewidth=2.8, zorder=1)
    # 之后再画 nodes
    ...

说明：

* 红色 tap 点 P 就是两个 child 的中点。
* 画线时直接 P -> L 和 P -> R。
* 不要再计算 B。
* 不要再画 P -> B -> L/R。
* 因为 P 就是中点，所以两个 child 会自然通过该 node marker 合并。
* 对于底层两个 sink 的合并，node marker 应该正好在两个 sink 坐标的几何中点。
* 对于更高层的合并，node marker 应该在两个 child tap/sink 的中点。

⸻

右侧节点样式

定义统一样式表：

```python
KIND_STYLES = {
    "SINK": {"face": "#202020", "edge": "#202020", "marker": "o"},
    "CLUSTER_INTERNAL": {"face": "#ffe08a", "edge": "#b7791f", "marker": "o"},
    "CLUSTER_ACCESS": {"face": "#90cdf4", "edge": "#2b6cb0", "marker": "s"},
    "GLOBAL": {"face": "#c6f6d5", "edge": "#2f855a", "marker": "D"},
}
```

左右两个子图都要显示 node kind legend。

source：

sx, sy = source_pos
ax.scatter([sx], [sy], marker="^", s=120,
           facecolors="red", edgecolors="darkred",
           linewidths=1.5, zorder=10)
ax.text(sx, sy, "SRC", ha="center", va="bottom",
        fontsize=10, color="darkred", zorder=11)

sink：

sink_xs = []
sink_ys = []
for node_id, (_, sink_id, x, y) in leaf_info.items():
    sink_xs.append(x)
    sink_ys.append(y)
ax.scatter(sink_xs, sink_ys, marker="o", s=45,
           facecolors="#202020", edgecolors="#202020", zorder=5)

sink label 可选，但建议保留：

for node_id, (_, sink_id, x, y) in leaf_info.items():
    ax.text(x, y, sink_id, ha="center", va="center",
            fontsize=sink_font_size, color="black", zorder=7)

internal tap：

for kind in ("CLUSTER_INTERNAL", "CLUSTER_ACCESS", "GLOBAL"):
    xs = []
    ys = []
    for node_id, node in nodes.items():
        if node.node_kind != kind:
            continue
        x, y = visual_pos[node_id]
        xs.append(x)
        ys.append(y)
    style = KIND_STYLES[kind]
    ax.scatter(xs, ys, marker=style["marker"], s=55,
               facecolors=style["face"], edgecolors=style["edge"],
               linewidths=2.0, zorder=6)

注意：

* internal tap 包括 root 对应的 visual tap，并按其 `node_kind` 上色。
* source 仍然单独画红色三角形。
* 如果 root visual tap 和 source 很近，也照常画。

⸻

右侧绘制顺序

在 draw_grid_tree() 中按以下顺序绘制：

1. 设置坐标轴范围和 grid。
2. 调用 draw_branch(root) 画所有黑色粗线。
3. 画 source 到 root visual tap 的黑色粗线。
4. 按 `node_kind` 画所有 internal tap marker。
5. 画所有 sink 黑色实心点和 sink_id label。
6. 画 source 红色三角形和 SRC label。

坐标轴设置：

ax.set_xlim(0, width)
ax.set_ylim(0, height)
ax.set_aspect("equal", adjustable="box")
ax.grid(True, linewidth=0.5, alpha=0.4)
ax.set_title("Tree on Die Grid")

⸻

主函数要求

命令行：

python3 vtree.py <sample_index | tree/sample<k>_vtree.txt>

例如：

python3 vtree.py 1

也支持：

python3 vtree.py tree/sample1_vtree.txt

行为：

1. 构造路径：

tree_path = f"tree/sample{idx}_vtree.txt"
sample_path = f"samples/sample{idx}.txt"

2. 解析 tree 和 sample。
3. 计算并打印：

print(f"max_est_skew = {max_est_skew:.3f}")

4. 创建左右子图。figure 大小根据 sink 数量和树深度自适应：

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

5. 调用：

draw_topology(ax1, root, nodes, leaf_info, edges)
draw_grid_tree(ax2, root, nodes, leaf_info, edges, width, height, source_pos)

6. 总标题显示：

fig.suptitle(
    f"Topology Tree Visualization: sample{idx} | max_est_skew={max_est_skew:.3f}"
)

7. 如果是非交互 backend，保存：

tree/sample<k>_vtree.png

否则 plt.show()。

⸻

最终效果要求

运行：

python3 vtree.py 1

应得到：

* 左侧：普通二叉树 topology；parent-child 是直线；internal node 显示 node id；leaf 显示 sink_id；root 金色高亮并标注 ROOT。
* 右侧：die grid 上的几何树；source 是红色三角形；其他 node 按 `node_kind` 显示颜色/marker。
* 右侧每个 internal tap marker 必须在它的两个 child 的几何中点。
* 右侧底层两个 sink 合并时，tap marker 应正好在两个 sink 的连线中点。
* 右侧更高层合并时，tap marker 应在两个 child tap/sink 的连线中点。
* 右侧黑色粗线直接从 tap marker 连到两个 child。
* 右侧不能是 parent-child 两点直连。
* 右侧不能是每条 edge 各自独立的 L 形折线。
* 右侧不能使用额外的 B 点或 P -> B -> L/R 画法。
* 右侧不能出现红色 tap 点和黑线分叉/连接点明显分离的情况。
* 终端输出 max_est_skew = ...。
* figure 标题显示 max_est_skew=...。
