下面这版提示词匹配最新版 `treer` 输出格式：`NODE` 行最后包含 `node_kind`，并区分 `SINK`、`CLUSTER_INTERNAL`、`CLUSTER_ACCESS`、`CLUSTER_BRIDGE`、`CLUSTER_TOP`、`GLOBAL`。右侧逻辑为：二叉 merge 点就是两个 child 的几何中点；一元 wrapper（例如 `ClusterAccess` / `ClusterTop`）沿用唯一 child 的可视位置；按 `node_kind` 使用不同颜色/marker；不再使用 NODE.cx/cy 作为右侧 tap 点位置，也不再引入额外的 B 分叉点。

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
NODE <id> <parent> <left> <right> <is_sink> <sink_index> <sink_count> <cx> <cy> <bbox_lx> <bbox_ly> <bbox_ux> <bbox_uy> <region_lx> <region_ly> <region_ux> <region_uy> <subtree_skew_to_node> <node_kind>
LEAF <node_id> <sink_index> <sink_id> <x> <y>
EDGE <parent_id> <child_id>

需要解析：

* TREE_VALID
* ROOT
* NODE
* LEAF
* EDGE

NODE 行有 19 个值，不含开头 NODE token：

id parent left right is_sink sink_index sink_count cx cy bbox_lx bbox_ly bbox_ux bbox_uy region_lx region_ly region_ux region_uy subtree_skew_to_node node_kind

`node_kind` 是以下字符串之一：`SINK`、`CLUSTER_INTERNAL`、`CLUSTER_ACCESS`、`CLUSTER_BRIDGE`、`CLUSTER_TOP`、`GLOBAL`。不需要兼容旧版 tree 文件格式。

zero-sink tree 合法格式：

```text
TREE_VALID 1
ROOT -1
NUM_NODES 0
NUM_SINKS 0
```

这种情况下可视化脚本应输出 `max_est_skew = 0.000`，左/右子图显示为空树，不报错。

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
    is_leaf: bool  # parse from NODE is_sink; script may keep this old field name
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
* internal node 的 x 坐标取所有有效 child 的平均值；普通 merge node 有两个 child，一元 wrapper 有一个 child。
* y 坐标使用负 depth，root depth = 0。
* root 在最上方，leaf 在最下方。

左侧绘制要求：

* 左侧必须显示 source：在 root 正上方绘制红色三角形 `SRC`，并用黑色直线连接到 root。
* 左侧 source 是拓扑入口标记，不使用 sample 中的真实几何坐标参与布局。
* zero-sink tree 时左侧显示 `EMPTY TREE`，同时仍显示红色三角形 `SRC`。
* parent-child edge 用黑色直线连接。
* 注意：左侧 topology 图不要画 Manhattan 折线，不要画 elbow polyline，不要画共享 trunk。左侧就是普通二叉树拓扑。
* 每个 node 使用与右侧 die grid 完全一致的 marker shape 和颜色。
* internal node 圆片中显示 node id。
* leaf node 圆片中显示 sink_id，例如 L0、R3。
* 不同 `node_kind` 使用不同颜色，并添加 legend：`SINK` 黑色、`CLUSTER_INTERNAL` 黄色、`CLUSTER_ACCESS` 蓝色、`CLUSTER_BRIDGE` 紫色、`CLUSTER_TOP` 橙色、`GLOBAL` 绿色。
* root node 不额外改成金色；仍按自身 `node_kind` 的 marker/color 显示，只用稍粗边框和下方 `ROOT` 文本标注。
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
* internal tap 点默认位于它的两个 child 的几何中点，并按 `node_kind` 上色。
* 唯一特殊规则：`CLUSTER_TOP` 在右侧 die grid 上的显示位置必须在该 cluster 的 bbox 外侧，用来表示 cluster 对外接入点。
* 如果 child 是 sink，则 child 坐标就是 LEAF 坐标。
* 如果 child 是 internal node，则 child 坐标是它自己递归算出来的 visual tap 坐标。

重要禁止项：

* 不要把每条 EDGE parent child 直接画成两点直线。
* 不要把每条 edge 各自独立画成 L 形折线。
* 不要引入额外的 trunk point B。
* 不要使用 P -> B -> L/R 的画法。
* 不要使用 NODE.cx/cy 作为右侧普通 tap 点位置。`CLUSTER_TOP` 只允许用 parent/source 的 NODE debug 位置判断外部连接方向，不直接把自身显示在 NODE.cx/cy。
* 不要出现红色 tap 圆圈在一处、黑线实际分叉/连接点在另一处的情况。
* 右侧绘制的基本单位不是单条 EDGE，而是 internal node 的二叉合并结构。

⸻

右侧 visual position 计算

右侧需要重新计算每个 node 的可视化位置 visual_pos。

规则：

* leaf/sink 的 visual_pos 是 LEAF 行真实坐标。
* binary internal node 的 visual_pos 是它的两个 child 的 visual_pos 的几何中点。
* unary wrapper node 的 visual_pos 等于唯一 child 的 visual_pos。最新版 treer 中 `CLUSTER_ACCESS` 和 `CLUSTER_TOP` 常作为一元 wrapper 出现。
* 唯一例外是 `CLUSTER_TOP`：无论它是 unary 还是 binary，都必须显示在自己的 `bbox` 外侧。
* root 作为 tree root 也有自己的 visual_pos：普通 root 按 child midpoint；若 root 是 `CLUSTER_TOP`，按同样的 bbox 外侧规则放置。
* source 不属于 tree node visual position；source 只额外连到 root 的 visual_pos。

### ClusterTop 外侧放置规则

对 `node.node_kind == "CLUSTER_TOP"`：

1. 先递归计算 child 的 visual_pos，保证其它 node 的连接方式不变。
2. 再选择 `ClusterTop` 自己的 visual_pos，不使用 child midpoint。
3. 若该 `ClusterTop` 有 parent 且 parent 是 `GLOBAL`，用 parent 的 debug 坐标 `(nodes[parent].cx, nodes[parent].cy)` 判断外部 global 连接来自 bbox 哪一侧。
4. 若该 `ClusterTop` 没有 parent，也就是它是整棵树 root，则用 `source_pos` 判断外部连接方向。
5. 将 anchor 点与 cluster bbox center 比较，选择绝对位移更大的轴：
   - anchor 在 bbox 下方 / `dy < 0`：放在 `bbox_ly - margin`。
   - anchor 在 bbox 上方 / `dy >= 0`：放在 `bbox_uy + margin`。
   - anchor 在 bbox 左侧 / `dx < 0`：放在 `bbox_lx - margin`。
   - anchor 在 bbox 右侧 / `dx >= 0`：放在 `bbox_ux + margin`。
6. 非外推轴坐标使用 anchor 坐标 clamp 到 bbox 的对应范围内。
7. `margin` 可取 `max(2.0, 0.03 * min(width, height))`，并允许绘图坐标轴给 die 边界外留少量 padding，避免 marker 被裁剪。

请实现：

def compute_grid_visual_pos(root, nodes, leaf_info, source_pos, width, height):
    visual_pos = {}
    margin = max(2.0, 0.03 * min(width, height))
    def cluster_top_anchor(node):
        if node.parent >= 0 and node.parent in nodes:
            parent = nodes[node.parent]
            return parent.cx, parent.cy
        return source_pos
    def cluster_top_visual_pos(node):
        ax, ay = cluster_top_anchor(node)
        cx = (node.bbox_lx + node.bbox_ux) / 2.0
        cy = (node.bbox_ly + node.bbox_uy) / 2.0
        dx = ax - cx
        dy = ay - cy
        if abs(dx) >= abs(dy):
            y = min(max(ay, node.bbox_ly), node.bbox_uy)
            x = node.bbox_lx - margin if dx < 0 else node.bbox_ux + margin
        else:
            x = min(max(ax, node.bbox_lx), node.bbox_ux)
            y = node.bbox_ly - margin if dy < 0 else node.bbox_uy + margin
        return x, y
    def dfs(u):
        node = nodes[u]
        if node.is_leaf:
            x, y = get_leaf_pos(u, leaf_info)
            visual_pos[u] = (float(x), float(y))
            return visual_pos[u]
        child_ids = [child_id for child_id in (node.left, node.right) if child_id >= 0]
        child_positions = [dfs(child_id) for child_id in child_ids]
        ux = sum(p[0] for p in child_positions) / len(child_positions)
        uy = sum(p[1] for p in child_positions) / len(child_positions)
        if node.node_kind == "CLUSTER_TOP":
            visual_pos[u] = cluster_top_visual_pos(node)
        else:
            visual_pos[u] = (ux, uy)
        return visual_pos[u]
    dfs(root)
    return visual_pos

说明：

* 这个 visual_pos 只用于右侧 grid 可视化。
* 它不会修改 nodes[u].cx/cy。
* 它不影响 max_est_skew 计算。
* 右侧 internal node 标记必须画在 internal node 的 visual_pos[u] 上，并按 `node_kind` 使用对应样式。
* 除 `CLUSTER_TOP` 外，其余 node 的连接方式不变。

⸻

右侧 edge 绘制方法

请实现：

def draw_grid_tree(ax, root, nodes, leaf_info, edges, width, height, source_pos):
    visual_pos = compute_grid_visual_pos(root, nodes, leaf_info, source_pos, width, height)
    def draw_branch(u):
        node = nodes[u]
        if node.is_leaf:
            return
        P = visual_pos[u]
        child_ids = [child_id for child_id in (node.left, node.right) if child_id >= 0]
        for child_id in child_ids:
            C = visual_pos[child_id]
            ax.plot([P[0], C[0]], [P[1], C[1]],
                    color="black", linewidth=2.8, zorder=1)
            draw_branch(child_id)
    # 先画 tree branch
    draw_branch(root)
    # 再画 source 到 root 的连接
    root_pos = visual_pos[root]
    ax.plot([source_pos[0], root_pos[0]], [source_pos[1], root_pos[1]],
            color="black", linewidth=2.8, zorder=1)
    # 之后再画 nodes
    ...

说明：

* binary node 的 tap 点 P 就是两个 child 的中点。
* unary wrapper 的 tap 点 P 等于唯一 child 的位置，因此该 wrapper 到 child 的线段长度可能为 0。
* `CLUSTER_TOP` 是唯一特殊 node：它的 tap 点 P 在 cluster bbox 外侧，黑线从这个外侧 P 直接连到 child visual_pos。
* 画线时直接 P -> child；binary node 会画 P -> L 和 P -> R。
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
    "CLUSTER_BRIDGE": {"face": "#d6bcfa", "edge": "#6b46c1", "marker": "P"},
    "CLUSTER_TOP": {"face": "#fbd38d", "edge": "#c05621", "marker": "*"},
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

for kind in ("CLUSTER_INTERNAL", "CLUSTER_ACCESS", "CLUSTER_BRIDGE", "CLUSTER_TOP", "GLOBAL"):
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

draw_topology(ax1, root, nodes, leaf_info, edges, source_pos)
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

* 左侧：普通二叉树 topology；parent-child 是直线；node marker shape/color 与右侧一致；internal node 显示 node id；leaf 显示 sink_id；root 保持自身 kind 样式并标注 ROOT。
* 左侧：source 在 root 上方显示为红色三角形 `SRC`，并连接到 root。
* 右侧：die grid 上的几何树；source 是红色三角形；其他 node 按 `node_kind` 显示颜色/marker。
* 右侧每个 binary internal tap marker 必须在它的两个 child 的几何中点。
* 右侧每个 unary wrapper marker 必须与它的唯一 child visual_pos 重合。
* 右侧底层两个 sink 合并时，tap marker 应正好在两个 sink 的连线中点。
* 右侧更高层合并时，tap marker 应在两个 child tap/sink 的连线中点。
* 右侧黑色粗线直接从 tap marker 连到两个 child。
* 右侧不能是 parent-child 两点直连。
* 右侧不能是每条 edge 各自独立的 L 形折线。
* 右侧不能使用额外的 B 点或 P -> B -> L/R 画法。
* 右侧不能出现红色 tap 点和黑线分叉/连接点明显分离的情况。
* 终端输出 max_est_skew = ...。
* figure 标题显示 max_est_skew=...。
