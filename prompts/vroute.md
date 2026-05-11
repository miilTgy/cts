

# vroute.md — Routing Result Visualization Script 提示词

请实现一个 Python 可视化脚本，用于查看 `router` 输出的 Manhattan polyline routing result。

脚本读取：

```text
route/sample<k>_route.txt
```

并额外读取对应 loc debug 文件、topology 文件和原始 sample 文件：

```text
loc/sample<k>_loc.txt
tree/sample<k>_vtree.txt
samples/sample<k>.txt
```

然后画出：

1. router 实际输出的每条 routed edge polyline；
2. locer 输出的所有 node 最终 loc；
3. node 的颜色、形状、大小必须与 `vloc.py` 保持一致；
4. 支持显示 route edge id、node id/class、失败 edge；
5. 显示 source，并保留 `EDGE SRC <child>` 对应的 source-to-child route；
6. 输出图片到：

```text
route/fig/sample<k>_route.png
```

---

# 1. 输入输出

命令行：

```bash
python3 vroute.py 3
```

脚本需要自动推导：

```text
sample_name = sample3
route file  = route/sample3_route.txt
loc file    = loc/sample3_loc.txt
tree file   = tree/sample3_vtree.txt
sample file = samples/sample3.txt
out file    = route/fig/sample3_route.png
```

输入只接受 `sample_id`（整数），不需要兼容旧版输入格式（如 `sample1` 或 `route/sample1_route.txt`）。

写图前确保 `route/fig/` 目录存在。

---

# 2. 解析 route/sample<k>_route.txt

route debug 文件推荐格式：

```text
# ROUTER_DEBUG_ROUTE v1
# valid=<0/1>
# num_edges=<E>
# columns: edge_id parent child parent_class child_class policy selected_shape parent_exit_dir child_entry_dir score wirelength bends pattern_candidate_count maze_candidate_count legal_candidate_count failure_reason point_count points...
edge <edge_id> <parent> <child> <parent_class> <child_class> <policy> <selected_shape> <parent_exit_dir> <child_entry_dir> <score> <wirelength> <bends> <pattern_candidate_count> <maze_candidate_count> <legal_candidate_count> <failure_reason> <point_count> <x0> <y0> <x1> <y1> ...
```

解析规则：

```text
ignore empty lines
ignore lines starting with '#'
parse lines starting with 'edge'
```

字段位置：

```text
0:  edge literal
1:  edge_id
2:  parent
3:  child
4:  parent_class
5:  child_class
6:  policy
7:  selected_shape
8:  parent_exit_dir
9:  child_entry_dir
10: score
11: wirelength
12: bends
13: pattern_candidate_count
14: maze_candidate_count
15: legal_candidate_count
16: failure_reason
17: point_count
18+: flattened x/y point list
```

注意：

- `x/y` 可能是整数，也可能是浮点数。
- `selected_shape` 可以是 `I/L/Z/MAZE/FAILED`。
- 失败 edge 也需要解析，但可能没有完整 points。
- 如果 `point_count <= 1` 或点数不足，保留该 edge 的 metadata，但绘图时不画 polyline，并 warning。
- 缺失后续 debug 字段时，不要直接崩溃；尽量解析 edge_id、parent、child、selected_shape 和已有 points。

建议数据结构：

```python
@dataclass
class RouteEdge:
    edge_id: int
    parent: int
    child: int
    parent_class: str
    child_class: str
    policy: str
    selected_shape: str
    parent_exit_dir: str
    child_entry_dir: str
    score: float = 0.0
    wirelength: float = 0.0
    bends: int = 0
    pattern_candidate_count: int = 0
    maze_candidate_count: int = 0
    legal_candidate_count: int = 0
    failure_reason: str = ""
    points: list[tuple[float, float]] = field(default_factory=list)
```

---

# 3. 解析 loc/sample<k>_loc.txt

必须读取 loc 文件来获得 node 坐标和 node class。node 样式必须与 `vloc.py` 一致。

loc 文件格式示例：

```text
# LOCER_DEBUG_LOC v1
node 0 top -1 38 60 TOP_CONGESTION_AWARE -1 1 8 65 1365.92 0 0.35461 0 26 7 50 52 2 2.2
```

解析规则：

```text
ignore empty lines
ignore lines starting with '#'
parse lines starting with 'node'
```

核心字段：

```text
0: node literal
1: node_id
2: class
3: cluster_id
4: x
5: y
6: loc_mode
7: parent
8: left
9: right
```

建议数据结构：

```python
@dataclass
class LocNode:
    node_id: int
    cls: str
    cluster_id: int
    x: float
    y: float
    loc_mode: str = ""
    parent: int = -1
    left: int = -1
    right: int = -1
    skew_to_node: float = 0.0
```

如果 loc 文件缺失或某个 route endpoint 不在 loc 文件里：

```text
[WARN] endpoint node <id> missing from loc file
```

source 作为虚拟 node 处理，坐标优先从 `samples/sample<k>.txt` 的 source 信息读取；若 sample 文件没有，则尝试从 tree 文件读取；若仍没有，则从 route polyline 中 source 相关 edge 的首点推断。

---

# 4. 解析 tree/sample<k>_vtree.txt 与 sample 文件

`vroute.py` 主要依赖 route 文件画 polyline，因此 tree 文件不是画线的 source of truth。但仍应读取 tree/sample 文件用于：

1. 获取 source 坐标；
2. 校验 route 中的 parent-child 是否属于 topology edge；
3. 在 `--show-unrouted-topology` 时，用淡灰色虚线画未 route 的 topology edge。

要求脚本支持宽松解析 tree 文件，兼容常见 debug tree 格式。

解析目标：

```python
topology_edges: set[tuple[int, int]]
source_pos: tuple[float, float] | None
```

支持格式：

```text
node <id> ... parent <p> left <l> right <r> ...
node <id> ... parent=<p> left=<l> right=<r> ...
EDGE <parent> <child>
EDGE SRC <child>
```

如果 tree 文件缺失或解析失败，不影响 route polyline 绘制，只打印 warning。

---

# 5. 可视化要求

使用 `matplotlib`，不要使用 seaborn。

画布：

```python
fig, ax = plt.subplots(figsize=(11, 9))
ax.set_aspect('equal', adjustable='box')
ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.4)
```

绘图顺序：

```text
1. 可选 cluster bbox
2. 可选未 route topology edge 灰色虚线
3. 已 route polyline
4. 失败 edge 标记
5. node markers
6. node labels / edge labels / legend
```

---

# 6. Route polyline 样式

route polyline 是本脚本最重要的显示内容。必须按 route 文件中的 `points` 逐段画真实折线，不要重新用 parent/child loc 直连。

基本样式：

```text
LocalClusterPatternOnly        : linewidth=2.0, linestyle='-'
ExternalAccessPatternThenMaze  : linewidth=2.6, linestyle='-'
GlobalPatternThenMaze          : linewidth=3.0, linestyle='-'
unknown policy                 : linewidth=2.0, linestyle='-'
```

颜色建议：

```text
I-shape    : '#4a5568'
L-shape    : '#2b6cb0'
Z-shape    : '#805ad5'
MAZE       : '#dd6b20'
FAILED     : '#e53e3e'
```

如果不想区分形状，也至少保证：

```text
successful route: black or dark color solid line
failed route: red dashed line or red cross marker
```

对于 `FAILED` edge：

- 若 route 文件中仍有 points，则用红色虚线画出已有 partial path。
- 若没有 points，但 parent/child loc 存在，则在 parent-child loc 中点画红色 `x`，并可用淡红色虚线直连辅助定位。
- 若 parent/child loc 缺失，则只在 stdout warning。

---

# 7. Node 样式：必须与 vloc 保持一致

node 的 marker、颜色、大小必须与 `vloc.py` / `vloc.md` 中 location view 一致。

```python
STYLE = {
    'sink':     dict(marker='o', face='#202020', edge='#202020', size=45, linewidth=1.2),
    'internal': dict(marker='o', face='#ffe08a', edge='#b7791f', size=55, linewidth=2.0),
    'access':   dict(marker='s', face='#90cdf4', edge='#2b6cb0', size=55, linewidth=2.0),
    'bridge':   dict(marker='P', face='#d6bcfa', edge='#6b46c1', size=55, linewidth=2.0),
    'top':      dict(marker='*', face='#fbd38d', edge='#c05621', size=55, linewidth=2.0),
    'global':   dict(marker='D', face='#c6f6d5', edge='#2f855a', size=55, linewidth=2.0),
    'source':   dict(marker='^', face='red', edge='darkred', size=120, linewidth=1.5),
    'unknown':  dict(marker='o', face='white', edge='black', size=55, linewidth=2.0),
}
```

绘制 node 时：

```python
ax.scatter([x], [y], marker=style['marker'], s=style['size'],
           facecolors=style['face'], edgecolors=style['edge'],
           linewidths=style['linewidth'], zorder=5)
```

必须有 node class legend。route shape/policy legend 可选，但建议加入。

---

# 8. 标签显示

默认显示 node id：

```text
label = node_id
```

支持可选参数：

```bash
python3 vroute.py 3 --label none
python3 vroute.py 3 --label id
python3 vroute.py 3 --label class
python3 vroute.py 3 --label id_class
python3 vroute.py 3 --label skew
```

含义：

```text
none     : 不显示 node 文字
id       : 显示 node id
class    : 显示 class
id_class : 显示 id:class
skew     : 显示 id 和 skew_to_node，如果 loc 文件中有该字段
```

文字位置稍微偏移，避免盖住 marker：

```python
ax.text(x + 0.5, y + 0.5, label, fontsize=7, zorder=6)
```

支持 route edge label：

```bash
python3 vroute.py 3 --edge-label none
python3 vroute.py 3 --edge-label id
python3 vroute.py 3 --edge-label shape
python3 vroute.py 3 --edge-label policy
python3 vroute.py 3 --edge-label failure
```

edge label 画在 polyline 的中间 segment 附近：

```text
id      : edge_id
shape   : I/L/Z/MAZE/FAILED
policy  : Local/External/Global 简写
failure : failed edge 显示 failure_reason，成功 edge 不显示
```

---

# 9. cluster bbox 可选绘制

如果 loc 文件里有 cluster_id 和 sink 坐标，可以从每个 cluster 的 sink nodes 推导 bbox，并画出来。

命令行参数：

```bash
python3 vroute.py 3 --bbox
```

bbox 计算：

```text
for each cluster_id >= 0:
    collect nodes where cls == 'sink'
    bbox = min/max x/y
```

画法：

```python
Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, fill=False, linestyle='--', linewidth=1.0, alpha=0.5)
ax.text(xmin, ymax, f'C{cluster_id}', fontsize=8)
```

bbox 用于 debug，不能影响 route/node 绘制。

---

# 10. 可选显示未 route topology edge

命令行参数：

```bash
python3 vroute.py 3 --show-unrouted-topology
```

用途：检查 router 是否漏 route 某些 topology edge。

实现：

```text
routed_pairs = {(edge.parent, edge.child) for edge in route_edges if edge.selected_shape != 'FAILED'}
for each topology edge:
    if topology edge not in routed_pairs:
        draw light gray dashed straight line between endpoint locs
```

注意：

- 这只是 debug reference，不是实际 route。
- 如果 endpoint loc 缺失，跳过并 warning。

---

# 11. 输出信息

脚本运行时打印：

```text
[VROUTE] route file: ...
[VROUTE] loc file: ...
[VROUTE] tree file: ...
[VROUTE] parsed nodes: N
[VROUTE] parsed routes: E
[VROUTE] routed edges: R
[VROUTE] failed edges: F
[VROUTE] output: route/fig/sample<k>_route.png
```

如果 route 文件缺失：

```text
[ERROR] missing route file: route/sample<k>_route.txt
```

如果某些 route point count 不匹配：

```text
[WARN] edge <edge_id> point_count mismatch, parsed <n> points
```

如果某些 endpoint loc 缺失：

```text
[WARN] skip edge parent->child because endpoint loc missing
```

---

# 12. 文件位置

请新增/维护脚本：

```text
scripts/vroute.py
vroute.py
```

其中 `vroute.py` 作为根目录入口，支持：

```bash
python3 vroute.py <sample_id>
```

不要修改 parser、treer、locer、router、writer 等 C++ 代码。

---

# 13. 代码质量要求

- 使用 Python 3。
- 使用 `argparse`。
- 使用 `dataclasses.dataclass`。
- 使用 `matplotlib`，不要使用 seaborn。
- 不要依赖工作目录以外的绝对路径。
- 输出图片目录固定为 `route/fig/`，不要输出到项目根目录的 `fig/`。
- 解析函数拆分清楚：

```python
parse_args()
infer_paths(sample_id)
parse_route_file(path)
parse_loc_file(path)
parse_sample_source(path)
parse_tree_edges_and_source(path)
compute_cluster_bboxes(nodes)
plot_routes(nodes, route_edges, topology_edges, bboxes, source_pos, out_path, args)
main()
```

- 绘图函数中不要重新生成 routing；只能画 route debug 文件中已有的 polyline。
- 缺失辅助文件时尽量 degrade gracefully；但 route 文件缺失应直接报错退出。