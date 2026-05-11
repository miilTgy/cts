# vloc.md — Location Assignment Visualization Script 提示词

请实现一个 Python 可视化脚本，用于查看 `locer` 输出的 node location assignment。

脚本读取：

```text
loc/sample<k>_loc.txt
```

并额外读取对应 topology 文件和原始 sample 文件：

```text
tree/sample<k>_vtree.txt
samples/sample<k>.txt
```

然后画出：

1. 所有 node 的最终 loc；
2. 不同 class node 用不同颜色/形状区分；
3. 按 tree topology 中的 parent-child 关系连接 node；
4. 可选显示 node id 和 class；
5. 显示 source，并保留 `EDGE SRC <child>` 到 global/top root 的连线；
6. 输出图片到：

```text
loc/fig/sample<k>_loc.png
```

---

# 1. 输入输出

命令行：

```bash
python3 vloc.py 3
```

脚本需要自动推导：

```text
sample_name = sample3
loc file    = loc/sample3_loc.txt
tree file   = tree/sample3_vtree.txt
sample file = samples/sample3.txt
out file    = loc/fig/sample3_loc.png
```

输入只接受 `sample_id`（整数），不需要兼容旧版输入格式（如 `sample1` 或 `loc/sample1_loc.txt`）。

写图前确保 `loc/fig/` 目录存在。

---

# 2. 解析 loc/sample<k>_loc.txt

loc 文件格式示例：

```text
# LOCER_DEBUG_LOC v1
# valid=1
# num_nodes=28
# columns: node_id class cluster_id x y loc_mode parent left right candidate_count loc_score inside_related_bbox congestion_penalty lshape_penalty wire_est_to_parent sink_delay_count min_sink_delay max_sink_delay skew_to_node skew_penalty
node 0 top -1 38 60 TOP_CONGESTION_AWARE -1 1 8 65 1365.92 0 0.35461 0 26 7 50 52 2 2.2
```

解析规则：

```text
ignore empty lines
ignore lines starting with '#'
parse lines starting with 'node'
```

字段位置：

```text
0:  node literal
1:  node_id
2:  class
3:  cluster_id
4:  x
5:  y
6:  loc_mode
7:  parent
8:  left
9:  right
10: candidate_count
11: loc_score
12: inside_related_bbox
13: congestion_penalty
14: lshape_penalty
15: wire_est_to_parent
16: sink_delay_count
17: min_sink_delay
18: max_sink_delay
19: skew_to_node
20: skew_penalty
```

注意：

- `x/y` 可能是整数，也可能是浮点数。
- `cluster_id` 可为 `-1`。
- parent/left/right 使用原始 topology node id。
- 缺失后续 debug 字段时，不要直接崩溃；尽量解析前 10 个核心字段即可。

建议数据结构：

```python
@dataclass
class LocNode:
    node_id: int
    cls: str
    cluster_id: int
    x: float
    y: float
    loc_mode: str
    parent: int
    left: int
    right: int
    candidate_count: int = 0
    loc_score: float = 0.0
    inside_related_bbox: int = 0
    congestion_penalty: float = 0.0
    lshape_penalty: float = 0.0
    wire_est_to_parent: float = 0.0
    sink_delay_count: int = 0
    min_sink_delay: float = 0.0
    max_sink_delay: float = 0.0
    skew_to_node: float = 0.0
    skew_penalty: float = 0.0
```

---

# 3. 解析 tree/sample<k>_vtree.txt

必须读取 tree 文件来确认 topology。不要只依赖 loc 文件中的 parent/left/right，因为 loc 文件是 debug 输出，tree 文件是 topology source of truth。

要求脚本支持宽松解析，兼容常见 debug tree 格式。

解析目标：

```python
edges: list[tuple[int, int]]  # (parent, child), SRC parent 可映射为虚拟 source id
source_pos: tuple[float, float] | None
```

优先支持以下格式之一：

```text
node <id> ... parent <p> left <l> right <r> ...
node <id> ... parent=<p> left=<l> right=<r> ...
node <id> <class> parent=<p> left=<l> right=<r>
```

也支持从 loc 文件 fallback：

```text
if tree file missing or no edges parsed:
    for each loc node:
        if parent < 0: add edge(SRC, node_id)
        if left >= 0:  add edge(node_id, left)
        if right >= 0: add edge(node_id, right)
```

但必须打印 warning：

```text
[WARN] failed to parse tree/sample<k>_vtree.txt, fallback to loc parent/left/right fields
```

树边要求：

- 只画 parent-child edge。
- `EDGE SRC <child>` 不要跳过；应该生成虚拟 source node 后画 source-to-child edge。
- source 坐标优先从 `samples/sample<k>.txt` 的 `SOURCE <id> <x> <y>` 读取，若不存在则从 `tree/sample<k>_vtree.txt` 的 `SOURCE <x> <y>` 读取。
- 若 edge endpoint 不在 loc 文件中，且不是 source 虚拟节点，跳过并 warning。
- 重复 edge 去重。

---

# 4. 可视化要求

使用 `matplotlib`，不要使用 seaborn。

画布：

```python
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect('equal', adjustable='box')
ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.4)
```

先画 edge，再画 node。

edge：

```text
parent-child edge 与 vtree.py grid 视图一致，用黑色线
outer edge 可以稍粗，推荐 linewidth=2.8
DME cluster-internal edge 可以稍细，推荐 linewidth=2.0
```

edge 类型判断：

```text
if parent.cls in {source, global, bridge, top} or child.cls in {source, global, bridge, top}:
    outer edge
else:
    internal edge
```

node 样式（必须与 `vtree.py` 的 grid 视图一致）：

```text
sink     : marker='o', face='#202020', edge='#202020'
internal : marker='o', face='#ffe08a', edge='#b7791f'
access   : marker='s', face='#90cdf4', edge='#2b6cb0'
bridge   : marker='P', face='#d6bcfa', edge='#6b46c1'
top      : marker='*', face='#fbd38d', edge='#c05621'
global   : marker='D', face='#c6f6d5', edge='#2f855a'
source   : marker='^', face='red', edge='darkred'
unknown  : marker='o', face='white', edge='black'
```

大小也与 `vtree.py` grid 对齐：

```text
sink=45
其他 class=55
source=120
```

必须有 legend。

建议 mapping：

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

---

# 5. 标签显示

默认显示 node id：

```text
label = node_id
```

支持可选参数：

```bash
python3 vloc.py 3 --label none
python3 vloc.py 3 --label id
python3 vloc.py 3 --label class
python3 vloc.py 3 --label id_class
python3 vloc.py 3 --label skew
```

含义：

```text
none     : 不显示文字
id       : 显示 node id
class    : 显示 class
id_class : 显示 id:class
skew     : 显示 id 和 skew_to_node
```

文字位置稍微偏移，避免盖住 marker：

```python
ax.text(x + 0.5, y + 0.5, label, fontsize=7)
```

---

# 6. cluster bbox 可选绘制

如果 loc 文件里有 cluster_id 和 sink 坐标，可以从每个 cluster 的 sink nodes 推导 bbox，并画出来。

命令行参数：

```bash
python3 vloc.py 3 --bbox
```

bbox 计算：

```text
for each cluster_id >= 0:
    collect nodes where cls == 'sink'
    bbox = min/max x/y
```

画法：

```python
Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, fill=False, linestyle='--')
ax.text(xmin, ymax, f'C{cluster_id}', fontsize=8)
```

bbox 用于 debug，不能影响 node/edge 绘制。

---

# 7. 输出信息

脚本运行时打印：

```text
[VLOC] loc file: ...
[VLOC] tree file: ...
[VLOC] parsed nodes: N
[VLOC] parsed edges: E
[VLOC] output: loc/fig/sample<k>_loc.png
```

如果 fallback 到 loc 文件 parent/left/right：

```text
[WARN] failed to parse tree file; fallback to loc file edges
```

如果某些 edge endpoint 缺失：

```text
[WARN] skip edge parent->child because endpoint loc missing
```

---

# 8. 文件位置

请新增/维护脚本：

```text
scripts/vloc.py
vloc.py
```

其中 `vloc.py` 作为根目录入口，支持 `python3 vloc.py <sample_id>`。

不要修改 parser、treer、locer、writer 等 C++ 代码。

---

# 9. 代码质量要求

- 使用 Python 3。
- 使用 `argparse`。
- 使用 `dataclasses.dataclass`。
- 解析函数拆分清楚：

```python
parse_args()
infer_paths(sample_id)
parse_loc_file(path)
parse_tree_edges(path)
fallback_edges_from_loc(nodes)
compute_cluster_bboxes(nodes)
plot_loc(nodes, edges, bboxes, out_path, label_mode)
main()
```

- 不要依赖工作目录以外的绝对路径。
- 输出图片目录固定为 `loc/fig/`，不要输出到项目根目录的 `fig/`。
