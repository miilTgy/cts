import os
import re
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

plt = None


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


LeafInfo = Tuple[int, str, int, int]
GeoPos = Optional[Tuple[float, float]]
Edge = Tuple[Optional[int], int]

KIND_STYLES = {
    "SINK": {
        "face": "#202020",
        "edge": "#202020",
        "text": "white",
        "grid_marker": "o",
        "label": "Sink",
    },
    "CLUSTER_INTERNAL": {
        "face": "#ffe08a",
        "edge": "#b7791f",
        "text": "#1f2933",
        "grid_marker": "o",
        "label": "Cluster internal",
    },
    "CLUSTER_ACCESS": {
        "face": "#90cdf4",
        "edge": "#2b6cb0",
        "text": "#102a43",
        "grid_marker": "s",
        "label": "Cluster access",
    },
    "CLUSTER_BRIDGE": {
        "face": "#d6bcfa",
        "edge": "#6b46c1",
        "text": "#201047",
        "grid_marker": "P",
        "label": "Cluster bridge",
    },
    "CLUSTER_TOP": {
        "face": "#fbd38d",
        "edge": "#c05621",
        "text": "#3b1f0b",
        "grid_marker": "*",
        "label": "Cluster top",
    },
    "GLOBAL": {
        "face": "#c6f6d5",
        "edge": "#2f855a",
        "text": "#102a43",
        "grid_marker": "D",
        "label": "Global",
    },
    "UNKNOWN": {
        "face": "white",
        "edge": "black",
        "text": "black",
        "grid_marker": "o",
        "label": "Unknown",
    },
}


def normalize_node_kind(raw: str, is_leaf: bool) -> str:
    kind = raw.upper()
    if kind in KIND_STYLES and kind != "UNKNOWN":
        return kind
    return "SINK" if is_leaf else "UNKNOWN"


def kind_style(node: Node) -> Dict[str, str]:
    return KIND_STYLES.get(node.node_kind, KIND_STYLES["UNKNOWN"])


def fail(message: str) -> None:
    print(f"Error: {message}", file=sys.stderr)
    sys.exit(1)


def warn(message: str) -> None:
    print(f"Warning: {message}", file=sys.stderr)


def parse_int(token: str, context: str) -> int:
    try:
        return int(token)
    except ValueError:
        fail(f"expected integer for {context}, got '{token}'")


def parse_float(token: str, context: str) -> float:
    try:
        return float(token)
    except ValueError:
        fail(f"expected number for {context}, got '{token}'")


def parse_tree_file(path: str):
    nodes: Dict[int, Node] = {}
    leaf_info: Dict[int, LeafInfo] = {}
    edges: List[Edge] = []
    source_children: List[int] = []
    root = -1
    num_nodes = None
    num_sinks = None
    tree_valid_seen = False

    with open(path, "r") as f:
        for line_no, line in enumerate(f, start=1):
            parts = line.split()
            if not parts:
                continue

            kind = parts[0]
            if kind == "TREE_VALID":
                if len(parts) != 2:
                    fail(f"{path}:{line_no}: TREE_VALID expects 1 value")
                tree_valid_seen = True
                if parse_int(parts[1], "TREE_VALID") != 1:
                    fail(f"{path}:{line_no}: TREE_VALID must be 1")
            elif kind == "ROOT":
                if len(parts) != 2:
                    fail(f"{path}:{line_no}: ROOT expects 1 value")
                root = parse_int(parts[1], "ROOT")
            elif kind == "NUM_NODES":
                if len(parts) != 2:
                    fail(f"{path}:{line_no}: NUM_NODES expects 1 value")
                num_nodes = parse_int(parts[1], "NUM_NODES")
            elif kind == "SOURCE":
                if len(parts) != 3:
                    fail(f"{path}:{line_no}: SOURCE expects x and y")
            elif kind == "NUM_SINKS":
                if len(parts) != 2:
                    fail(f"{path}:{line_no}: NUM_SINKS expects 1 value")
                num_sinks = parse_int(parts[1], "NUM_SINKS")
            elif kind == "NODE":
                if len(parts) != 20:
                    fail(
                        f"{path}:{line_no}: NODE expects 19 values after NODE, "
                        f"got {len(parts) - 1}"
                    )
                node_id = parse_int(parts[1], "NODE id")
                region_lx = parse_int(parts[14], "NODE region_lx")
                region_ly = parse_int(parts[15], "NODE region_ly")
                region_ux = parse_int(parts[16], "NODE region_ux")
                region_uy = parse_int(parts[17], "NODE region_uy")
                subtree_skew_to_node = parse_float(parts[18], "NODE subtree_skew_to_node")
                raw_kind = parts[19]
                is_leaf = parse_int(parts[5], "NODE is_leaf") != 0
                node = Node(
                    id=node_id,
                    parent=parse_int(parts[2], "NODE parent"),
                    left=parse_int(parts[3], "NODE left"),
                    right=parse_int(parts[4], "NODE right"),
                    is_leaf=is_leaf,
                    sink_index=parse_int(parts[6], "NODE sink_index"),
                    sink_count=parse_int(parts[7], "NODE sink_count"),
                    cx=parse_float(parts[8], "NODE cx"),
                    cy=parse_float(parts[9], "NODE cy"),
                    bbox_lx=parse_int(parts[10], "NODE bbox_lx"),
                    bbox_ly=parse_int(parts[11], "NODE bbox_ly"),
                    bbox_ux=parse_int(parts[12], "NODE bbox_ux"),
                    bbox_uy=parse_int(parts[13], "NODE bbox_uy"),
                    region_lx=region_lx,
                    region_ly=region_ly,
                    region_ux=region_ux,
                    region_uy=region_uy,
                    subtree_skew_to_node=subtree_skew_to_node,
                    node_kind=normalize_node_kind(raw_kind, is_leaf),
                )
                nodes[node_id] = node
            elif kind == "LEAF":
                if len(parts) != 6:
                    fail(f"{path}:{line_no}: LEAF expects 5 values after LEAF")
                node_id = parse_int(parts[1], "LEAF node_id")
                sink_index = parse_int(parts[2], "LEAF sink_index")
                sink_id = parts[3]
                x = parse_int(parts[4], "LEAF x")
                y = parse_int(parts[5], "LEAF y")
                leaf_info[node_id] = (sink_index, sink_id, x, y)
            elif kind == "EDGE":
                if len(parts) != 3:
                    fail(f"{path}:{line_no}: EDGE expects parent_id and child_id")
                parent_id = None if parts[1] == "SRC" else parse_int(parts[1], "EDGE parent_id")
                child_id = parse_int(parts[2], "EDGE child_id")
                if parent_id is None:
                    source_children.append(child_id)
                edges.append((parent_id, child_id))

    if not tree_valid_seen:
        fail(f"{path}: missing TREE_VALID line")
    if root < 0 and nodes and not source_children:
        fail(f"{path}: missing ROOT line")
    if root < 0 and not nodes:
        return root, nodes, leaf_info, edges, source_children
    if root >= 0 and root not in nodes:
        fail(f"{path}: ROOT node {root} is not present in NODE records")

    if num_nodes is not None and num_nodes != len(nodes):
        warn(f"{path}: NUM_NODES is {num_nodes}, but parsed {len(nodes)} NODE lines")
    if num_sinks is not None and num_sinks != len(leaf_info):
        warn(f"{path}: NUM_SINKS is {num_sinks}, but parsed {len(leaf_info)} LEAF lines")

    for parent_id, child_id in edges:
        if parent_id is not None and parent_id not in nodes:
            warn(f"{path}: EDGE references missing parent node {parent_id}")
        if child_id not in nodes:
            warn(f"{path}: EDGE references missing child node {child_id}")

    return root, nodes, leaf_info, edges, source_children


def parse_sample_file(path: str) -> Tuple[int, int, float, float]:
    width = 0
    height = 0
    source_x = 0.0
    source_y = 0.0
    has_die = False
    has_source = False

    with open(path, "r") as f:
        for line_no, line in enumerate(f, start=1):
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "DIE":
                if len(parts) != 3:
                    fail(f"{path}:{line_no}: DIE expects width and height")
                width = parse_int(parts[1], "DIE width")
                height = parse_int(parts[2], "DIE height")
                has_die = True
            elif parts[0] == "SOURCE":
                if len(parts) != 4:
                    fail(f"{path}:{line_no}: SOURCE expects id x y")
                source_x = parse_float(parts[2], "SOURCE x")
                source_y = parse_float(parts[3], "SOURCE y")
                has_source = True

    if not has_die:
        fail(f"{path}: missing DIE <width> <height>")
    if not has_source:
        fail(f"{path}: missing SOURCE line")

    return width, height, source_x, source_y


def get_leaf_pos(node_id: int, leaf_info: Dict[int, LeafInfo]) -> Tuple[float, float]:
    _, _, x, y = leaf_info[node_id]
    return float(x), float(y)


def get_node_debug_pos(
    node_id: int,
    nodes: Dict[int, Node],
    leaf_info: Dict[int, LeafInfo],
) -> GeoPos:
    if node_id in leaf_info:
        return get_leaf_pos(node_id, leaf_info)

    node = nodes.get(node_id)
    if node is None:
        warn(f"missing node {node_id}; no debug position available")
        return None

    return node.cx, node.cy


def manhattan(x1: float, y1: float, x2: float, y2: float) -> float:
    return abs(x1 - x2) + abs(y1 - y2)


def clamp_float(value: float, lo: float, hi: float) -> float:
    return max(lo, min(value, hi))


def compute_max_est_skew(
    root: int,
    nodes: Dict[int, Node],
    leaf_info: Dict[int, LeafInfo],
    source_pos: Tuple[float, float],
    source_children: List[int],
) -> float:
    source_child_set = set(source_children)
    leaf_delays = []
    for leaf_id in leaf_info:
        if leaf_id not in nodes:
            warn(f"skew skipped missing leaf node {leaf_id}")
            continue

        total = 0.0
        current = leaf_id
        while True:
            if current == root and root >= 0:
                root_node = nodes.get(root)
                if root_node is None:
                    warn(f"skew skipped root-to-source segment because root node {root} is missing")
                    break
                total += manhattan(root_node.cx, root_node.cy, source_pos[0], source_pos[1])
                leaf_delays.append(total)
                break

            node = nodes.get(current)
            if node is None:
                warn(f"skew path from leaf {leaf_id} stopped at missing node {current}")
                break

            parent_id = node.parent
            if parent_id < 0:
                if root < 0 and current in source_child_set:
                    current_pos = get_node_debug_pos(current, nodes, leaf_info)
                    if current_pos is None:
                        warn(f"skew path from leaf {leaf_id} stopped at missing source child geometry")
                        break
                    cx, cy = current_pos
                    total += manhattan(cx, cy, source_pos[0], source_pos[1])
                    leaf_delays.append(total)
                else:
                    warn(f"skew path from leaf {leaf_id} stopped before reaching root/source")
                break

            parent = nodes.get(parent_id)
            if parent is None:
                warn(f"skew path from leaf {leaf_id} stopped at missing parent {parent_id}")
                break

            current_pos = get_node_debug_pos(current, nodes, leaf_info)
            parent_pos = get_node_debug_pos(parent_id, nodes, leaf_info)
            if current_pos is None or parent_pos is None:
                warn(f"skew path from leaf {leaf_id} stopped at missing geometry")
                break

            cx, cy = current_pos
            px, py = parent_pos
            total += manhattan(cx, cy, px, py)
            current = parent_id

    if not leaf_delays:
        return 0.0

    return max(leaf_delays) - min(leaf_delays)


def compute_tree_depth(root: int, nodes: Dict[int, Node], source_children: List[int]) -> int:
    def dfs(node_id: int) -> int:
        node = nodes.get(node_id)
        if node is None:
            warn(f"tree depth skipped missing node {node_id}")
            return 0
        if node.is_leaf:
            return 0

        child_depths = []
        for child_id in (node.left, node.right):
            if child_id >= 0:
                child_depths.append(dfs(child_id))
        if not child_depths:
            warn(f"tree depth skipped missing children of node {node_id}")
            return 0

        return 1 + max(child_depths)

    if root >= 0:
        return dfs(root)
    if source_children:
        return 1 + max(dfs(child_id) for child_id in source_children)
    return 0


def compute_topology_layout(
    root: int,
    nodes: Dict[int, Node],
    source_children: List[int],
) -> Dict[int, Tuple[float, float]]:
    pos: Dict[int, Tuple[float, float]] = {}

    next_leaf_x = [0.0]

    def dfs(node_id: int, depth: int) -> float:
        if node_id not in nodes:
            warn(f"topology layout skipped missing node {node_id}")
            return next_leaf_x[0]

        node = nodes[node_id]
        child_ids = []
        if not node.is_leaf:
            if node.left >= 0:
                child_ids.append(node.left)
            if node.right >= 0:
                child_ids.append(node.right)

        if not child_ids:
            x = next_leaf_x[0]
            next_leaf_x[0] += 1.0
        else:
            child_xs = [dfs(child_id, depth + 1) for child_id in child_ids]
            x = sum(child_xs) / len(child_xs)

        pos[node_id] = (x, -float(depth))
        return x

    if root >= 0:
        dfs(root, 0)
    else:
        for child_id in source_children:
            dfs(child_id, 1)
    return pos


def draw_topology(
    ax,
    root: int,
    nodes: Dict[int, Node],
    leaf_info: Dict[int, LeafInfo],
    edges: List[Edge],
    source_pos: Tuple[float, float],
    source_children: List[int],
) -> None:
    pos = compute_topology_layout(root, nodes, source_children)
    num_leaves = max(1, len(leaf_info))
    marker_size = max(18, min(90, 2200 / num_leaves))
    font_size = max(4, min(8, int(150 / max(num_leaves, 1))))
    source_marker_size = max(marker_size * 1.25, 90)

    source_layout_pos: Optional[Tuple[float, float]] = None
    if root >= 0 and root in pos:
        root_x, root_y = pos[root]
        source_layout_pos = (root_x, root_y + 1.0)
        ax.plot(
            [source_layout_pos[0], root_x],
            [source_layout_pos[1], root_y],
            color="black",
            linewidth=1.4,
            zorder=1,
        )
    elif source_children:
        child_positions = [pos[child] for child in source_children if child in pos]
        if child_positions:
            sx = sum(p[0] for p in child_positions) / len(child_positions)
            sy = max(p[1] for p in child_positions) + 1.0
            source_layout_pos = (sx, sy)
            for child_id in source_children:
                if child_id not in pos:
                    warn(f"topology source edge SRC->{child_id} skipped because position is missing")
                    continue
                cx, cy = pos[child_id]
                ax.plot([sx, cx], [sy, cy], color="black", linewidth=1.4, zorder=1)

    for parent_id, child_id in edges:
        if parent_id is None:
            continue
        if parent_id not in pos or child_id not in pos:
            warn(f"topology edge {parent_id}->{child_id} skipped because position is missing")
            continue
        x1, y1 = pos[parent_id]
        x2, y2 = pos[child_id]
        ax.plot([x1, x2], [y1, y2], color="black", linewidth=1.2, zorder=1)

    for node_id, (x, y) in pos.items():
        node = nodes[node_id]
        is_root = node_id == root
        style = kind_style(node)
        ax.scatter(
            [x],
            [y],
            s=marker_size,
            marker=style["grid_marker"],
            facecolors=style["face"],
            edgecolors=style["edge"],
            linewidths=2.0 if is_root else 1.0,
            zorder=3,
        )
        if node.is_leaf:
            label = leaf_info.get(node_id, (node.sink_index, str(node_id), 0, 0))[1]
        else:
            label = str(node_id)
        ax.text(x, y, label, ha="center", va="center", fontsize=font_size, zorder=4)
        if not node.is_leaf:
            ax.text(x, y + 0.23, node.node_kind.replace("_", "\n"),
                    ha="center", va="bottom", fontsize=max(3, font_size * 0.55),
                    color=style["edge"], zorder=5)
        if is_root:
            ax.text(x, y - 0.4, "ROOT", ha="center", va="top",
                    fontsize=font_size * 0.85, fontweight="bold",
                    color="darkorange", zorder=5)

    if source_layout_pos is not None:
        sx, sy = source_layout_pos
        ax.scatter(
            [sx],
            [sy],
            s=source_marker_size,
            marker="^",
            facecolors="red",
            edgecolors="darkred",
            linewidths=1.5,
            zorder=6,
        )
        ax.text(sx, sy + 0.22, "SRC", ha="center", va="bottom",
                fontsize=max(font_size, 7), color="darkred", zorder=7)

    if pos:
        xs = [p[0] for p in pos.values()]
        ys = [p[1] for p in pos.values()]
        if source_layout_pos is not None:
            xs.append(source_layout_pos[0])
            ys.append(source_layout_pos[1])
        ax.set_xlim(min(xs) - 1.0, max(xs) + 1.0)
        ax.set_ylim(min(ys) - 1.0, max(ys) + 1.0)

    if root < 0 and not source_children:
        ax.text(0.5, 0.5, "EMPTY TREE", ha="center", va="center",
                transform=ax.transAxes, fontsize=12, fontweight="bold")
        ax.scatter(
            [0.5],
            [0.78],
            s=source_marker_size,
            marker="^",
            facecolors="red",
            edgecolors="darkred",
            linewidths=1.5,
            transform=ax.transAxes,
            zorder=6,
        )
        ax.text(0.5, 0.86, "SRC", ha="center", va="bottom",
                fontsize=10, color="darkred", transform=ax.transAxes, zorder=7)

    ax.set_title("Binary Topology")
    add_kind_legend(ax)
    ax.set_aspect("auto")
    ax.axis("off")


def add_kind_legend(ax) -> None:
    handles = []
    labels = []
    for kind in (
        "SINK",
        "CLUSTER_INTERNAL",
        "CLUSTER_ACCESS",
        "CLUSTER_BRIDGE",
        "CLUSTER_TOP",
        "GLOBAL",
    ):
        style = KIND_STYLES[kind]
        handle = ax.scatter(
            [],
            [],
            s=55,
            marker=style["grid_marker"],
            facecolors=style["face"],
            edgecolors=style["edge"],
            linewidths=1.2,
        )
        handles.append(handle)
        labels.append(style["label"])
    ax.legend(handles, labels, loc="upper right", fontsize=7, frameon=True)


def compute_grid_visual_pos(
    root: int,
    nodes: Dict[int, Node],
    leaf_info: Dict[int, LeafInfo],
    source_pos: Tuple[float, float],
    width: int,
    height: int,
    source_children: List[int],
) -> Dict[int, Tuple[float, float]]:
    visual_pos: Dict[int, Tuple[float, float]] = {}
    if root < 0 and not source_children:
        return visual_pos

    visiting = set()
    outside_margin = max(2.0, 0.03 * float(min(width, height)))

    def cluster_top_external_anchor(node: Node) -> Tuple[float, float]:
        if node.parent >= 0 and node.parent in nodes:
            parent = nodes[node.parent]
            return parent.cx, parent.cy
        return source_pos

    def cluster_top_visual_pos(node: Node) -> Tuple[float, float]:
        ax, ay = cluster_top_external_anchor(node)
        center_x = (float(node.bbox_lx) + float(node.bbox_ux)) / 2.0
        center_y = (float(node.bbox_ly) + float(node.bbox_uy)) / 2.0
        dx = ax - center_x
        dy = ay - center_y

        if abs(dx) >= abs(dy):
            y = clamp_float(ay, float(node.bbox_ly), float(node.bbox_uy))
            if dx < 0:
                return float(node.bbox_lx) - outside_margin, y
            return float(node.bbox_ux) + outside_margin, y

        x = clamp_float(ax, float(node.bbox_lx), float(node.bbox_ux))
        if dy < 0:
            return x, float(node.bbox_ly) - outside_margin
        return x, float(node.bbox_uy) + outside_margin

    def dfs(node_id: int) -> GeoPos:
        if node_id in visual_pos:
            return visual_pos[node_id]
        if node_id in visiting:
            warn(f"grid visual position skipped recursive cycle at node {node_id}")
            return None

        node = nodes.get(node_id)
        if node is None:
            warn(f"grid visual position skipped missing node {node_id}")
            return None

        visiting.add(node_id)
        if node.is_leaf:
            if node_id not in leaf_info:
                warn(f"grid visual position skipped leaf {node_id} without LEAF record")
                visiting.remove(node_id)
                return None
            visual_pos[node_id] = get_leaf_pos(node_id, leaf_info)
        else:
            child_ids = [child_id for child_id in (node.left, node.right) if child_id >= 0]
            if not child_ids:
                warn(f"grid visual position skipped incomplete children of node {node_id}")
                visiting.remove(node_id)
                return None

            child_positions = [dfs(child_id) for child_id in child_ids]
            if any(pos is None for pos in child_positions):
                warn(f"grid visual position skipped node {node_id} because child position is missing")
                visiting.remove(node_id)
                return None

            ux = sum(pos[0] for pos in child_positions if pos is not None) / len(child_positions)
            uy = sum(pos[1] for pos in child_positions if pos is not None) / len(child_positions)
            if node.node_kind == "CLUSTER_TOP":
                visual_pos[node_id] = cluster_top_visual_pos(node)
            else:
                visual_pos[node_id] = (ux, uy)

        visiting.remove(node_id)
        return visual_pos[node_id]

    if root >= 0:
        dfs(root)
    else:
        for child_id in source_children:
            dfs(child_id)
    return visual_pos


def draw_grid_tree(
    ax,
    root: int,
    nodes: Dict[int, Node],
    leaf_info: Dict[int, LeafInfo],
    edges: List[Edge],
    width: int,
    height: int,
    source_pos: Tuple[float, float],
    source_children: List[int],
) -> None:
    visual_pos = compute_grid_visual_pos(root, nodes, leaf_info, source_pos,
                                         width, height, source_children)
    num_sinks = max(1, len(leaf_info))
    sink_font_size = max(3, min(6, int(70 / num_sinks)))
    visited = set()

    if visual_pos:
        xs = [source_pos[0], 0.0, float(width)] + [pos[0] for pos in visual_pos.values()]
        ys = [source_pos[1], 0.0, float(height)] + [pos[1] for pos in visual_pos.values()]
        pad = max(2.0, 0.02 * float(max(width, height)))
        ax.set_xlim(min(xs) - pad, max(xs) + pad)
        ax.set_ylim(min(ys) - pad, max(ys) + pad)
    else:
        ax.set_xlim(0, width)
        ax.set_ylim(0, height)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.5, alpha=0.4)
    ax.set_title("Tree on Die Grid")

    def draw_branch(node_id: int) -> None:
        if node_id in visited:
            warn(f"geometry branch recursion skipped repeated node {node_id}")
            return
        visited.add(node_id)

        node = nodes.get(node_id)
        if node is None:
            warn(f"geometry branch skipped missing node {node_id}")
            return
        if node.is_leaf:
            return

        child_ids = [child_id for child_id in (node.left, node.right) if child_id >= 0]
        if node_id not in visual_pos or any(child_id not in visual_pos for child_id in child_ids):
            warn(f"geometry branch skipped node {node_id} because visual position is missing")
            return

        parent_pos = visual_pos[node_id]
        px, py = parent_pos
        for child_id in child_ids:
            cx, cy = visual_pos[child_id]
            ax.plot([px, cx], [py, cy], color="black", linewidth=2.8, zorder=1)
            draw_branch(child_id)

    if root >= 0:
        draw_branch(root)
    else:
        for child_id in source_children:
            draw_branch(child_id)

    root_pos = visual_pos.get(root) if root >= 0 else None
    if root_pos is not None:
        ax.plot(
            [source_pos[0], root_pos[0]],
            [source_pos[1], root_pos[1]],
            color="black",
            linewidth=2.8,
            zorder=1,
        )
    elif source_children:
        for child_id in source_children:
            child_pos = visual_pos.get(child_id)
            if child_pos is None:
                warn(f"source-to-child connection skipped because child {child_id} visual position is missing")
                continue
            ax.plot(
                [source_pos[0], child_pos[0]],
                [source_pos[1], child_pos[1]],
                color="black",
                linewidth=2.8,
                zorder=1,
            )
    else:
        warn(f"source-to-root connection skipped because root visual position is missing")

    internal_by_kind: Dict[str, Tuple[List[float], List[float]]] = {}
    for node_id, node in nodes.items():
        if node.is_leaf:
            continue
        if node_id not in visual_pos:
            warn(f"geometry tap for node {node_id} skipped because visual position is missing")
            continue
        x, y = visual_pos[node_id]
        xs, ys = internal_by_kind.setdefault(node.node_kind, ([], []))
        xs.append(x)
        ys.append(y)

    for kind, (xs, ys) in internal_by_kind.items():
        style = KIND_STYLES.get(kind, KIND_STYLES["UNKNOWN"])
        ax.scatter(
            xs,
            ys,
            marker=style["grid_marker"],
            s=55,
            facecolors=style["face"],
            edgecolors=style["edge"],
            linewidths=2.0,
            zorder=6,
        )

    for node_id, (sink_index, sink_id, lx, ly) in leaf_info.items():
        node = nodes.get(node_id)
        style = kind_style(node) if node is not None else KIND_STYLES["SINK"]
        ax.scatter(
            [float(lx)], [float(ly)],
            marker=style["grid_marker"],
            s=45,
            facecolors=style["face"],
            edgecolors=style["edge"],
            linewidths=1.2,
            zorder=5,
        )
        ax.text(float(lx), float(ly), sink_id, ha="center", va="center",
                fontsize=sink_font_size, color=style["text"], zorder=7)

    sx, sy = source_pos
    ax.scatter(
        [sx], [sy],
        marker="^",
        s=120,
        facecolors="red",
        edgecolors="darkred",
        linewidths=1.5,
        zorder=10,
    )
    ax.text(sx, sy, "SRC", ha="center", va="bottom", fontsize=10,
            color="darkred", zorder=11)
    add_kind_legend(ax)


def print_usage() -> None:
    print("Usage: python3 vtree.py <sample_index | tree/sample<k>_vtree.txt>")
    print("Example: python3 vtree.py 1")
    print("Example: python3 vtree.py tree/sample1_vtree.txt")


def resolve_paths(arg: str) -> Tuple[str, str, str]:
    if arg.isdigit():
        idx = arg
        return f"tree/sample{idx}_vtree.txt", f"samples/sample{idx}.txt", f"sample{idx}"

    tree_path = arg
    base = os.path.basename(tree_path)
    match = re.match(r"(sample(\d+))(?:_vtree)?\.txt$", base)
    if not match:
        fail(
            "tree path must look like tree/sample<k>_vtree.txt "
            "or pass just the sample index"
        )
    sample_base = match.group(1)
    idx = match.group(2)
    return tree_path, f"samples/sample{idx}.txt", sample_base


def main() -> None:
    if len(sys.argv) != 2:
        print_usage()
        sys.exit(1)

    tree_path, sample_path, sample_base = resolve_paths(sys.argv[1])

    if not os.path.exists(tree_path):
        fail(f"tree file not found: {tree_path}")
    if not os.path.exists(sample_path):
        fail(f"sample file not found: {sample_path}")

    root, nodes, leaf_info, edges, source_children = parse_tree_file(tree_path)
    width, height, source_x, source_y = parse_sample_file(sample_path)
    source_pos = (source_x, source_y)
    max_est_skew = compute_max_est_skew(root, nodes, leaf_info, source_pos,
                                        source_children)
    print(f"max_est_skew = {max_est_skew:.3f}")
    num_leaves = max(1, len(leaf_info))
    max_depth = compute_tree_depth(root, nodes, source_children)

    if "MPLCONFIGDIR" not in os.environ:
        try:
            os.makedirs("/tmp/matplotlib", exist_ok=True)
            os.environ["MPLCONFIGDIR"] = "/tmp/matplotlib"
        except OSError:
            pass

    if "MPLBACKEND" not in os.environ:
        os.environ["MPLBACKEND"] = "Agg"

    global plt
    import matplotlib.pyplot as plt

    fig_width = max(14, min(36, 0.32 * num_leaves + 10))
    fig_height = max(7, min(18, 0.70 * max_depth + 4))

    fig, (ax1, ax2) = plt.subplots(
        1,
        2,
        figsize=(fig_width, fig_height),
        layout="constrained",
        gridspec_kw={"width_ratios": [1.4, 1.0]},
    )
    draw_topology(ax1, root, nodes, leaf_info, edges, source_pos, source_children)
    draw_grid_tree(ax2, root, nodes, leaf_info, edges, width, height,
                   source_pos, source_children)
    fig.suptitle(f"Topology Tree Visualization: {sample_base} | max_est_skew={max_est_skew:.3f}")

    backend = plt.get_backend().lower()
    output_path = f"tree/{sample_base}_vtree.png"
    fig.savefig(output_path, dpi=160)
    print(f"saved visualization: {output_path}")
    if "agg" in backend or "pdf" in backend or "svg" in backend:
        return
    else:
        plt.show()


if __name__ == "__main__":
    main()
