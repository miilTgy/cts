import os
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
    est_delay: float


LeafInfo = Tuple[int, str, int, int]
GeoPos = Optional[Tuple[float, float]]


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
    edges: List[Tuple[int, int]] = []
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
            elif kind == "NUM_SINKS":
                if len(parts) != 2:
                    fail(f"{path}:{line_no}: NUM_SINKS expects 1 value")
                num_sinks = parse_int(parts[1], "NUM_SINKS")
            elif kind == "NODE":
                if len(parts) != 15:
                    fail(
                        f"{path}:{line_no}: NODE expects 14 values after NODE, "
                        f"got {len(parts) - 1}"
                    )
                node_id = parse_int(parts[1], "NODE id")
                node = Node(
                    id=node_id,
                    parent=parse_int(parts[2], "NODE parent"),
                    left=parse_int(parts[3], "NODE left"),
                    right=parse_int(parts[4], "NODE right"),
                    is_leaf=parse_int(parts[5], "NODE is_leaf") != 0,
                    sink_index=parse_int(parts[6], "NODE sink_index"),
                    sink_count=parse_int(parts[7], "NODE sink_count"),
                    cx=parse_float(parts[8], "NODE cx"),
                    cy=parse_float(parts[9], "NODE cy"),
                    bbox_lx=parse_int(parts[10], "NODE bbox_lx"),
                    bbox_ly=parse_int(parts[11], "NODE bbox_ly"),
                    bbox_ux=parse_int(parts[12], "NODE bbox_ux"),
                    bbox_uy=parse_int(parts[13], "NODE bbox_uy"),
                    est_delay=parse_float(parts[14], "NODE est_delay"),
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
                parent_id = parse_int(parts[1], "EDGE parent_id")
                child_id = parse_int(parts[2], "EDGE child_id")
                edges.append((parent_id, child_id))

    if not tree_valid_seen:
        fail(f"{path}: missing TREE_VALID line")
    if root < 0:
        fail(f"{path}: missing ROOT line")
    if root not in nodes:
        fail(f"{path}: ROOT node {root} is not present in NODE records")

    if num_nodes is not None and num_nodes != len(nodes):
        warn(f"{path}: NUM_NODES is {num_nodes}, but parsed {len(nodes)} NODE lines")
    if num_sinks is not None and num_sinks != len(leaf_info):
        warn(f"{path}: NUM_SINKS is {num_sinks}, but parsed {len(leaf_info)} LEAF lines")

    for parent_id, child_id in edges:
        if parent_id not in nodes:
            warn(f"{path}: EDGE references missing parent node {parent_id}")
        if child_id not in nodes:
            warn(f"{path}: EDGE references missing child node {child_id}")

    return root, nodes, leaf_info, edges


def parse_die_size(path: str) -> Tuple[int, int]:
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
                return width, height

    fail(f"{path}: missing DIE <width> <height>")


def compute_max_est_skew(nodes: Dict[int, Node], leaf_info: Dict[int, LeafInfo]) -> float:
    leaf_delays = []
    for node_id in leaf_info:
        if node_id in nodes:
            leaf_delays.append(nodes[node_id].est_delay)

    if not leaf_delays:
        return 0.0

    return max(leaf_delays) - min(leaf_delays)


def compute_tree_depth(root: int, nodes: Dict[int, Node]) -> int:
    def dfs(node_id: int) -> int:
        node = nodes.get(node_id)
        if node is None:
            warn(f"tree depth skipped missing node {node_id}")
            return 0
        if node.is_leaf:
            return 0

        child_depths = []
        for child_id in (node.left, node.right):
            if child_id < 0:
                warn(f"tree depth skipped missing child of node {node_id}")
                child_depths.append(0)
            else:
                child_depths.append(dfs(child_id))

        return 1 + max(child_depths)

    return dfs(root)


def compute_topology_layout(root: int, nodes: Dict[int, Node]) -> Dict[int, Tuple[float, float]]:
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

    dfs(root, 0)
    return pos


def draw_topology(
    ax,
    root: int,
    nodes: Dict[int, Node],
    leaf_info: Dict[int, LeafInfo],
    edges: List[Tuple[int, int]],
) -> None:
    pos = compute_topology_layout(root, nodes)
    num_leaves = max(1, len(leaf_info))
    marker_size = max(18, min(90, 2200 / num_leaves))
    font_size = max(4, min(8, int(150 / max(num_leaves, 1))))

    for parent_id, child_id in edges:
        if parent_id not in pos or child_id not in pos:
            warn(f"topology edge {parent_id}->{child_id} skipped because position is missing")
            continue
        x1, y1 = pos[parent_id]
        x2, y2 = pos[child_id]
        ax.plot([x1, x2], [y1, y2], color="black", linewidth=1.2, zorder=1)

    for node_id, (x, y) in pos.items():
        node = nodes[node_id]
        ax.scatter(
            [x],
            [y],
            s=marker_size,
            facecolors="white",
            edgecolors="black",
            linewidths=1.0,
            zorder=3,
        )
        if node.is_leaf:
            label = leaf_info.get(node_id, (node.sink_index, str(node_id), 0, 0))[1]
        else:
            label = str(node_id)
        ax.text(x, y, label, ha="center", va="center", fontsize=font_size, zorder=4)

    if pos:
        xs = [p[0] for p in pos.values()]
        ys = [p[1] for p in pos.values()]
        ax.set_xlim(min(xs) - 1.0, max(xs) + 1.0)
        ax.set_ylim(min(ys) - 1.0, max(ys) + 1.0)

    ax.set_title("Binary Topology")
    ax.set_aspect("auto")
    ax.axis("off")


def get_geo_pos(node_id: int, nodes: Dict[int, Node], leaf_info: Dict[int, LeafInfo]) -> GeoPos:
    if node_id in leaf_info:
        _, _, x, y = leaf_info[node_id]
        return float(x), float(y)

    node = nodes.get(node_id)
    if node is None:
        warn(f"missing node {node_id}; no geometry position available")
        return None

    return node.cx, node.cy


def draw_grid_tree(
    ax,
    root: int,
    nodes: Dict[int, Node],
    leaf_info: Dict[int, LeafInfo],
    edges: List[Tuple[int, int]],
    width: int,
    height: int,
) -> None:
    del root

    for parent_id, child_id in edges:
        parent_pos = get_geo_pos(parent_id, nodes, leaf_info)
        child_pos = get_geo_pos(child_id, nodes, leaf_info)
        if parent_pos is None or child_pos is None:
            warn(f"geometry edge {parent_id}->{child_id} skipped because position is missing")
            continue
        x1, y1 = parent_pos
        x2, y2 = child_pos
        ax.plot([x1, x2], [y1, y2], color="black", linewidth=2.5, zorder=2)

    xs = []
    ys = []
    for node_id in sorted(nodes):
        geo_pos = get_geo_pos(node_id, nodes, leaf_info)
        if geo_pos is None:
            warn(f"geometry point for node {node_id} skipped because position is missing")
            continue
        x, y = geo_pos
        xs.append(x)
        ys.append(y)

    ax.scatter(xs, ys, color="black", s=20, zorder=3)
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.5, alpha=0.4)
    ax.set_title("Tree on Die Grid")


def print_usage() -> None:
    print("Usage: python3 vtree.py <sample_index>")
    print("Example: python3 vtree.py 1")


def main() -> None:
    if len(sys.argv) != 2 or not sys.argv[1].isdigit():
        print_usage()
        sys.exit(1)

    idx = sys.argv[1]
    tree_path = f"tree/sample{idx}.txt"
    sample_path = f"samples/sample{idx}.txt"

    if not os.path.exists(tree_path):
        fail(f"tree file not found: {tree_path}")
    if not os.path.exists(sample_path):
        fail(f"sample file not found: {sample_path}")

    root, nodes, leaf_info, edges = parse_tree_file(tree_path)
    width, height = parse_die_size(sample_path)
    max_est_skew = compute_max_est_skew(nodes, leaf_info)
    print(f"max_est_skew = {max_est_skew:.3f}")
    num_leaves = max(1, len(leaf_info))
    max_depth = compute_tree_depth(root, nodes)

    if "MPLCONFIGDIR" not in os.environ:
        try:
            os.makedirs("/tmp/matplotlib", exist_ok=True)
            os.environ["MPLCONFIGDIR"] = "/tmp/matplotlib"
        except OSError:
            pass

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
    draw_topology(ax1, root, nodes, leaf_info, edges)
    draw_grid_tree(ax2, root, nodes, leaf_info, edges, width, height)
    fig.suptitle(f"Topology Tree Visualization: sample{idx} | max_est_skew={max_est_skew:.3f}")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
