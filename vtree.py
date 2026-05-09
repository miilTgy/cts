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
    region_lx: int
    region_ly: int
    region_ux: int
    region_uy: int
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
                if len(parts) not in (15, 19):
                    fail(
                        f"{path}:{line_no}: NODE expects 14 or 18 values after NODE, "
                        f"got {len(parts) - 1}"
                    )
                node_id = parse_int(parts[1], "NODE id")
                if len(parts) == 19:
                    region_lx = parse_int(parts[14], "NODE region_lx")
                    region_ly = parse_int(parts[15], "NODE region_ly")
                    region_ux = parse_int(parts[16], "NODE region_ux")
                    region_uy = parse_int(parts[17], "NODE region_uy")
                    est_delay = parse_float(parts[18], "NODE est_delay")
                else:
                    region_lx = 0
                    region_ly = 0
                    region_ux = 0
                    region_uy = 0
                    est_delay = parse_float(parts[14], "NODE est_delay")
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
                    region_lx=region_lx,
                    region_ly=region_ly,
                    region_ux=region_ux,
                    region_uy=region_uy,
                    est_delay=est_delay,
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


def compute_max_est_skew(
    root: int,
    nodes: Dict[int, Node],
    leaf_info: Dict[int, LeafInfo],
    source_pos: Tuple[float, float],
) -> float:
    leaf_delays = []
    for leaf_id in leaf_info:
        if leaf_id not in nodes:
            warn(f"skew skipped missing leaf node {leaf_id}")
            continue

        total = 0.0
        current = leaf_id
        while current != root:
            node = nodes.get(current)
            if node is None:
                warn(f"skew path from leaf {leaf_id} stopped at missing node {current}")
                break

            parent_id = node.parent
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
        else:
            root_node = nodes.get(root)
            if root_node is None:
                warn(f"skew skipped root-to-source segment because root node {root} is missing")
            else:
                total += manhattan(root_node.cx, root_node.cy, source_pos[0], source_pos[1])
            leaf_delays.append(total)

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
        is_root = node_id == root
        facecolor = "gold" if is_root else "white"
        edgecolor = "darkorange" if is_root else "black"
        ax.scatter(
            [x],
            [y],
            s=marker_size,
            facecolors=facecolor,
            edgecolors=edgecolor,
            linewidths=1.0,
            zorder=3,
        )
        if node.is_leaf:
            label = leaf_info.get(node_id, (node.sink_index, str(node_id), 0, 0))[1]
        else:
            label = str(node_id)
        ax.text(x, y, label, ha="center", va="center", fontsize=font_size, zorder=4)
        if is_root:
            ax.text(x, y - 0.4, "ROOT", ha="center", va="top",
                    fontsize=font_size * 0.85, fontweight="bold",
                    color="darkorange", zorder=5)

    if pos:
        xs = [p[0] for p in pos.values()]
        ys = [p[1] for p in pos.values()]
        ax.set_xlim(min(xs) - 1.0, max(xs) + 1.0)
        ax.set_ylim(min(ys) - 1.0, max(ys) + 1.0)

    ax.set_title("Binary Topology")
    ax.set_aspect("auto")
    ax.axis("off")


def compute_grid_visual_pos(
    root: int,
    nodes: Dict[int, Node],
    leaf_info: Dict[int, LeafInfo],
) -> Dict[int, Tuple[float, float]]:
    visual_pos: Dict[int, Tuple[float, float]] = {}
    visiting = set()

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
            left_id = node.left
            right_id = node.right
            if left_id < 0 or right_id < 0:
                warn(f"grid visual position skipped incomplete children of node {node_id}")
                visiting.remove(node_id)
                return None

            left_pos = dfs(left_id)
            right_pos = dfs(right_id)
            if left_pos is None or right_pos is None:
                warn(f"grid visual position skipped node {node_id} because child position is missing")
                visiting.remove(node_id)
                return None

            ux = (left_pos[0] + right_pos[0]) / 2.0
            uy = (left_pos[1] + right_pos[1]) / 2.0
            visual_pos[node_id] = (ux, uy)

        visiting.remove(node_id)
        return visual_pos[node_id]

    dfs(root)
    return visual_pos


def draw_grid_tree(
    ax,
    root: int,
    nodes: Dict[int, Node],
    leaf_info: Dict[int, LeafInfo],
    edges: List[Tuple[int, int]],
    width: int,
    height: int,
    source_pos: Tuple[float, float],
) -> None:
    visual_pos = compute_grid_visual_pos(root, nodes, leaf_info)
    num_sinks = max(1, len(leaf_info))
    sink_font_size = max(3, min(6, int(70 / num_sinks)))
    visited = set()

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

        left_id = node.left
        right_id = node.right
        if node_id not in visual_pos or left_id not in visual_pos or right_id not in visual_pos:
            warn(f"geometry branch skipped node {node_id} because visual position is missing")
            return

        parent_pos = visual_pos[node_id]
        left_pos = visual_pos[left_id]
        right_pos = visual_pos[right_id]

        px, py = parent_pos
        lx, ly = left_pos
        rx, ry = right_pos

        ax.plot([px, lx], [py, ly], color="black", linewidth=2.8, zorder=1)
        ax.plot([px, rx], [py, ry], color="black", linewidth=2.8, zorder=1)

        draw_branch(left_id)
        draw_branch(right_id)

    draw_branch(root)
    root_pos = visual_pos.get(root)
    if root_pos is not None:
        ax.plot(
            [source_pos[0], root_pos[0]],
            [source_pos[1], root_pos[1]],
            color="black",
            linewidth=2.8,
            zorder=1,
        )
    else:
        warn(f"source-to-root connection skipped because root visual position is missing")

    internal_xs = []
    internal_ys = []
    for node_id, node in nodes.items():
        if node.is_leaf:
            continue
        if node_id not in visual_pos:
            warn(f"geometry tap for node {node_id} skipped because visual position is missing")
            continue
        x, y = visual_pos[node_id]
        internal_xs.append(x)
        internal_ys.append(y)

    if internal_xs:
        ax.scatter(
            internal_xs,
            internal_ys,
            marker="o",
            s=55,
            facecolors="white",
            edgecolors="red",
            linewidths=2.0,
            zorder=6,
        )

    sink_xs = []
    sink_ys = []
    sink_ids = []
    for node_id, (sink_index, sink_id, lx, ly) in leaf_info.items():
        sink_xs.append(float(lx))
        sink_ys.append(float(ly))
        sink_ids.append(sink_id)

    if sink_xs:
        ax.scatter(
            sink_xs, sink_ys,
            marker="o",
            s=45,
            color="black",
            zorder=5,
        )
        for x, y, label in zip(sink_xs, sink_ys, sink_ids):
            ax.text(x, y, label, ha="center", va="center", fontsize=sink_font_size,
                    color="black", zorder=7)

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
    width, height, source_x, source_y = parse_sample_file(sample_path)
    source_pos = (source_x, source_y)
    max_est_skew = compute_max_est_skew(root, nodes, leaf_info, source_pos)
    print(f"max_est_skew = {max_est_skew:.3f}")
    num_leaves = max(1, len(leaf_info))
    max_depth = compute_tree_depth(root, nodes)

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
    draw_topology(ax1, root, nodes, leaf_info, edges)
    draw_grid_tree(ax2, root, nodes, leaf_info, edges, width, height, source_pos)
    fig.suptitle(f"Topology Tree Visualization: sample{idx} | max_est_skew={max_est_skew:.3f}")

    backend = plt.get_backend().lower()
    if "agg" in backend or "pdf" in backend or "svg" in backend:
        output_path = f"tree/sample{idx}_vtree.png"
        fig.savefig(output_path, dpi=160)
        print(f"saved visualization: {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
