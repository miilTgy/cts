#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

ROOT_DIR = Path(__file__).resolve().parents[1]
MPLCONFIGDIR = ROOT_DIR / ".cache" / "matplotlib"
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from scripts.vloc import (
    STYLE as NODE_STYLE,
    compute_cluster_bboxes,
    parse_loc_file,
    parse_sample_source,
)


NUM_RE = re.compile(r"[-+]?(?:\d*\.\d+|\d+)")
NAMED_INT_RE = re.compile(r"(?:^|[^\w]){name}\s*=\s*(-?\d+)|(?:^|[^\w]){name}\s+(-?\d+)")


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
    points: List[Tuple[float, float]] = field(default_factory=list)

    @property
    def routed(self) -> bool:
        return self.selected_shape.upper() != "FAILED" and len(self.points) >= 2


@dataclass
class TopologyEdge:
    parent: Optional[int]
    child: int


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


def warn(message: str) -> None:
    print(f"[WARN] {message}", file=sys.stderr)


def fail(message: str) -> None:
    print(f"[ERROR] {message}", file=sys.stderr)
    raise SystemExit(1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize router output")
    parser.add_argument("sample_id", type=int, help="sample id, e.g. 3 for sample3")
    parser.add_argument(
        "--label",
        choices=["none", "id", "class", "id_class", "skew"],
        default="id",
        help="node label mode",
    )
    parser.add_argument(
        "--edge-label",
        choices=["none", "id", "shape", "policy", "failure"],
        default="none",
        help="edge label mode",
    )
    parser.add_argument(
        "--bbox",
        action="store_true",
        help="draw cluster bboxes from sink nodes",
    )
    parser.add_argument(
        "--show-unrouted-topology",
        action="store_true",
        help="draw topology edges that are not present in routed output",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="show the plot in an interactive window",
    )
    return parser.parse_args()


def infer_paths(sample_id: int) -> Dict[str, Path]:
    sample_name = f"sample{sample_id}"
    return {
        "route": Path("route") / f"{sample_name}_route.txt",
        "loc": Path("loc") / f"{sample_name}_loc.txt",
        "tree": Path("tree") / f"{sample_name}_vtree.txt",
        "sample": Path("samples") / f"{sample_name}.txt",
        "out": Path("route") / "fig" / f"{sample_name}_route.png",
    }


def extract_numbers(line: str) -> List[float]:
    return [float(token) for token in NUM_RE.findall(line)]


def parse_int(token: str, default: int = -1) -> int:
    try:
        return int(float(token))
    except ValueError:
        return default


def parse_float(token: str, default: float = 0.0) -> float:
    try:
        return float(token)
    except ValueError:
        return default


def parse_named_int(line: str, name: str) -> Optional[int]:
    pattern = re.compile(rf"(?:^|[^\w]){re.escape(name)}\s*=\s*(-?\d+)|(?:^|[^\w]){re.escape(name)}\s+(-?\d+)")
    match = pattern.search(line)
    if not match:
        return None
    token = match.group(1) if match.group(1) is not None else match.group(2)
    return int(token)


def clean_points(points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    cleaned: List[Tuple[float, float]] = []
    for x, y in points:
        point = (float(x), float(y))
        if cleaned and cleaned[-1] == point:
            continue
        cleaned.append(point)
    return cleaned


def short_policy(policy: str) -> str:
    mapping = {
        "LocalClusterPatternOnly": "Local",
        "ExternalAccessPatternThenMaze": "External",
        "GlobalPatternThenMaze": "Global",
        "Unknown": "Unknown",
    }
    return mapping.get(policy, policy)


def edge_width(policy: str) -> float:
    mapping = {
        "LocalClusterPatternOnly": 2.0,
        "ExternalAccessPatternThenMaze": 2.6,
        "GlobalPatternThenMaze": 3.0,
    }
    return mapping.get(policy, 2.0)


def edge_color(shape: str) -> str:
    mapping = {
        "I": "#4a5568",
        "L": "#2b6cb0",
        "Z": "#805ad5",
        "MAZE": "#dd6b20",
        "FAILED": "#e53e3e",
    }
    return mapping.get(shape.upper(), "#4a5568")


def edge_style(shape: str, policy: str, routed: bool) -> Tuple[str, float, str, float]:
    color = edge_color(shape)
    linewidth = edge_width(policy)
    linestyle = "-"
    alpha = 0.88
    if not routed:
        color = "#e53e3e"
        linewidth = max(1.4, linewidth)
        linestyle = "--"
        alpha = 0.72
    return color, linewidth, linestyle, alpha


def route_label(edge: RouteEdge, label_mode: str) -> str:
    if label_mode == "none":
        return ""
    if label_mode == "id":
        return str(edge.edge_id)
    if label_mode == "shape":
        return edge.selected_shape
    if label_mode == "policy":
        return short_policy(edge.policy)
    if label_mode == "failure":
        return edge.failure_reason if edge.selected_shape.upper() == "FAILED" else ""
    return ""


def label_text(node: LocNode, label_mode: str) -> str:
    if label_mode == "none":
        return ""
    if label_mode == "id":
        return str(node.node_id)
    if label_mode == "class":
        return node.cls
    if label_mode == "id_class":
        return f"{node.node_id}:{node.cls}"
    if label_mode == "skew":
        return f"{node.node_id}\n{node.skew_to_node:g}"
    return str(node.node_id)


def parse_route_file(path: Path) -> List[RouteEdge]:
    if not path.exists():
        fail(f"missing route file: {path}")

    edges: List[RouteEdge] = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if not parts or parts[0].lower() != "edge":
                continue
            if len(parts) < 3:
                warn(f"{path}:{line_no}: malformed edge line")
                continue

            edge_id = parse_int(parts[1], -1)
            parent = parse_int(parts[2], -1)
            child = parse_int(parts[3], -1) if len(parts) > 3 else -1
            parent_class = parts[4] if len(parts) > 4 else "UNKNOWN"
            child_class = parts[5] if len(parts) > 5 else "UNKNOWN"
            policy = parts[6] if len(parts) > 6 else "Unknown"
            selected_shape = parts[7] if len(parts) > 7 else "FAILED"
            parent_exit_dir = parts[8] if len(parts) > 8 else ""
            child_entry_dir = parts[9] if len(parts) > 9 else ""
            score = parse_float(parts[10], 0.0) if len(parts) > 10 else 0.0
            wirelength = parse_float(parts[11], 0.0) if len(parts) > 11 else 0.0
            bends = parse_int(parts[12], 0) if len(parts) > 12 else 0
            pattern_candidate_count = parse_int(parts[13], 0) if len(parts) > 13 else 0
            maze_candidate_count = parse_int(parts[14], 0) if len(parts) > 14 else 0
            legal_candidate_count = parse_int(parts[15], 0) if len(parts) > 15 else 0
            failure_reason = parts[16] if len(parts) > 16 else ""
            point_count = parse_int(parts[17], 0) if len(parts) > 17 else 0

            points: List[Tuple[float, float]] = []
            coord_tokens = parts[18:] if len(parts) > 18 else []
            if coord_tokens:
                if len(coord_tokens) % 2 != 0:
                    warn(f"edge {edge_id} has odd number of coordinate tokens; trailing token ignored")
                pair_count = len(coord_tokens) // 2
                for i in range(pair_count):
                    x = parse_float(coord_tokens[2 * i], 0.0)
                    y = parse_float(coord_tokens[2 * i + 1], 0.0)
                    points.append((x, y))
            if point_count > 0 and point_count != len(points):
                warn(f"edge {edge_id} point_count mismatch, parsed {len(points)} points")
            if point_count <= 1 and selected_shape.upper() != "FAILED":
                warn(f"edge {edge_id} has insufficient points for a routed polyline")

            edges.append(
                RouteEdge(
                    edge_id=edge_id,
                    parent=parent,
                    child=child,
                    parent_class=parent_class,
                    child_class=child_class,
                    policy=policy,
                    selected_shape=selected_shape,
                    parent_exit_dir=parent_exit_dir,
                    child_entry_dir=child_entry_dir,
                    score=score,
                    wirelength=wirelength,
                    bends=bends,
                    pattern_candidate_count=pattern_candidate_count,
                    maze_candidate_count=maze_candidate_count,
                    legal_candidate_count=legal_candidate_count,
                    failure_reason=failure_reason,
                    points=clean_points(points),
                )
            )
    return edges


def parse_tree_topology(path: Path) -> Tuple[List[TopologyEdge], Optional[Tuple[float, float]], bool]:
    edges: List[TopologyEdge] = []
    source: Optional[Tuple[float, float]] = None
    saw_any = False

    if not path.exists():
        return edges, source, False

    with path.open("r", encoding="utf-8") as f:
        for line_no, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            kind = parts[0].upper()
            saw_any = True
            if kind == "SOURCE":
                nums = extract_numbers(line)
                if len(nums) >= 2:
                    source = (nums[-2], nums[-1])
            elif kind == "EDGE" and len(parts) >= 3:
                parent_token = parts[1]
                child = parse_int(parts[2], -1)
                if child < 0:
                    continue
                parent = None if parent_token.upper() == "SRC" else parse_int(parent_token, -1)
                edges.append(TopologyEdge(parent=parent, child=child))
            elif kind == "NODE":
                parent = parse_named_int(line, "parent")
                left = parse_named_int(line, "left")
                right = parse_named_int(line, "right")
                if parent is None and len(parts) >= 5:
                    parent = parse_int(parts[2], -1)
                    left = parse_int(parts[3], -1)
                    right = parse_int(parts[4], -1)
                node_id = parse_int(parts[1], -1)
                if node_id >= 0:
                    if parent is not None and parent >= 0:
                        edges.append(TopologyEdge(parent=parent, child=node_id))
                    if left is not None and left >= 0:
                        edges.append(TopologyEdge(parent=node_id, child=left))
                    if right is not None and right >= 0:
                        edges.append(TopologyEdge(parent=node_id, child=right))

    return dedupe_edges(edges), source, saw_any


def fallback_edges_from_loc(nodes: Dict[int, LocNode]) -> List[TopologyEdge]:
    edges: List[TopologyEdge] = []
    for node in nodes.values():
        if node.parent >= 0:
            edges.append(TopologyEdge(parent=node.parent, child=node.node_id))
        if node.left >= 0:
            edges.append(TopologyEdge(parent=node.node_id, child=node.left))
        if node.right >= 0:
            edges.append(TopologyEdge(parent=node.node_id, child=node.right))
    return dedupe_edges(edges)


def dedupe_edges(edges: Sequence[TopologyEdge]) -> List[TopologyEdge]:
    seen = set()
    out: List[TopologyEdge] = []
    for edge in edges:
        key = (edge.parent, edge.child)
        if key in seen:
            continue
        seen.add(key)
        out.append(edge)
    return out


def infer_source_from_route(route_edges: Sequence[RouteEdge], nodes: Dict[int, LocNode]) -> Optional[Tuple[float, float]]:
    for edge in route_edges:
        if edge.parent_class.upper() == "SOURCE":
            if edge.points:
                return edge.points[0]
            parent = nodes.get(edge.parent)
            if parent is not None:
                return parent.x, parent.y
    return None


def draw_route_polyline(ax, edge: RouteEdge, label_mode: str) -> None:
    if len(edge.points) >= 2:
        xs = [p[0] for p in edge.points]
        ys = [p[1] for p in edge.points]
        color, linewidth, linestyle, alpha = edge_style(edge.selected_shape, edge.policy, edge.routed)
        ax.plot(xs, ys, color=color, linewidth=linewidth, linestyle=linestyle, alpha=alpha, zorder=2)
        if not edge.routed and edge.selected_shape.upper() == "FAILED":
            ax.plot(xs, ys, color=color, linewidth=max(1.2, linewidth * 0.8), linestyle="--", alpha=0.5, zorder=2)
    return


def route_label_point(points: Sequence[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    if len(points) < 2:
        return None
    lengths: List[float] = []
    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        lengths.append(abs(x1 - x0) + abs(y1 - y0))
    total = sum(lengths)
    if total <= 0:
        return points[len(points) // 2]
    halfway = total / 2.0
    acc = 0.0
    for i, seg_len in enumerate(lengths):
        if acc + seg_len >= halfway:
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            ratio = 0.5 if seg_len == 0 else (halfway - acc) / seg_len
            return x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio
        acc += seg_len
    return points[-1]


def draw_edge_label(ax, edge: RouteEdge, label_mode: str) -> None:
    text = route_label(edge, label_mode)
    if not text:
        return
    pos = route_label_point(edge.points)
    if pos is None:
        return
    ax.text(
        pos[0] + 0.35,
        pos[1] + 0.35,
        text,
        fontsize=7,
        color="#1a202c",
        zorder=6,
        bbox=dict(boxstyle="round,pad=0.15", facecolor="white", edgecolor="none", alpha=0.72),
    )


def draw_failed_edge_markers(ax, edge: RouteEdge, nodes: Dict[int, LocNode], source: Optional[Tuple[float, float]]) -> None:
    if edge.points:
        pos = route_label_point(edge.points)
        if pos is None:
            return
        ax.scatter([pos[0]], [pos[1]], marker="x", s=70, color="#e53e3e", linewidths=2.0, zorder=4)
        return

    parent_pos = source if edge.parent < 0 else nodes.get(edge.parent)
    child_pos = nodes.get(edge.child)
    if parent_pos is None or child_pos is None:
        warn(f"skip failed edge {edge.edge_id} because endpoint loc missing")
        return
    if edge.parent < 0 and source is None:
        warn(f"skip failed edge {edge.edge_id} because source loc missing")
        return

    if edge.parent < 0:
        p0 = source
    else:
        p0 = (parent_pos.x, parent_pos.y)
    p1 = (child_pos.x, child_pos.y)
    if p0 is None:
        return
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color="#e53e3e", linestyle="--", linewidth=1.4, alpha=0.5, zorder=1)
    mx = (p0[0] + p1[0]) / 2.0
    my = (p0[1] + p1[1]) / 2.0
    ax.scatter([mx], [my], marker="x", s=60, color="#e53e3e", linewidths=2.0, zorder=4)


def plot_routes(
    nodes: Dict[int, LocNode],
    route_edges: Sequence[RouteEdge],
    topology_edges: Sequence[TopologyEdge],
    bboxes: Dict[int, Tuple[float, float, float, float]],
    source: Optional[Tuple[float, float]],
    out_path: Path,
    args: argparse.Namespace,
) -> None:
    if not nodes:
        fail("no node data available")

    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)

    all_x = [node.x for node in nodes.values()]
    all_y = [node.y for node in nodes.values()]
    for edge in route_edges:
        for x, y in edge.points:
            all_x.append(x)
            all_y.append(y)
    if source is not None:
        all_x.append(source[0])
        all_y.append(source[1])

    routed_pairs = {
        (edge.parent, edge.child)
        for edge in route_edges
        if edge.selected_shape.upper() != "FAILED"
    }
    has_source_topology = False

    if args.show_unrouted_topology or True:
        for topo_edge in topology_edges:
            if topo_edge.parent is None:
                has_source_topology = True
                if source is None:
                    continue
                child = nodes.get(topo_edge.child)
                if child is None:
                    warn(f"skip source topology edge ->{topo_edge.child} because endpoint loc missing")
                    continue
                ax.plot(
                    [source[0], child.x],
                    [source[1], child.y],
                    color="#718096",
                    linewidth=1.1,
                    linestyle="--",
                    alpha=0.55,
                    zorder=0,
                )
                continue
            if not args.show_unrouted_topology:
                continue
            if (topo_edge.parent, topo_edge.child) in routed_pairs:
                continue
            parent = nodes.get(topo_edge.parent)
            child = nodes.get(topo_edge.child)
            if parent is None or child is None:
                warn(
                    f"skip topology edge {topo_edge.parent}->{topo_edge.child} "
                    "because endpoint loc missing"
                )
                continue
            ax.plot(
                [parent.x, child.x],
                [parent.y, child.y],
                color="#cbd5e0",
                linewidth=1.0,
                linestyle="--",
                alpha=0.6,
                zorder=0,
            )

    for edge in route_edges:
        draw_route_polyline(ax, edge, args.edge_label)
        draw_edge_label(ax, edge, args.edge_label)
        if edge.selected_shape.upper() == "FAILED":
            draw_failed_edge_markers(ax, edge, nodes, source)

    if args.bbox and bboxes:
        for cluster_id, (xmin, ymin, xmax, ymax) in sorted(bboxes.items()):
            rect = Rectangle(
                (xmin, ymin),
                xmax - xmin,
                ymax - ymin,
                fill=False,
                linestyle="--",
                linewidth=1.0,
                edgecolor="#718096",
                alpha=0.55,
                zorder=1,
            )
            ax.add_patch(rect)
            ax.text(xmin, ymax, f"C{cluster_id}", fontsize=8, color="#4a5568", zorder=6)

    class_order = ["sink", "internal", "access", "bridge", "top", "global", "unknown"]
    for cls in class_order:
        cls_nodes = [node for node in nodes.values() if node.cls == cls]
        if not cls_nodes:
            continue
        style = NODE_STYLE.get(cls, NODE_STYLE["unknown"])
        xs = [node.x for node in cls_nodes]
        ys = [node.y for node in cls_nodes]
        ax.scatter(
            xs,
            ys,
            marker=style["marker"],
            s=style["size"],
            facecolors=style["face"],
            edgecolors=style["edge"],
            linewidths=style["linewidth"],
            zorder=5,
            label=cls,
        )

    if source is not None:
        style = NODE_STYLE["source"]
        ax.scatter(
            [source[0]],
            [source[1]],
            marker=style["marker"],
            s=style["size"],
            facecolors=style["face"],
            edgecolors=style["edge"],
            linewidths=style["linewidth"],
            zorder=6,
            label="source",
        )
        ax.text(source[0] + 0.5, source[1] + 0.5, "SRC", fontsize=8, zorder=7)

    for node in nodes.values():
        text = label_text(node, args.label)
        if not text:
            continue
        ax.text(
            node.x + 0.5,
            node.y + 0.5,
            text,
            fontsize=7,
            zorder=7,
            bbox=dict(boxstyle="round,pad=0.12", facecolor="white", edgecolor="none", alpha=0.68),
        )

    x_min = min(all_x) if all_x else 0.0
    x_max = max(all_x) if all_x else 1.0
    y_min = min(all_y) if all_y else 0.0
    y_max = max(all_y) if all_y else 1.0
    pad_x = max(3.0, (x_max - x_min) * 0.08)
    pad_y = max(3.0, (y_max - y_min) * 0.08)
    ax.set_xlim(x_min - pad_x, x_max + pad_x)
    ax.set_ylim(y_min - pad_y, y_max + pad_y)

    legend_handles: List[Line2D] = []
    legend_handles.extend(
        [
            Line2D([0], [0], color="#4a5568", linewidth=3.0, label="I"),
            Line2D([0], [0], color="#2b6cb0", linewidth=3.0, label="L"),
            Line2D([0], [0], color="#805ad5", linewidth=3.0, label="Z"),
            Line2D([0], [0], color="#dd6b20", linewidth=3.0, label="MAZE"),
            Line2D([0], [0], color="#e53e3e", linewidth=3.0, linestyle="--", label="FAILED"),
        ]
    )
    legend_handles.extend(
        [
            Line2D([0], [0], color="#2f855a", linewidth=3.0, label="Global policy"),
            Line2D([0], [0], color="#2b6cb0", linewidth=2.6, label="External policy"),
            Line2D([0], [0], color="#718096", linewidth=2.0, label="Local policy"),
        ]
    )
    for cls in class_order:
        style = NODE_STYLE.get(cls, NODE_STYLE["unknown"])
        legend_handles.append(
            Line2D(
                [0],
                [0],
                marker=style["marker"],
                color="none",
                markerfacecolor=style["face"],
                markeredgecolor=style["edge"],
                markeredgewidth=style["linewidth"],
                markersize=8,
                label=cls,
            )
        )
    if source is not None:
        style = NODE_STYLE["source"]
        legend_handles.append(
            Line2D(
                [0],
                [0],
                marker=style["marker"],
                color="none",
                markerfacecolor=style["face"],
                markeredgecolor=style["edge"],
                markeredgewidth=style["linewidth"],
                markersize=10,
                label="source",
            )
        )
    if args.bbox and bboxes:
        legend_handles.append(
            Line2D([0], [0], color="#718096", linestyle="--", linewidth=1.0, label="cluster bbox")
        )
    if args.show_unrouted_topology or has_source_topology:
        legend_handles.append(
            Line2D([0], [0], color="#cbd5e0", linestyle="--", linewidth=1.0, label="unrouted topology")
        )

    ax.legend(handles=legend_handles, loc="best", fontsize=8, framealpha=0.92, ncol=2)

    fig.savefig(out_path, dpi=220)
    if args.show:
        plt.show()
    plt.close(fig)


def main() -> None:
    args = parse_args()
    if args.show:
        plt.switch_backend("TkAgg")
    paths = infer_paths(args.sample_id)

    print(f"[VROUTE] route file: {paths['route']}")
    print(f"[VROUTE] loc file: {paths['loc']}")
    print(f"[VROUTE] tree file: {paths['tree']}")

    nodes = parse_loc_file(paths["loc"])
    if not nodes:
        fail(f"missing or empty loc file: {paths['loc']}")

    route_edges = parse_route_file(paths["route"])
    topology_edges, tree_source, tree_ok = parse_tree_topology(paths["tree"])
    if not tree_ok or not topology_edges:
        warn(
            f"failed to parse {paths['tree']}, fallback to loc parent/left/right fields"
        )
        topology_edges = fallback_edges_from_loc(nodes)

    source = parse_sample_source(paths["sample"]) or tree_source or infer_source_from_route(route_edges, nodes)
    if source is None:
        warn("source location not found in sample/tree/route inputs")

    bboxes = compute_cluster_bboxes(nodes)

    routed_count = sum(1 for edge in route_edges if edge.routed)
    failed_count = len(route_edges) - routed_count

    print(f"[VROUTE] parsed nodes: {len(nodes)}")
    print(f"[VROUTE] parsed routes: {len(route_edges)}")
    print(f"[VROUTE] routed edges: {routed_count}")
    print(f"[VROUTE] failed edges: {failed_count}")

    plot_routes(
        nodes=nodes,
        route_edges=route_edges,
        topology_edges=topology_edges,
        bboxes=bboxes,
        source=source,
        out_path=paths["out"],
        args=args,
    )

    print(f"[VROUTE] output: {paths['out']}")


if __name__ == "__main__":
    main()
