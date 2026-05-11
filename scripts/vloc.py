#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import re
import sys
from collections import defaultdict
from dataclasses import dataclass
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


NUM_RE = re.compile(r"[-+]?(?:\d*\.\d+|\d+)")


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


Edge = Tuple[Optional[int], int]
BBox = Tuple[float, float, float, float]

STYLE = {
    "sink": dict(marker="o", face="#202020", edge="#202020", size=45, linewidth=1.2),
    "internal": dict(marker="o", face="#ffe08a", edge="#b7791f", size=55, linewidth=2.0),
    "access": dict(marker="s", face="#90cdf4", edge="#2b6cb0", size=55, linewidth=2.0),
    "bridge": dict(marker="P", face="#d6bcfa", edge="#6b46c1", size=55, linewidth=2.0),
    "top": dict(marker="*", face="#fbd38d", edge="#c05621", size=55, linewidth=2.0),
    "global": dict(marker="D", face="#c6f6d5", edge="#2f855a", size=55, linewidth=2.0),
    "source": dict(marker="^", face="red", edge="darkred", size=120, linewidth=1.5),
    "unknown": dict(marker="o", face="white", edge="black", size=55, linewidth=2.0),
}


def warn(message: str) -> None:
    print(f"[WARN] {message}", file=sys.stderr)


def fail(message: str) -> None:
    print(f"Error: {message}", file=sys.stderr)
    raise SystemExit(1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize locer node placements")
    parser.add_argument("sample_id", type=int, help="sample id, e.g. 3 for sample3")
    parser.add_argument(
        "--label",
        choices=["none", "id", "class", "id_class", "skew"],
        default="id",
        help="node label mode",
    )
    parser.add_argument(
        "--bbox",
        action="store_true",
        help="draw cluster bboxes from sink nodes",
    )
    return parser.parse_args()


def infer_paths(sample_id: int) -> Dict[str, Path]:
    sample_name = f"sample{sample_id}"
    return {
        "loc": Path("loc") / f"{sample_name}_loc.txt",
        "tree": Path("tree") / f"{sample_name}_vtree.txt",
        "sample": Path("samples") / f"{sample_name}.txt",
        "out": Path("loc") / "fig" / f"{sample_name}_loc.png",
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


def parse_loc_file(path: Path) -> Dict[int, LocNode]:
    nodes: Dict[int, LocNode] = {}
    if not path.exists():
        fail(f"missing loc file: {path}")

    with path.open("r", encoding="utf-8") as f:
        for line_no, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if not parts or parts[0].lower() != "node":
                continue
            if len(parts) < 7:
                warn(f"{path}:{line_no}: malformed node line")
                continue

            node_id = parse_int(parts[1], -1)
            if node_id < 0:
                warn(f"{path}:{line_no}: invalid node id")
                continue

            node = LocNode(
                node_id=node_id,
                cls=parts[2].lower(),
                cluster_id=parse_int(parts[3], -1),
                x=parse_float(parts[4]),
                y=parse_float(parts[5]),
                loc_mode=parts[6],
                parent=parse_int(parts[7], -1) if len(parts) > 7 else -1,
                left=parse_int(parts[8], -1) if len(parts) > 8 else -1,
                right=parse_int(parts[9], -1) if len(parts) > 9 else -1,
                candidate_count=parse_int(parts[10], 0) if len(parts) > 10 else 0,
                loc_score=parse_float(parts[11], 0.0) if len(parts) > 11 else 0.0,
                inside_related_bbox=parse_int(parts[12], 0) if len(parts) > 12 else 0,
                congestion_penalty=parse_float(parts[13], 0.0) if len(parts) > 13 else 0.0,
                lshape_penalty=parse_float(parts[14], 0.0) if len(parts) > 14 else 0.0,
                wire_est_to_parent=parse_float(parts[15], 0.0) if len(parts) > 15 else 0.0,
                sink_delay_count=parse_int(parts[16], 0) if len(parts) > 16 else 0,
                min_sink_delay=parse_float(parts[17], 0.0) if len(parts) > 17 else 0.0,
                max_sink_delay=parse_float(parts[18], 0.0) if len(parts) > 18 else 0.0,
                skew_to_node=parse_float(parts[19], 0.0) if len(parts) > 19 else 0.0,
                skew_penalty=parse_float(parts[20], 0.0) if len(parts) > 20 else 0.0,
            )
            nodes[node_id] = node

    return nodes


def parse_sample_source(path: Path) -> Optional[Tuple[float, float]]:
    if not path.exists():
        return None

    with path.open("r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if parts and parts[0].upper() == "SOURCE":
                nums = extract_numbers(line)
                if len(nums) >= 2:
                    return nums[-2], nums[-1]
    return None


def parse_tree_edges(path: Path) -> Tuple[List[Edge], Optional[Tuple[float, float]]]:
    edges: List[Edge] = []
    source: Optional[Tuple[float, float]] = None

    if not path.exists():
        return edges, source

    with path.open("r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            kind = parts[0].upper()
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
                edges.append((parent, child))

    return dedupe_edges(edges), source


def fallback_edges_from_loc(nodes: Dict[int, LocNode]) -> List[Edge]:
    edges: List[Edge] = []
    for node in nodes.values():
        if node.parent >= 0:
            edges.append((node.parent, node.node_id))
        else:
            edges.append((None, node.node_id))
        if node.left >= 0:
            edges.append((node.node_id, node.left))
        if node.right >= 0:
            edges.append((node.node_id, node.right))
    return dedupe_edges(edges)


def dedupe_edges(edges: Sequence[Edge]) -> List[Edge]:
    seen = set()
    out: List[Edge] = []
    for parent, child in edges:
        key = (parent, child)
        if key in seen:
            continue
        seen.add(key)
        out.append((parent, child))
    return out


def compute_cluster_bboxes(nodes: Dict[int, LocNode]) -> Dict[int, BBox]:
    cluster_points: Dict[int, List[Tuple[float, float]]] = defaultdict(list)
    for node in nodes.values():
        if node.cluster_id >= 0 and node.cls == "sink":
            cluster_points[node.cluster_id].append((node.x, node.y))

    bboxes: Dict[int, BBox] = {}
    for cluster_id, points in cluster_points.items():
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        bboxes[cluster_id] = (min(xs), min(ys), max(xs), max(ys))
    return bboxes


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


def edge_style(parent_cls: Optional[str], child_cls: str) -> Tuple[str, float]:
    outer_classes = {"source", "global", "bridge", "top"}
    if parent_cls in outer_classes or child_cls in outer_classes:
        return "black", 2.8
    return "black", 2.0


def point_for_source(source: Optional[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    if source is None:
        return None
    return float(source[0]), float(source[1])


def plot_loc(
    nodes: Dict[int, LocNode],
    edges: Sequence[Edge],
    bboxes: Dict[int, BBox],
    out_path: Path,
    label_mode: str,
    source: Optional[Tuple[float, float]] = None,
    draw_bboxes: bool = False,
) -> None:
    if not nodes:
        fail("no node data available")

    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)

    all_x = [node.x for node in nodes.values()]
    all_y = [node.y for node in nodes.values()]
    if source is not None:
        all_x.append(source[0])
        all_y.append(source[1])

    source_point = point_for_source(source)
    used_classes = {node.cls for node in nodes.values()}

    for parent_id, child_id in edges:
        child = nodes.get(child_id)
        if child is None:
            warn(f"skip edge {parent_id}->{child_id} because endpoint loc missing")
            continue

        if parent_id is None:
            parent_point = source_point
            parent_cls = "source"
        else:
            parent = nodes.get(parent_id)
            if parent is None:
                warn(f"skip edge {parent_id}->{child_id} because endpoint loc missing")
                continue
            parent_point = (parent.x, parent.y)
            parent_cls = parent.cls

        if parent_point is None:
            warn(f"skip edge {parent_id}->{child_id} because source loc missing")
            continue

        color, linewidth = edge_style(parent_cls, child.cls)
        ax.plot(
            [parent_point[0], child.x],
            [parent_point[1], child.y],
            color=color,
            linewidth=linewidth,
            alpha=0.78,
            zorder=1,
        )

    if draw_bboxes and bboxes:
        for cluster_id, (xmin, ymin, xmax, ymax) in sorted(bboxes.items()):
            rect = Rectangle(
                (xmin, ymin),
                xmax - xmin,
                ymax - ymin,
                fill=False,
                linestyle="--",
                linewidth=1.2,
                edgecolor="#718096",
                alpha=0.8,
                zorder=0,
            )
            ax.add_patch(rect)
            ax.text(xmin, ymax, f"C{cluster_id}", fontsize=8, color="#4a5568")

    class_order = ["sink", "internal", "access", "bridge", "top", "global", "unknown"]
    for cls in class_order:
        cls_nodes = [node for node in nodes.values() if node.cls == cls]
        if not cls_nodes:
            continue
        style = STYLE.get(cls, STYLE["unknown"])
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
            zorder=3,
            label=cls,
        )

    if source_point is not None:
        style = STYLE["source"]
        ax.scatter(
            [source_point[0]],
            [source_point[1]],
            marker=style["marker"],
            s=style["size"],
            facecolors=style["face"],
            edgecolors=style["edge"],
            linewidths=style["linewidth"],
            zorder=4,
            label="source",
        )

    for node in nodes.values():
        text = label_text(node, label_mode)
        if not text:
            continue
        ax.text(node.x + 0.5, node.y + 0.5, text, fontsize=7, zorder=5)

    if source_point is not None:
        ax.text(source_point[0] + 0.5, source_point[1] + 0.5, "SRC", fontsize=8, zorder=5)

    x_min = min(all_x) if all_x else 0.0
    x_max = max(all_x) if all_x else 1.0
    y_min = min(all_y) if all_y else 0.0
    y_max = max(all_y) if all_y else 1.0
    pad_x = max(3.0, (x_max - x_min) * 0.08)
    pad_y = max(3.0, (y_max - y_min) * 0.08)
    ax.set_xlim(x_min - pad_x, x_max + pad_x)
    ax.set_ylim(y_min - pad_y, y_max + pad_y)

    legend_handles: List[Line2D] = [
        Line2D([0], [0], color="black", linewidth=2.8, label="outer edge"),
        Line2D([0], [0], color="black", linewidth=2.0, label="internal edge"),
    ]
    for cls in class_order:
        if cls not in used_classes and cls != "unknown":
            continue
        style = STYLE.get(cls, STYLE["unknown"])
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
    if source_point is not None:
        style = STYLE["source"]
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
    if draw_bboxes and bboxes:
        legend_handles.append(
            Line2D([0], [0], color="#718096", linestyle="--", linewidth=1.2, label="cluster bbox")
        )
    ax.legend(handles=legend_handles, loc="best", fontsize=8, framealpha=0.92)

    fig.savefig(out_path, dpi=220)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    paths = infer_paths(args.sample_id)

    print(f"[VLOC] loc file: {paths['loc']}")
    print(f"[VLOC] tree file: {paths['tree']}")

    nodes = parse_loc_file(paths["loc"])
    if not nodes:
        fail(f"no nodes parsed from {paths['loc']}")

    edges, tree_source = parse_tree_edges(paths["tree"])
    source = parse_sample_source(paths["sample"]) or tree_source
    if not edges:
        warn(f"failed to parse tree file; fallback to loc file edges")
        edges = fallback_edges_from_loc(nodes)

    bboxes = compute_cluster_bboxes(nodes)

    print(f"[VLOC] parsed nodes: {len(nodes)}")
    print(f"[VLOC] parsed edges: {len(edges)}")

    plot_loc(
        nodes=nodes,
        edges=edges,
        bboxes=bboxes,
        out_path=paths["out"],
        label_mode=args.label,
        source=source,
        draw_bboxes=args.bbox,
    )

    print(f"[VLOC] output: {paths['out']}")


if __name__ == "__main__":
    main()
