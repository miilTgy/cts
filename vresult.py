#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import re
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

ROOT_DIR = Path(__file__).resolve().parent
MPLCONFIGDIR = ROOT_DIR / ".cache" / "matplotlib"
MPLCONFIGDIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPLCONFIGDIR))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle


NUM_RE = re.compile(r"[-+]?(?:\d*\.\d+|\d+)")


STYLE = {
    "sink":     dict(marker="o", face="#202020", edge="#202020", size=40, linewidth=1.2),
    "internal": dict(marker="o", face="#ffe08a", edge="#b7791f", size=55, linewidth=2.0),
    "access":   dict(marker="s", face="#90cdf4", edge="#2b6cb0", size=55, linewidth=2.0),
    "bridge":   dict(marker="P", face="#d6bcfa", edge="#6b46c1", size=55, linewidth=2.0),
    "top":      dict(marker="*", face="#fbd38d", edge="#c05621", size=55, linewidth=2.0),
    "global":   dict(marker="D", face="#c6f6d5", edge="#2f855a", size=55, linewidth=2.0),
    "source":   dict(marker="^", face="red", edge="darkred", size=120, linewidth=1.5),
    "buffer":   dict(marker="H", face="#e11d48", edge="#9f1239", size=95, linewidth=2.0),
    "unknown":  dict(marker="o", face="white", edge="black", size=55, linewidth=2.0),
}


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


@dataclass
class RoutePath:
    sink_id: str
    points: List[Tuple[float, float]]


@dataclass
class BufferInfo:
    buf_id: str
    type_name: str
    x: float
    y: float


def warn(message: str) -> None:
    print(f"[WARN] {message}", file=sys.stderr)


def fail(message: str) -> None:
    print(f"Error: {message}", file=sys.stderr)
    raise SystemExit(1)


def extract_numbers(line: str) -> List[float]:
    return [float(t) for t in NUM_RE.findall(line)]


def parse_int(token: str, default: int = -1) -> int:
    try:
        return int(float(token))
    except (ValueError, TypeError):
        return default


def parse_float(token: str, default: float = 0.0) -> float:
    try:
        return float(token)
    except (ValueError, TypeError):
        return default


# --- Loc file parser (from vloc.py) ---

def parse_loc_file(path: Path) -> Dict[int, LocNode]:
    nodes: Dict[int, LocNode] = {}
    if not path.exists():
        return nodes
    with path.open("r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if not parts or parts[0].lower() != "node":
                continue
            if len(parts) < 6:
                continue
            node_id = parse_int(parts[1], -1)
            if node_id < 0:
                continue
            nodes[node_id] = LocNode(
                node_id=node_id,
                cls=parts[2],
                cluster_id=parse_int(parts[3], -1),
                x=parse_float(parts[4]),
                y=parse_float(parts[5]),
                loc_mode=parts[6] if len(parts) > 6 else "",
                parent=parse_int(parts[7], -1) if len(parts) > 7 else -1,
                left=parse_int(parts[8], -1) if len(parts) > 8 else -1,
                right=parse_int(parts[9], -1) if len(parts) > 9 else -1,
            )
    return nodes


def compute_cluster_bboxes(nodes: Dict[int, LocNode]) -> Dict[int, Tuple[float, float, float, float]]:
    cluster_sinks: Dict[int, List[LocNode]] = defaultdict(list)
    for node in nodes.values():
        if node.cls == "sink" and node.cluster_id >= 0:
            cluster_sinks[node.cluster_id].append(node)
    bboxes: Dict[int, Tuple[float, float, float, float]] = {}
    for cid, snks in cluster_sinks.items():
        xmin = min(n.x for n in snks)
        xmax = max(n.x for n in snks)
        ymin = min(n.y for n in snks)
        ymax = max(n.y for n in snks)
        bboxes[cid] = (xmin, ymin, xmax, ymax)
    return bboxes


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


def parse_die(path: Path) -> Tuple[float, float]:
    if not path.exists():
        return 0.0, 0.0
    with path.open("r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if parts and parts[0].upper() == "DIE" and len(parts) >= 3:
                return float(parts[1]), float(parts[2])
    return 0.0, 0.0


# --- Solution file parser ---

def parse_solution_file(path: Path) -> Tuple[List[BufferInfo], List[RoutePath]]:
    buffers: List[BufferInfo] = []
    routes: List[RoutePath] = []

    if not path.exists():
        fail(f"missing solution file: {path}")

    with path.open("r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f if ln.strip() and not ln.strip().startswith("#")]

    i = 0
    # NUM_BUFS
    if i >= len(lines) or not lines[i].startswith("NUM_BUFS"):
        fail("expected NUM_BUFS")
    num_bufs = int(lines[i].split()[1])
    i += 1

    # BUF lines
    for _ in range(num_bufs):
        if i >= len(lines):
            fail("missing BUF line")
        parts = lines[i].split()
        if parts[0] != "BUF" or len(parts) < 3:
            fail(f"malformed BUF line: {lines[i]}")
        buffers.append(BufferInfo(
            buf_id=parts[1],
            type_name=parts[2] if len(parts) > 2 else "UNKNOWN",
            x=float(parts[3]) if len(parts) > 3 else 0.0,
            y=float(parts[4]) if len(parts) > 4 else 0.0,
        ))
        i += 1

    # NUM_ROUTES
    if i >= len(lines) or not lines[i].startswith("NUM_ROUTES"):
        fail("expected NUM_ROUTES")
    num_routes = int(lines[i].split()[1])
    i += 1

    # ROUTE lines
    for _ in range(num_routes):
        if i >= len(lines):
            fail("missing ROUTE line")
        header = lines[i].split()
        if header[0] != "ROUTE" or len(header) < 3:
            fail(f"malformed ROUTE line: {lines[i]}")
        sink_id = header[1]
        point_count = int(header[2])
        i += 1
        points: List[Tuple[float, float]] = []
        for _ in range(point_count):
            if i >= len(lines):
                fail(f"missing point for ROUTE {sink_id}")
            coords = lines[i].split()
            if len(coords) < 2:
                fail(f"malformed point: {lines[i]}")
            points.append((float(coords[0]), float(coords[1])))
            i += 1
        routes.append(RoutePath(sink_id=sink_id, points=points))

    return buffers, routes


def clean_points(points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    cleaned: List[Tuple[float, float]] = []
    for x, y in points:
        pt = (float(x), float(y))
        if cleaned and cleaned[-1] == pt:
            continue
        cleaned.append(pt)
    return cleaned


# --- Visualisation ---

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize final solution (routing + buffers)")
    parser.add_argument("sample_id", type=int, help="sample id, e.g. 3 for sample3")
    parser.add_argument(
        "--bbox",
        action="store_true",
        help="draw cluster bboxes from sink nodes",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="show the plot interactively",
    )
    return parser.parse_args()


def infer_paths(sample_id: int) -> Dict[str, Path]:
    name = f"sample{sample_id}"
    return {
        "solution": Path("result") / f"{name}_solution.txt",
        "loc": Path("loc") / f"{name}_loc.txt",
        "sample": Path("samples") / f"{name}.txt",
        "out": Path("result") / "fig" / f"{name}_solution.png",
    }


def route_color(idx: int, total: int) -> str:
    import colorsys
    hue = idx / max(1, total)
    r, g, b = colorsys.hsv_to_rgb(hue * 0.85, 0.55, 0.72)
    return f"#{int(r*255):02x}{int(g*255):02x}{int(b*255):02x}"


def plot_solution(
    nodes: Dict[int, LocNode],
    routes: List[RoutePath],
    buffers: List[BufferInfo],
    bboxes: Dict[int, Tuple[float, float, float, float]],
    source: Optional[Tuple[float, float]],
    die_size: Tuple[float, float],
    out_path: Path,
    args: argparse.Namespace,
) -> None:  # noqa: ARG001
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.set_aspect("equal", adjustable="box")

    # Draw die boundary
    dw, dh = die_size
    if dw > 0 and dh > 0:
        rect = Rectangle((0, 0), dw, dh, fill=False, linestyle="-",
                         linewidth=1.5, edgecolor="#333333", alpha=0.5, zorder=0)
        ax.add_patch(rect)

    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.35)

    all_x: List[float] = [0.0, dw]
    all_y: List[float] = [0.0, dh]

    if source is not None:
        all_x.append(source[0])
        all_y.append(source[1])

    # Draw cluster bboxes
    if args.bbox and bboxes:
        for cid, (xmin, ymin, xmax, ymax) in sorted(bboxes.items()):
            rect = Rectangle(
                (xmin, ymin), xmax - xmin, ymax - ymin,
                fill=False, linestyle="--", linewidth=1.0,
                edgecolor="#718096", alpha=0.5, zorder=1,
            )
            ax.add_patch(rect)

    # Draw route polylines
    n_routes = len(routes)
    for ri, route in enumerate(routes):
        pts = clean_points(route.points)
        if len(pts) < 2:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        all_x.extend(xs)
        all_y.extend(ys)

        color = route_color(ri, n_routes)
        ax.plot(xs, ys, color=color, linewidth=1.2, alpha=0.72, zorder=2)

    # Draw buffer positions
    for buf in buffers:
        style = STYLE["buffer"]
        ax.scatter([buf.x], [buf.y],
                   marker=style["marker"], s=style["size"],
                   facecolors=style["face"], edgecolors=style["edge"],
                   linewidths=style["linewidth"], zorder=7)

    # Draw sink and source nodes
    node_order = ["sink", "internal", "access", "bridge", "top", "global", "unknown"]
    for cls in node_order:
        cls_nodes = [n for n in nodes.values() if n.cls == cls]
        if not cls_nodes:
            continue
        style = STYLE.get(cls, STYLE["unknown"])
        xs = [n.x for n in cls_nodes]
        ys = [n.y for n in cls_nodes]
        ax.scatter(xs, ys, marker=style["marker"], s=style["size"],
                   facecolors=style["face"], edgecolors=style["edge"],
                   linewidths=style["linewidth"], zorder=5, label=cls)
        all_x.extend(xs)
        all_y.extend(ys)

    if source is not None:
        style = STYLE["source"]
        ax.scatter([source[0]], [source[1]],
                   marker=style["marker"], s=style["size"],
                   facecolors=style["face"], edgecolors=style["edge"],
                   linewidths=style["linewidth"], zorder=6, label="source")

    # Axis limits
    x_min = min(all_x) if all_x else 0.0
    x_max = max(all_x) if all_x else 1.0
    y_min = min(all_y) if all_y else 0.0
    y_max = max(all_y) if all_y else 1.0
    pad_x = max(2.0, (x_max - x_min) * 0.06)
    pad_y = max(2.0, (y_max - y_min) * 0.06)
    ax.set_xlim(x_min - pad_x, x_max + pad_x)
    ax.set_ylim(y_min - pad_y, y_max + pad_y)

    # Legend
    legend_handles: List[Line2D] = []
    for cls in node_order + ["source", "buffer"]:
        style = STYLE.get(cls)
        if not style:
            continue
        legend_handles.append(
            Line2D([0], [0], marker=style["marker"], color="none",
                   markerfacecolor=style["face"], markeredgecolor=style["edge"],
                   markeredgewidth=style["linewidth"], markersize=8, label=cls))
    if args.bbox and bboxes:
        legend_handles.append(
            Line2D([0], [0], color="#718096", linestyle="--", linewidth=1.0,
                   label="cluster bbox"))
    legend_handles.append(
        Line2D([0], [0], color="#333333", linewidth=1.5, label="die boundary"))

    ax.legend(handles=legend_handles, loc="best", fontsize=7.5,
              framealpha=0.92, ncol=2)

    fig.savefig(out_path, dpi=220)
    if args.show:
        plt.show()
    plt.close(fig)


def main() -> None:
    args = parse_args()
    if args.show:
        plt.switch_backend("TkAgg")

    paths = infer_paths(args.sample_id)

    print(f"[VRESULT] solution file: {paths['solution']}")
    print(f"[VRESULT] loc file: {paths['loc']}")

    buffers, routes = parse_solution_file(paths["solution"])
    print(f"[VRESULT] parsed buffers: {len(buffers)}")
    print(f"[VRESULT] parsed routes: {len(routes)}")

    nodes = parse_loc_file(paths["loc"])
    print(f"[VRESULT] parsed nodes: {len(nodes)}")

    source = parse_sample_source(paths["sample"])
    if source is None:
        warn("source location not found in sample file")
    else:
        print(f"[VRESULT] source: ({source[0]:.0f}, {source[1]:.0f})")

    die_size = parse_die(paths["sample"])
    print(f"[VRESULT] die: {die_size[0]:.0f} x {die_size[1]:.0f}")

    bboxes = compute_cluster_bboxes(nodes)

    plot_solution(
        nodes=nodes,
        routes=routes,
        buffers=buffers,
        bboxes=bboxes,
        source=source,
        die_size=die_size,
        out_path=paths["out"],
        args=args,
    )

    print(f"[VRESULT] output: {paths['out']}")


if __name__ == "__main__":
    main()
