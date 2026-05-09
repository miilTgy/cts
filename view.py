import os
import re
import sys
from typing import Dict, List, Optional, Tuple


Coord = Tuple[int, int]
Grid = Optional[Tuple[int, int]]
Buffer = Tuple[str, str, int, int]
RouteMap = Dict[str, List[Coord]]


INT_RE = re.compile(r"^[+-]?\d+$")


def warn(message: str) -> None:
    print(f"Warning: {message}", file=sys.stderr)


def fail(message: str) -> None:
    print(f"Error: {message}", file=sys.stderr)
    sys.exit(1)


def is_int_token(token: str) -> bool:
    return INT_RE.fullmatch(token) is not None


def extract_ints(line: str) -> List[int]:
    """Return integer tokens from a whitespace-split line."""
    return [int(token) for token in line.split() if is_int_token(token)]


def parse_sample(path: str):
    """Return grid info, source coordinate, and sink dictionary."""
    grid: Grid = None
    source: Optional[Coord] = None
    sinks: Dict[str, Coord] = {}

    if not os.path.exists(path):
        warn(f"input file does not exist: {path}")
        return grid, source, sinks

    with open(path, "r") as f:
        for line_no, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            keyword = parts[0].upper()
            ints = extract_ints(line)

            if keyword in ("DIE", "GRID"):
                if len(ints) >= 2:
                    grid = (ints[0], ints[1])
                else:
                    warn(f"{path}:{line_no}: {parts[0]} line has fewer than 2 integers")
            elif keyword == "SOURCE":
                if len(ints) >= 2:
                    source = (ints[-2], ints[-1])
                else:
                    warn(f"{path}:{line_no}: SOURCE line has fewer than 2 integers")
            elif keyword == "SINK":
                if len(ints) < 2:
                    warn(f"{path}:{line_no}: SINK line has fewer than 2 integers")
                    continue

                if len(parts) >= 4 or (len(parts) >= 2 and not is_int_token(parts[1])):
                    sink_id = parts[1]
                else:
                    sink_id = f"S{len(sinks)}"

                sinks[sink_id] = (ints[-2], ints[-1])

    return grid, source, sinks


def parse_solution(path: str):
    """Return buffer list and route dictionary."""
    buffers: List[Buffer] = []
    routes: RouteMap = {}

    if not os.path.exists(path):
        warn(f"output file does not exist: {path}")
        return buffers, routes

    with open(path, "r") as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line_no = i + 1
        line = lines[i].strip()
        i += 1
        if not line or line.startswith("#"):
            continue

        parts = line.split()
        keyword = parts[0].upper()

        if keyword == "BUF":
            ints = extract_ints(line)
            if len(parts) < 5 or len(ints) < 2:
                warn(f"{path}:{line_no}: malformed BUF line")
                continue
            buffers.append((parts[1], parts[2], ints[-2], ints[-1]))
        elif keyword == "ROUTE":
            if len(parts) < 3:
                warn(f"{path}:{line_no}: ROUTE line expects sink id and point count")
                continue

            sink_id = parts[1]
            try:
                point_count = int(parts[2])
            except ValueError:
                warn(f"{path}:{line_no}: ROUTE point count is not an integer: {parts[2]}")
                continue

            points: List[Coord] = []
            for point_index in range(point_count):
                if i >= len(lines):
                    warn(
                        f"{path}:{line_no}: ROUTE {sink_id} expected {point_count} "
                        f"points, got {point_index}"
                    )
                    break

                point_line_no = i + 1
                point_line = lines[i].strip()
                i += 1
                ints = extract_ints(point_line)
                if len(ints) < 2:
                    warn(f"{path}:{point_line_no}: malformed route point for {sink_id}")
                    continue
                points.append((ints[0], ints[1]))

            routes[sink_id] = points

    return buffers, routes


def validate_routes(source, sinks, routes):
    """Print route-format and endpoint warnings."""
    for sink_id, points in routes.items():
        if not points:
            warn(f"route {sink_id} has no points")
            if sink_id not in sinks:
                warn(f"solution references unknown sink id: {sink_id}")
            continue

        if source is not None and points[0] != source:
            warn(f"route {sink_id} starts at {points[0]}, expected source {source}")

        if sink_id not in sinks:
            warn(f"solution references unknown sink id: {sink_id}")
        elif points[-1] != sinks[sink_id]:
            warn(f"route {sink_id} ends at {points[-1]}, expected sink {sinks[sink_id]}")

        for index in range(1, len(points)):
            prev = points[index - 1]
            cur = points[index]
            if cur == prev:
                warn(f"route {sink_id} has duplicate adjacent point at index {index}: {cur}")
            elif cur[0] != prev[0] and cur[1] != prev[1]:
                warn(
                    f"route {sink_id} segment {index - 1}->{index} is not rectilinear: "
                    f"{prev} -> {cur}"
                )


def collect_points(source, sinks, buffers, routes) -> List[Coord]:
    points: List[Coord] = []
    if source is not None:
        points.append(source)
    points.extend(sinks.values())
    points.extend((x, y) for _, _, x, y in buffers)
    for route_points in routes.values():
        points.extend(route_points)
    return points


def add_labels(ax, points_with_labels, dx: float, dy: float, fontsize: int = 8) -> None:
    for x, y, label in points_with_labels:
        ax.text(x + dx, y + dy, label, fontsize=fontsize, zorder=5)


def plot_cts(sample_id, grid, source, sinks, buffers, routes):
    """Draw the CTS routing visualization."""
    all_points = collect_points(source, sinks, buffers, routes)
    if not all_points and grid is None:
        fail("no drawable CTS data found")

    if "MPLCONFIGDIR" not in os.environ:
        try:
            os.makedirs("/tmp/matplotlib", exist_ok=True)
            os.environ["MPLCONFIGDIR"] = "/tmp/matplotlib"
        except OSError:
            pass

    if "MPLBACKEND" not in os.environ:
        os.environ["MPLBACKEND"] = "Agg"

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11, 8), layout="constrained")

    route_label_used = False
    for points in routes.values():
        if not points:
            continue
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        label = "Route" if not route_label_used else None
        ax.plot(xs, ys, color="black", linewidth=2.0, alpha=0.75, label=label, zorder=1)
        route_label_used = True

    if source is not None:
        ax.scatter(
            [source[0]],
            [source[1]],
            marker="*",
            s=180,
            color="red",
            edgecolors="black",
            linewidths=0.6,
            label="Source",
            zorder=4,
        )
        add_labels(ax, [(source[0], source[1], "SRC")], 1.5, 1.5, fontsize=9)

    if sinks:
        xs = [coord[0] for coord in sinks.values()]
        ys = [coord[1] for coord in sinks.values()]
        ax.scatter(xs, ys, marker="o", s=46, color="royalblue", label="Sink", zorder=3)
        add_labels(ax, [(x, y, sink_id) for sink_id, (x, y) in sinks.items()], 1.5, 1.5)

    if buffers:
        xs = [x for _, _, x, _ in buffers]
        ys = [y for _, _, _, y in buffers]
        ax.scatter(xs, ys, marker="s", s=58, color="forestgreen", label="Buffer", zorder=3)
        add_labels(
            ax,
            [(x, y, f"{buf_id}:{buf_type}") for buf_id, buf_type, x, y in buffers],
            1.5,
            1.5,
        )

    if grid is not None:
        width, height = grid
        if all_points:
            xs = [p[0] for p in all_points]
            ys = [p[1] for p in all_points]
            min_x, max_x = min(0, min(xs)), max(width, max(xs))
            min_y, max_y = min(0, min(ys)), max(height, max(ys))
        else:
            min_x, max_x = 0, width
            min_y, max_y = 0, height
    else:
        xs = [p[0] for p in all_points]
        ys = [p[1] for p in all_points]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

    span_x = max(1, max_x - min_x)
    span_y = max(1, max_y - min_y)
    margin = max(2.0, 0.04 * max(span_x, span_y))
    ax.set_xlim(min_x - margin, max_x + margin)
    ax.set_ylim(min_y - margin, max_y + margin)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.5, alpha=0.45)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(f"CTS Routing Visualization: sample{sample_id}")
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, loc="best")

    backend = plt.get_backend().lower()
    if "agg" in backend or "pdf" in backend or "svg" in backend:
        output_path = f"result/sample{sample_id}_view.png"
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        fig.savefig(output_path, dpi=160)
        print(f"saved visualization: {output_path}")
    else:
        plt.show()


def print_usage() -> None:
    print("Usage: python3 view.py <sample_id>")


def main():
    """Parse command line argument and run visualization."""
    if len(sys.argv) != 2:
        print_usage()
        sys.exit(1)

    try:
        sample_id = int(sys.argv[1])
    except ValueError:
        print_usage()
        sys.exit(1)

    sample_path = f"samples/sample{sample_id}.txt"
    solution_path = f"result/sample{sample_id}_solution.txt"

    grid, source, sinks = parse_sample(sample_path)
    buffers, routes = parse_solution(solution_path)
    validate_routes(source, sinks, routes)
    plot_cts(sample_id, grid, source, sinks, buffers, routes)


if __name__ == "__main__":
    main()
