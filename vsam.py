import os
import sys
from typing import Dict, List, Optional, Tuple

Coord = Tuple[int, int]
Grid = Optional[Tuple[int, int]]


def warn(message: str) -> None:
    print(f"Warning: {message}", file=sys.stderr)


def fail(message: str) -> None:
    print(f"Error: {message}", file=sys.stderr)
    sys.exit(1)


def parse_sample(path: str):
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

            if keyword in ("DIE", "GRID"):
                if len(parts) >= 3:
                    try:
                        grid = (int(parts[1]), int(parts[2]))
                    except ValueError:
                        warn(f"{path}:{line_no}: DIE line has non-integer dimensions")
                else:
                    warn(f"{path}:{line_no}: DIE line has fewer than 3 tokens")
            elif keyword == "SOURCE":
                if len(parts) == 3:
                    try:
                        source = (int(parts[1]), int(parts[2]))
                    except ValueError:
                        warn(f"{path}:{line_no}: SOURCE line has non-integer coords")
                elif len(parts) >= 4:
                    try:
                        source = (int(parts[2]), int(parts[3]))
                    except ValueError:
                        warn(f"{path}:{line_no}: SOURCE line has non-integer coords")
                else:
                    warn(f"{path}:{line_no}: SOURCE line has fewer than 3 tokens")
            elif keyword == "SINK":
                if len(parts) < 4:
                    warn(f"{path}:{line_no}: SINK line too short")
                    continue
                sink_id = parts[1]
                try:
                    sinks[sink_id] = (int(parts[2]), int(parts[3]))
                except ValueError:
                    warn(f"{path}:{line_no}: SINK line has non-integer coords")

    return grid, source, sinks


def plot_sample(sample_id: int, grid: Grid, source: Optional[Coord], sinks: Dict[str, Coord]) -> None:
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
        ax.annotate("SRC", (source[0], source[1]),
                     textcoords="offset points", xytext=(6, 6), fontsize=9)

    if sinks:
        xs = [coord[0] for coord in sinks.values()]
        ys = [coord[1] for coord in sinks.values()]
        ax.scatter(xs, ys, marker="o", s=46, color="royalblue", label="Sink", zorder=3)
        for sink_id, (x, y) in sinks.items():
            ax.annotate(sink_id, (x, y),
                        textcoords="offset points", xytext=(6, 6), fontsize=6)

    if grid is not None:
        width, height = grid
        all_xs = []
        all_ys = []
        if source is not None:
            all_xs.append(source[0])
            all_ys.append(source[1])
        for x, y in sinks.values():
            all_xs.append(x)
            all_ys.append(y)
        all_xs.append(0)
        all_xs.append(width)
        all_ys.append(0)
        all_ys.append(height)
    else:
        fail("no grid dimensions found in sample file")

    span_x = max(1, max(all_xs) - min(all_xs))
    span_y = max(1, max(all_ys) - min(all_ys))
    margin = max(2.0, 0.04 * max(span_x, span_y))
    ax.set_xlim(min(all_xs) - margin, max(all_xs) + margin)
    ax.set_ylim(min(all_ys) - margin, max(all_ys) + margin)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.5, alpha=0.45)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(f"CTS Sample {sample_id}  (DIE {width}x{height})")
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, loc="best")

    backend = plt.get_backend().lower()
    if "agg" in backend or "pdf" in backend or "svg" in backend:
        output_path = f"result/sample{sample_id}_vsam.png"
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        fig.savefig(output_path, dpi=160)
        print(f"saved visualization: {output_path}")
    else:
        plt.show()


def print_usage() -> None:
    print("Usage: python3 vsam.py <sample_id>")


def main() -> None:
    if len(sys.argv) != 2:
        print_usage()
        sys.exit(1)

    try:
        sample_id = int(sys.argv[1])
    except ValueError:
        print_usage()
        sys.exit(1)

    sample_path = f"samples/sample{sample_id}.txt"
    grid, source, sinks = parse_sample(sample_path)
    plot_sample(sample_id, grid, source, sinks)


if __name__ == "__main__":
    main()
