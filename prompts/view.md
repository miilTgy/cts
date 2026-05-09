

# view.py Prompt: CTS Routing Result Visualizer

Write a Python script `view.py` to visualize CTS routing results.

## How to run

```bash
python3 view.py 0
```

This command visualizes `sample0` by reading:

- input file: `samples/sample0.txt`
- output file: `result/sample0_solution.txt`

Similarly:

```bash
python3 view.py 1
python3 view.py 2
```

should visualize `sample1`, `sample2`, etc.

## Required behavior

The script must parse the original sample input and the solution output, then draw a 2D grid visualization showing:

1. source location
2. all sink locations
3. all inserted buffers
4. all rectilinear routes from source to each sink

Use `matplotlib` for visualization.

## Input file format

The script should read the corresponding input file from:

```text
samples/sample<k>.txt
```

The parser should be tolerant to the exact homework input style. It must extract at least:

- grid width / height, if present
- source coordinate
- sink ids and coordinates

Expected information may appear in forms similar to:

```text
GRID <width> <height>
SOURCE <x> <y>
SINK <sink_id> <x> <y>
```

or equivalent homework-style variants.

The script should implement robust line-based parsing:

- ignore blank lines
- ignore comment lines beginning with `#`
- split each line by whitespace
- recognize keywords case-insensitively
- extract all integer tokens from relevant lines

If grid size is not found, infer plotting bounds from source, sink, buffer, and route coordinates.

## Output solution format

The script should read the corresponding output file from:

```text
result/sample<k>_solution.txt
```

The output format is:

```text
NUM_BUFS <K>
BUF <buf_id> <type> <x> <y>
...

NUM_ROUTES <N>
ROUTE <sink_id> <P>
<x_0> <y_0>
<x_1> <y_1>
...
<x_{P-1}> <y_{P-1}>
```

Where:

- `P` is the number of polyline points in the route.
- For every two consecutive points, either x coordinates are equal or y coordinates are equal.
- The first point should be the source coordinate.
- The last point should be the target sink coordinate.

The parser should:

- read `NUM_BUFS`
- parse all `BUF` entries
- read `NUM_ROUTES`
- parse every `ROUTE <sink_id> <P>` block
- store each route as a list of `(x, y)` points

## Visualization requirements

The generated figure should show:

### Source

- Draw source as a red star marker.
- Label it as `SRC`.

### Sinks

- Draw sinks as blue circle markers.
- Label each sink using its sink id, for example `L0`, `R3`, `T1`, etc.

### Buffers

- Draw buffers as green square markers.
- Label each buffer using `<buf_id>:<type>`, for example `B0:BUF_SMALL`.

### Routes

- Draw each route as a black rectilinear polyline.
- Each consecutive pair of points should be connected by one horizontal or vertical segment.
- Use a reasonably visible line width.
- It is acceptable for multiple routes to overlap.

### Grid and axis

- Show grid lines.
- Use equal aspect ratio.
- Set x/y limits to include all source, sink, buffer, and route points, with a small margin.
- If grid width / height is parsed from the input, use it to set the default plotting area.
- Add title: `CTS Routing Visualization: sample<k>`.
- Add legend entries for source, sink, buffer, and route.

## Diagnostics and validation

The script should print useful warnings, but still try to visualize whenever possible.

Warnings to implement:

1. route does not start at the parsed source coordinate
2. route does not end at the coordinate of the declared sink id
3. consecutive route points are not rectilinear
4. duplicate adjacent route points
5. solution references a sink id that is not found in the input
6. input file or output file does not exist

Do not stop immediately on minor route warnings. Continue drawing the route so debugging is easy.

## Code structure

Implement the script with clear functions:

```python
def parse_sample(path):
    """Return grid info, source coordinate, and sink dictionary."""


def parse_solution(path):
    """Return buffer list and route dictionary."""


def validate_routes(source, sinks, routes):
    """Print route-format and endpoint warnings."""


def plot_cts(sample_id, grid, source, sinks, buffers, routes):
    """Draw the CTS routing visualization."""


def main():
    """Parse command line argument and run visualization."""
```

Use only standard Python libraries plus `matplotlib`.

## Command line interface

The script should require exactly one argument:

```bash
python3 view.py <sample_id>
```

Examples:

```bash
python3 view.py 0
python3 view.py 12
```

If the argument is missing or not an integer, print usage information:

```text
Usage: python3 view.py <sample_id>
```

## File path rules

Assume `view.py` is run from the project root directory, where the following folders exist:

```text
samples/
result/
```

Construct paths as:

```python
sample_path = f"samples/sample{sample_id}.txt"
solution_path = f"result/sample{sample_id}_solution.txt"
```

## Implementation notes

- Make parsing robust rather than overfitted to one sample.
- Use helper function `extract_ints(line)` to get integer tokens from a line.
- Preserve sink ids as strings.
- Preserve buffer ids and buffer types as strings.
- For route drawing, use:

```python
xs = [p[0] for p in points]
ys = [p[1] for p in points]
ax.plot(xs, ys, linewidth=2)
```

- Do not simplify or modify routes while visualizing.
- The purpose of this script is debugging the exact solution output, so it must draw the polyline exactly as written in `result/sample<k>_solution.txt`.