# Topology for Robotic Motion Planning

**TopoPlan** is a dependency-free Python toolkit for educational 2D motion planning. It combines collision-checked grid A*, conservative circular-robot clearance, and alternative routes distinguished by topological branch-cut signatures.

## Research benchmark extension

An additive experiment layer now provides independently checked route diversity,
formal finite-search outcomes, clearance sweeps, continuous RRT, optional real
OMPL PRM/RRT/RRT* baselines, moving-obstacle planning, prioritized multi-robot
scheduling and replay-verifiable benchmark bundles. The legacy CLI and Grid API
below remain unchanged. See [the experiment contracts and research boundaries](docs/MOTION_BENCHMARKS.md).

```sh
python -m topoplan.benchmark run examples/benchmark.json --output benchmark-review
python -m topoplan.benchmark verify benchmark-review
```


## Quick start

Requires Python 3.10 or newer. From the repository root:

```bash
python -m topoplan examples/obstacle.json --routes 2 --output output/report.json --svg output/routes.svg
python -m topoplan examples/disconnected.json
```

The first command finds routes on opposite sides of an obstacle. It prints JSON and exports a report and standalone SVG. Open the SVG in a browser: dark cells are obstacles, pale red areas show conservative obstacle expansion, and colored lines show robot-center paths. Green and red dots mark start and goal; their dot size is decorative, not robot radius. SVG coordinates increase rightward and downward.

The disconnected example returns a JSON report with no routes and exit status 2. Exit status is 0 if at least one route is found, even if fewer routes than requested are returned. Inspect `status` and the route count for completeness. Invalid inputs also exit with status 2, with an error on stderr. Output parent directories are created; existing files are replaced.

Install the package to use the `topoplan` command:

```bash
python -m pip install .
topoplan --help
```

## Input format

```json
{
  "grid": [".......", "...#...", "...#...", "...#...", "......."],
  "start": [0, 2],
  "goal": [6, 2],
  "radius": 0.2
}
```

Each `.` is free and each `#` is an occupied **closed unit square**. Rows have equal length. Start and goal are integer `[column, row]` indices, mapped to world-space cell centers `(column + 0.5, row + 0.5)`. `radius` is a nonnegative circular-robot radius in cell-width units; default is zero. The grid boundary is a wall. Boundary or obstacle contact is rejected.

## Python API

```python
from topoplan import Grid

grid = Grid([".......", "...#...", "...#...", "...#...", "......."], radius=0.2)
report = grid.plan(start=[0, 2], goal=[6, 2], routes=2)
for route in report["routes"]:
    print(route["length"], route["signature"], route["path"])
```

`Grid.clear_segment(a, b)` checks a straight segment between cell centers, including its interior. `Grid.signature(path)` computes ray crossings for an ordered sequence of cell coordinates; it does not validate that sequence as collision-free. `Grid.svg(report)` exports the report's routes. `Grid.neighbors(cell)` yields valid four-neighbor moves.

## Algorithms and guarantees within the model

| Feature | Implementation and scope |
| --- | --- |
| Single route | A* on a four-neighbor grid with unit costs and Manhattan heuristic; shortest in the resulting clearance-checked grid graph |
| Collision checks | Segment–box slab intersection, including endpoints and contact |
| Circular-robot clearance | Expand each occupied square by radius in both axes and shrink workspace bounds |
| Alternative routes | A* on `(cell, signature)` states; retain the shortest reached route per signature |
| Signatures | Signed crossings of rightward horizontal rays from one interior marker per four-connected occupied component |
| Search status | Distinguishes requested routes found, bounded state-space exhaustion, and expansion limit |

The axis-aligned expansion **contains** the true disk Minkowski sum, so clearance is conservative near obstacle corners: it may reject feasible circular-robot paths. It does not intentionally admit disk collisions under the ideal exact geometric model. Floating-point arithmetic is used; this is not a certified geometric predicate. Plans assume the supplied map and radius are accurate.

Every movement is a unit horizontal or vertical segment. Routes are robot-center waypoints with length in cell units. No diagonals, smoothing, orientation, velocity, acceleration, turning-radius constraints, manipulators, or dynamics are modeled. Grid failure does not prove that no path exists in continuous configuration space.

## What the signatures mean

An open route's raw signature depends on marker rays and endpoints. Comparing two routes with the **same endpoints**, the difference is the winding signature of their concatenated closed loop (up to orientation convention). Different signatures distinguish some nondeformable alternatives around occupied regions. Positive direction follows increasing grid y, which points down in the visualization.

These integer vectors are homology-style summaries, **not complete homotopy classifiers**. Routes with the same signature can still belong to different homotopy classes when several obstacles are present. Occupied components and their markers are derived from the original grid, not recomputed from clearance-expanded geometry; markers can be redundant if obstacles touch walls or expansions merge. The method does not identify the full fundamental group.

For multiple routes, each signature coordinate is bounded to −1, 0, or 1 **throughout the search**. This limits winding excursions and makes the search finite. Consequently a globally shortest path in a desired unrestricted class may be excluded. Returned alternatives can include repeated vertices and deliberate loops; they are not guaranteed to be simple paths. A request for additional routes can exhaust the bounded state graph with fewer routes found, even when additional unrestricted winding classes exist.

A single-route request uses ordinary A* without this signature bound. With identical start and goal, its shortest route has length zero.

## Limits and operational scope

At most 10,000 grid cells and 2,000 occupied cells. Alternative search supports up to six obstacle components and requests of 1–10 routes. `--max-expansions` defaults to 100,000 and can range from 1 to 200,000. Dense maps and signature-augmented searches can be expensive. An expansion-limit result does not prove unreachability; inspect the status before interpreting an empty route list.

This is an offline planner for static maps, not a robot controller. It does not connect to hardware, execute trajectories, account for localization error or moving obstacles, or certify real-world safety.

## Development and verification

```bash
python -m unittest discover -s tests -v
```

The original planner tests cover shortest paths against an independent breadth-first search on all 128 obstacle patterns of a 3×3 map with fixed free endpoints, opposite-side signatures and route clearance, narrow passages at different robot radii, segment collisions, blocked goals, expansion limits, signature algebra, input validation, and CLI JSON/SVG exports. CI installs the package and runs tests and the two-route example on Python 3.10, 3.12, and 3.13.

Licensed under Apache 2.0; see [LICENSE](LICENSE).

## ResearchOS / TECT concept integration

The [ResearchOS / Topological Epistemic Control Theory technical concept paper](docs/researchos/TECT_WHITE_PAPER.md) proposes an epistemic control and assurance infrastructure for AI-generated knowledge. Read the [repository integration status](docs/researchos/INTEGRATION.md) and [machine-readable scope](docs/researchos/integration.json) for the boundary between the concept and implemented capabilities. Performance targets are research objectives, not achieved results. This documentation addition does not implement the complete ResearchOS architecture or change operational authority.

## ResearchOS / TECT assurance layer

This repository includes the shared v0.1 research controller, a scoped executable domain pilot, replayable evidence/decision bundles, adversarial regression probes, and explicit unassured outcomes. See [the workflow](docs/RESEARCHOS.md), [versioned semantics](docs/TECT_SPEC.md), and [research-program traceability](docs/TECT_RESEARCH_PLAN.md). Existing domain results retain their documented limits. No claim state authorizes external or clinical execution.

## Portfolio implementation boundary

See [current implementation and shared-core integration](docs/portfolio/INTEGRATION.md) and [machine-readable status](docs/portfolio/STATUS.json). Independent empirical reproduction and real-provider acceptance remain separate from software checks.
