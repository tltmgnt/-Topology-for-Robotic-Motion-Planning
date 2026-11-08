# Topological diversity and explicit termination evidence

This additive experiment layer belongs to **Topology for Robotic Motion Planning**,
not the separate **Topology for Geographic and Spatial System** repository. The
existing `Grid`, A*, report format, `topoplan` CLI and ResearchOS domain check are
preserved. New commands use `topoplan-bench` / `python -m topoplan.benchmark`.

## What is implemented

| Priority | Implementation | Boundary |
|---|---|---|
| Topological diversity | Independent signature metrics and grouped repeated-trial diversity | Homology-style signatures, not a complete homotopy classifier |
| Termination guarantees | Deterministic replay plus independent finite-graph BFS | Only the specified graph/signature/horizon model |
| Clearance sensitivity | Radius sweeps, route surface clearance, corner-conservatism witnesses | Cell-center witnesses do not prove a continuous route exists |
| OMPL comparison | Real PRM, RRT and RRT* through optional OMPL 2.x Python bindings | Geometric 2D translation; no high-DoF or kinodynamic claims |
| Continuous space | Floating-point 2D endpoints, bounded seeded RRT and shared continuous collision checks | Same expanded-square obstacles; no smoothing, orientation or dynamics |
| Moving obstacles | Time-expanded A*, waiting, continuously swept disk checks | Supplied piecewise-linear motion, finite synchronized horizon |
| Multiple robots | Fixed-priority sequential planning with trajectory reservations | Not StAC, not complete joint planning, not joint optimality |
| Reproducibility | Captured receipts, offline independent analysis, exact bundle verification | Native searches/timing are observations, not deterministic replay |

The core stays dependency-free. The only optional package is installed with:

```sh
python -m pip install '.[ompl]'
```

OMPL is never imported to verify a captured bundle. Native planners run in isolated
processes so a backend timeout or crash does not become a false no-path claim.

## Run a study

```sh
python -m pip install .
python -m topoplan.benchmark generate --output study.json
python -m topoplan.benchmark run study.json --output review
python -m topoplan.benchmark verify review
```

The default study has 20 experiments: three static maps (an island obstacle, a
narrow passage and disconnected regions), three radii, A* and continuous RRT,
one moving-obstacle case, and one two-robot swap. Some tasks intentionally fail
or have invalid endpoints at larger radii; a completed experiment is not a claim
that all tasks have a solution.

Add `--include-ompl` to `generate` for a 47-experiment study including PRM, RRT and
RRT*. `examples/benchmark.json` provides a small four-experiment introduction.

All new destinations are created exclusively. Existing outputs are preserved;
parents must exist. New benchmark CLI exit codes are **0** for a successfully
captured/verified experiment bundle and **2** for malformed input or failed bundle
verification. Read each experiment's termination for planning outcomes. The old
`topoplan` CLI retains its existing exit behavior.

## Study contract

The top-level object has exactly `schema_version: 1` and `experiments` (1–64
objects). IDs are unique trimmed strings of 1–100 characters. Unknown fields,
booleans where numbers are expected, nonfinite values and duplicate JSON keys
are rejected. Input/bundle files are regular, nonsymlink UTF8 JSON, at most 8 MiB,
with nesting at most 64. Bundle directories and their file set are checked.

A static experiment:

```json
{
  "id": "island-small-radius",
  "kind": "static",
  "grid": [".......", "...#...", "...#...", "...#...", "......."],
  "start": [0, 2],
  "goal": [6, 2],
  "radius": 0.2,
  "planner": "astar",
  "routes": 2,
  "max_expansions": 10000,
  "iterations": 500,
  "seed": 7,
  "seconds": 0.15,
  "coordinate_mode": "cells"
}
```

Required keys: `id`, `kind`, `grid`, `start`, `goal`. Defaults for the other keys:
radius 0, planner `astar`, routes 2, max_expansions 100000, iterations 1000, seed 0,
seconds 0.2, coordinate_mode `cells`. Planners are `astar`, `rrt`, `ompl:PRM`,
`ompl:RRT`, `ompl:RRTstar`. Start/goal use integer cell coordinates by default;
all continuous backends receive the **same cell-center endpoints**. Continuous
backends also accept `coordinate_mode: world` and floating-point workspace
coordinates. A* requires cells. Workspace origin is top-left; y increases down.

The benchmark limits grids to 2500 cells / 500 occupied cells. Alternative-route
studies permit at most six occupied components. `routes` is 1–10, expansion budgets
1–200000, RRT iterations 1–10000, seeds 0–2^32−1, and native solve seconds
0.001–10. The sum of squared RRT iteration budgets may not exceed 25 million;
its simple nearest-neighbor implementation is intentionally bounded.

Each independent graph proof has at most 200000 potential states and a study at
most one million. Static augmented A* uses `cells * 3^marker_count`; ordinary
A* uses cells. Timed cases use `cells * (horizon+1)` per robot. Limits are enforced
before running the study. They bound finite experiments, not production-scale
performance.

## Guarantees and metrics

| Termination | Supported conclusion |
|---|---|
| `requested_routes_found` | Requested number of valid distinct signatures returned |
| `bounded_search_exhausted` | Original finite A* state graph exhausted; independent BFS checks all reachable goal signatures |
| `expansion_limit` | Search stopped at its expansion budget; does not establish no path |
| `iteration_limit` | Continuous RRT stopped at its sample-iteration budget; does not establish no path |
| `time_or_attempt_limit` | Native OMPL attempt/solve allocation ended; does not establish no path |
| `horizon_exhausted` | No goal-holding path in the declared timed graph through this horizon |
| `prioritized_failure` | Some robot failed given earlier robots' fixed reservations; no joint infeasibility claim |
| `invalid_input` | Structurally valid experiment had invalid planning parameters/endpoints; no planning conclusion |
| `backend_unavailable` / `backend_error` / `backend_timeout` | Native baseline unavailable or failed; never interpreted as no path |

The original A* state graph is not changed. Multiple routes use coordinates in
[−1,1] for every signature component **at every prefix**, not just at the goal.
Independent BFS uses separate orientation-based segment/rectangle collision tests
and separately constructed ray-crossing transitions. Returned grid lengths are
checked against independent shortest distances for the appropriate bounded
signature. A single route is optimal only in the four-neighbor graph.

Reports include route length, minimum physical disk surface clearance, signature,
number of distinct signatures, and number of distinct waypoint sequences. The
last is a precise representation-level count, not an epsilon-based geometric
trajectory equivalence relation. Repeated experiments are grouped only when map,
endpoints, coordinate convention, radius and planner match. Change seeds and IDs
to collect repeated trials. Group metrics do not count homotopy classes.

Surface clearance is the minimum Euclidean distance from the center path to the
original closed occupied squares or workspace wall, minus robot radius. Production
collision geometry remains conservative square expansion. Thus an independently
valid route can have more physical clearance than this conservative geometry
requires. Sweeps also identify cell centers rejected by expanded squares although
the exact disk-to-original-square model would admit them. They are witnesses of
corner conservatism, not proofs of a feasible global path or model calibration.

## Timed obstacles and multiple robots

A dynamic experiment replaces static planner fields with optional `obstacles`,
`horizon` (0–128, default 32), and `max_expansions`:

```json
{
  "id": "crossing",
  "kind": "dynamic",
  "grid": [".....", ".....", "....."],
  "start": [0, 1], "goal": [4, 1], "radius": 0.2,
  "horizon": 8,
  "obstacles": [{"radius": 0.2, "path": [[2, 0], [2, 1], [2, 2]]}]
}
```

Obstacle waypoints are cell centers at integer ticks, with linear motion between
them and a stationary final position thereafter. Their path must be clear of
static geometry at their own radius; it need not use unit-speed neighbor moves.
At most 16 moving disks and horizon+1 waypoints each are supported.

The planning robot can move to four neighbors or wait per tick. Relative disk
motion is checked over the **entire interval**, so opposite edge swaps and
mid-segment collisions are rejected. Contact is collision. A solution must remain
at the goal through the declared horizon; occupancy beyond it is unassessed.
Independent timed verification uses relative-segment distance and a separate
layered reachability enumeration, checking the earliest feasible goal-holding
arrival. A collision-free timed path is not a dynamically feasible control law.

A `kind: multi` experiment replaces `start`/`goal` with `robots`, each having exactly
`id`, `start`, `goal`. One to four robots share the specified radius. Input order
is priority order; earlier paths become moving-disk reservations for later robots.
Starts may not overlap/touch. Returned paths are padded through the horizon, and
all later paths are independently checked against earlier ones. Priority failure
can occur even when another order or joint search would solve the problem.

## OMPL integration and comparison fairness

The tested native backend is **OMPL 2.0.1**. It uses RealVectorStateSpace(2), shared
workspace bounds, and a custom continuous MotionValidator calling the exact same
conservative expanded-square segment model as the grid/RRT backends. Returned
routes are separately checked using the independent geometry code. Approximate
solutions are excluded; the literal requested goal is appended only after checking
the final connecting segment.

Each native trial sets its seed in a fresh process, records the package version,
and allows at most `4 * routes` solve attempts within its time allocation to
collect distinct signatures. An RRT* result is **not** a proof of optimality.
Native solves are stochastic and time-sensitive even with the same seed.

The collision assumptions, maps, radii and endpoints are matched. Search spaces
and budget units differ: A* expansions, RRT iterations, and native wall-clock solve
time. Native construction may exceed a requested allocation slightly; a separate
process timeout bounds hangs. Captured `wall_seconds` includes dispatch, process
startup and planning, and excludes independent verification. It is not a matched
CPU budget or a fair raw-speed ranking. No performance superiority is claimed.
Use repeated trials and a separately designed timing protocol for comparative
performance research.

## Evidence and reproducibility

A review contains exactly `study.json`, `receipts.json`, `report.json`,
`review.html`, `manifest.json`. The captured study is normalized with explicit
defaults. Each receipt contains a planner result, measured wall time, Python version and
implementation. Native results also record their OMPL package version. Input,
receipt and engine-source-file hashes bind report content. These hashes identify
the local implementation; they do not authenticate its origin. The HTML is escaped and script-free.

Verification never reruns OMPL, downloads dependencies or contacts services. It
replays internal deterministic A*/RRT/timed planners, independently validates
routes and applicable finite-graph claims, recomputes all metrics and HTML, and
compares every byte. Keep the engine and Python environment for exact internal
replay. Timing observations and native termination receipts are captured evidence,
not independently authenticated observations or reconstructed runtime behavior.

Editing a report and its manifest cannot bypass recomputation. Replacing all
inputs/receipts and regenerating a bundle can create a new self-consistent study;
there are no signatures, trusted external measurements or source-authenticity
claims. Independent geometric checks still use floating-point arithmetic and
share the declared map/radius/marker model; they are not certified predicates or
an external algorithm audit. No bundle authorizes hardware or clinical use.

## Research basis and scope

Primary sources checked November 8, 2026:

- [The Open Motion Planning Library 2.0](https://arxiv.org/abs/2605.29301v2),
  Guo et al.; submitted May 28, revised July 30, 2026. Motivates modular baseline
  integration and reusable planning infrastructure.
- [Look as You Leap: Planning Simultaneous Motion and Perception for High-DoF Robots](https://www.kavrakilab.rice.edu/publications/meng_psprm_2026.html),
  Meng et al., IEEE Transactions on Robotics 42, August 2026. Provides context for
  perception-constrained dynamic planning; this module implements no neural
  surrogate, perception scoring or GPU planner.
- [Sampling-Based Motion Planning with Scene Graphs Under Perception Constraints](https://arxiv.org/abs/2603.03514v2),
  Meng et al.; March 2026 submission, November 2 revision. Scene-graph perception
  constraints are research context and are not implemented here.
- [Efficient Multi-Robot Motion Planning for Manifold-Constrained Manipulators by Randomized Scheduling and Informed Path Generation](https://www.kavrakilab.org/publications/weihang-manifoldpath2026.pdf),
  Guo, Kingston, Hang and Kavraki. Online publication February 9, 2026, journal issue
  April 2026. This implementation uses fixed-priority grid reservations, not StAC's
  randomized scheduling, manifold constraints or informed path generation.
- [OMPL Python bindings](https://ompl.kavrakilab.org/python.html) and
  [termination conditions](https://ompl.kavrakilab.org/plannerTerminationConditions.html)
  inform the actual native adapter contract.

The testable hypothesis is that topological summaries and explicit termination
semantics supply useful information beyond path length and runtime. This upgrade
makes those experiments executable and reviewable. It does not establish novelty,
validate that hypothesis, reproduce the cited studies, or imply collaboration or
endorsement by Kavraki or her group.

## Validation

```sh
python -m unittest discover -s tests -v
python tests/check_benchmark_installed.py
python tests/check_researchos_installed.py
```

Core CI runs without OMPL on Python 3.10/3.12/3.13. A separate Python 3.12 job
installs the native extra, exercises real PRM/RRT/RRT* and runs the 47-experiment
synthetic study. Native absence is an explicit skip only in the core test job.
