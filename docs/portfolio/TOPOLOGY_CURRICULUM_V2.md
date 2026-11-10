# Topology compatibility and curriculum v2 — -Topology-for-Robotic-Motion-Planning

Version 2 adapts the explicit-molecular-graph teaching pattern to this repository's actual role. This is documentation and a learning/decision framework; it introduces no mathematical engine, runtime dependency or operational permission.

## Repository decision

| Question | Repository-specific answer |
| --- | --- |
| Architectural role | specialized planning tool |
| Minimum useful mathematics | Configuration-space connectivity and bounded branch-cut route signatures |
| Relationships to preserve | Grid occupancy, robot clearance and path signatures |
| Concrete operation / capstone | Compare routes around an obstacle and distinguish a search expansion limit from proven unreachability |
| Necessity and scope boundary | Bounded signatures do not enumerate all unrestricted homotopy classes; offline planning does not authorize hardware execution |

The decision is a source-based recommendation, not measured proof that a new specialty improves this repository. Start with the existing implementation and compare any proposed abstraction against a simpler alternative.

## Six-criterion compatibility scorecard

Complete one scorecard per proposed candidate and per concrete operation. Leave unmeasured entries unknown. Scores are human judgments from 0 (no fit) to 5 (strong evidenced fit); unknown is not zero. Do not sum an incomplete card. A total out of 30 is a heuristic, not adoption approval, and feasibility or necessity can veto a high total.

| Criterion | Score | Required evidence |
| --- | --- | --- |
| Structural compatibility | Not yet scored | Map the repository objects and the relationships above without losing direction, labels, units or missingness. |
| Transformation support | Not yet scored | Demonstrate the capstone operation on accepted and rejected inputs. |
| Invariant preservation | Not yet scored | State the property preserved and supply a counterexample or perturbation. |
| Computational feasibility | Not yet scored | Measure size, time and memory on bounded fixtures; record limits. |
| Integration compatibility | Not yet scored | Verify existing schemas, APIs and neighboring contracts remain compatible. |
| Mathematical necessity | Not yet scored | Compare with configuration-space connectivity and bounded branch-cut route signatures and explain the additional capability, if any. |

## Curriculum

Prerequisites: basic programming, sets, functions and graph vocabulary. Work in six competency-gated sessions, approximately 60–90 minutes each; empirical evaluation is a separate research activity.

| Session | Learning objective | Exercise | Completion evidence |
| --- | --- | --- | --- |
| 1. Model | Identify grid occupancy, robot clearance and path signatures. | Draw a minimal valid example and one excluded relationship. | Annotated object/relation inventory with explicit assumptions. |
| 2. Contract | Distinguish schema validity from substantive correctness. | Trace validation and propose malformed, missing and boundary inputs. | Input-to-rejection table grounded in current source. |
| 3. Operation | Explain configuration-space connectivity and bounded branch-cut route signatures. | Compare routes around an obstacle and distinguish a search expansion limit from proven unreachability. | Hand trace compared with existing software, or a clearly labeled specification when no runtime exists. |
| 4. Preservation | State what a transformation must preserve. | Rename identifiers, reorder records, then alter one meaningful relation; predict each effect under the actual contract. | Separate invariant, expected change and unsupported transformation. |
| 5. Comparison | Justify mathematical necessity and cost. | Complete the six-criterion card for the existing approach and one proposed alternative; record unknowns honestly. | Evidence-backed recommendation with baseline and explicit veto conditions. |
| 6. Reproduction | Communicate the bounded result. | Reproduce the capstone using existing offline fixtures and record software version, inputs, outputs and limitations. | Reviewable notebook or report distinguishing computation, replay, independent evaluation and authorization. |

## Source-linked reading laboratory

The table is a guided selection of source lines, not an assertion that the whole repository is covered. Line numbers refer to the inspected source blob `2af9a492b4c21867f107faa2b748d07f77299222` in [topoplan/core.py](../../topoplan/core.py). Recheck anchors after source changes.

| Line | Statement | Learning check |
| --- | --- | --- |
| [7](../../topoplan/core.py#L7) | `class Grid:` | State the input/output contract and trace one normal and one failing case. |
| [9](../../topoplan/core.py#L9) | `def __init__(self, rows, radius=0.0):` | State the input/output contract and trace one normal and one failing case. |
| [11](../../topoplan/core.py#L11) | `raise ValueError('grid must contain nonempty string rows')` | Construct the triggering input and explain why rejection preserves the contract. |
| [14](../../topoplan/core.py#L14) | `raise ValueError('Grid must be rectangular, contain only . and #, and have at most 10000 cells')` | Construct the triggering input and explain why rejection preserves the contract. |
| [16](../../topoplan/core.py#L16) | `raise ValueError('radius must be a finite nonnegative number')` | Construct the triggering input and explain why rejection preserves the contract. |
| [19](../../topoplan/core.py#L19) | `if len(self.blocked)>2000: raise ValueError('At most 2000 obstacle cells are supported')` | Construct the triggering input and explain why rejection preserves the contract. |
| [32](../../topoplan/core.py#L32) | `def point(self, cell):` | State the input/output contract and trace one normal and one failing case. |
| [34](../../topoplan/core.py#L34) | `raise ValueError('Start and goal must be integer [column, row] pairs')` | Construct the triggering input and explain why rejection preserves the contract. |
| [35](../../topoplan/core.py#L35) | `return tuple(cell)` | Predict the returned value and distinguish a computed result from authorization. |
| [37](../../topoplan/core.py#L37) | `def valid(self, cell):` | State the input/output contract and trace one normal and one failing case. |
| [39](../../topoplan/core.py#L39) | `if not (r<x<self.width-r and r<y<self.height-r): return False` | Predict the returned value and distinguish a computed result from authorization. |
| [40](../../topoplan/core.py#L40) | `return not any(a<=x<=c and b<=y<=d for a,b,c,d in self.boxes)` | Predict the returned value and distinguish a computed result from authorization. |
| [42](../../topoplan/core.py#L42) | `def clear_segment(self, a, b):` | State the input/output contract and trace one normal and one failing case. |
| [45](../../topoplan/core.py#L45) | `if not self.valid(a) or not self.valid(b): return False` | Predict the returned value and distinguish a computed result from authorization. |
| [55](../../topoplan/core.py#L55) | `if low<=high: return False` | Predict the returned value and distinguish a computed result from authorization. |
| [56](../../topoplan/core.py#L56) | `return True` | Predict the returned value and distinguish a computed result from authorization. |
| [58](../../topoplan/core.py#L58) | `def neighbors(self, cell):` | State the input/output contract and trace one normal and one failing case. |
| [63](../../topoplan/core.py#L63) | `def crossing(self, a, b):` | State the input/output contract and trace one normal and one failing case. |
| [73](../../topoplan/core.py#L73) | `return tuple(values)` | Predict the returned value and distinguish a computed result from authorization. |
| [75](../../topoplan/core.py#L75) | `def signature(self, path):` | State the input/output contract and trace one normal and one failing case. |
| [79](../../topoplan/core.py#L79) | `return tuple(result)` | Predict the returned value and distinguish a computed result from authorization. |
| [81](../../topoplan/core.py#L81) | `def plan(self, start, goal, routes=1, max_expansions=100000):` | State the input/output contract and trace one normal and one failing case. |
| [90](../../topoplan/core.py#L90) | `raise ValueError('Start or goal has insufficient clearance or lies outside the grid')` | Construct the triggering input and explain why rejection preserves the contract. |
| [91](../../topoplan/core.py#L91) | `if type(routes) is not int or not 1<=routes<=10: raise ValueError('routes must be from 1 to 10')` | Construct the triggering input and explain why rejection preserves the contract. |
## Assessment and verification

A learner must explain the model, trace the capstone, identify a counterexample and state the scope boundary. Assess these four competencies equally; a numerical score cannot compensate for an unsupported scientific or operational claim.

Relevant existing checks:

- [tests/test_experiments.py](../../tests/test_experiments.py)
- [tests/test_researchos.py](../../tests/test_researchos.py)
- [tests/test_topoplan.py](../../tests/test_topoplan.py)

Follow [README](../../README.md) for environment and installed-package instructions. Read scripts before execution; source-reading exercises do not authorize provider calls, model downloads, live orders, hardware actions or messages. Synthetic fixtures establish software behavior only. This documentation change does not report those exercises as already executed.

## Interoperability and adoption gate

Retain explicit domain contracts. Shared assurance records should refer to domain evidence and its assumptions rather than relabeling graph reachability, homology, statistical calibration and epistemic acceptance as interchangeable quantities. The canonical ResearchOs controller remains owned by ResearchOs; this curriculum does not create another controller fork.

Adopt additional mathematics only after a versioned experiment demonstrates a useful operation, the scorecard has evidence, the simpler baseline is inadequate and existing conformance checks remain satisfied. Bounded signatures do not enumerate all unrestricted homotopy classes; offline planning does not authorize hardware execution.
