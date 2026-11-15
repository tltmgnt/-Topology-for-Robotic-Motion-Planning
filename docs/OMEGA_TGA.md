# Ω-TGA — Topological Gap Analysis for Verified Physical AI

Version 0.1 implementation / research architecture, November 2026.
Scientific core: Topological Gap Analysis (TGA). Engine identity: Autonomous
Topological Gap Analysis Engine (ATGAE). Proposed humanoid application: OPTIMUS Ω.
This is an independent proposal, with no claimed Tesla architecture access,
integration, endorsement, hardware execution or safety certification.

## Research hypothesis

Explicitly representing missing capability, evidence, resources and authority
before action selection may improve safe task performance. This hypothesis has
not been established by the software tests in this repository.

The conceptual world model is W_t = (G_t, S_t, U_t). A mission defines an acceptable
set of worlds M, rather than necessarily one exact target world. A future general
model may define Δ(W_t, M) = inf_{w in M} d(W_t, w). This pilot does not implement
a universal world-distance metric or prove mission reachability from a score.

## Executable contract

`topoplan.tga.assess(scenario)` consumes schema version 1 and returns JSON-compatible
data. See `examples/tga/navigation.json` for a complete input.

- `capabilities`: 1–256 named minimum requirements. Each declares a category,
  nonempty unit, positive `required`, `evidence` intervals and optional `depends_on`.
- Categories: physical, perceptual, cognitive, computational, energy,
  environmental, verification and governance.
- Evidence is a list of nonnegative finite closed intervals in the requirement's
  unit. Empty evidence is missing; disjoint intervals are contradictory. For
  mutually consistent intervals the intersection's lower endpoint is the
  conservative available value. These are caller-declared envelopes, not
  calibrated confidence intervals or independently authenticated measurements.
- Each dependency is a prerequisite capability. References must exist and the
  graph must be acyclic. Unmet prerequisites propagate through the graph.
- `authorized` is a required Boolean declaration. `risk` and `risk_limit` are
  required finite values in [0,1]. Neither is inferred from capability.
- `navigation` supplies the existing Grid model's grid, radius, start and goal.
  A route must be returned; planning failure retains the planner's status.

For capability i with minimum r_i and conservative availability a_i:

    gap_i = max(0, (r_i - a_i) / r_i)

Missing/contradictory evidence receives diagnostic gap 1. Each category reports
the maximum member gap; the scalar score is the mean of the eight category gaps.
Empty categories have diagnostic score zero, meaning unmodeled, not verified.
Dependency, route, risk and authorization blockers remain separate hard constraints.
A zero scalar score does not imply eligibility or safety. This diagnostic is not
claimed to satisfy metric axioms or to measure spatial topology.

`ELIGIBLE_OFFLINE` means all declared checks passed; otherwise `BLOCKED` records
reasons. `hardware_authorized` is always false. CLI exit codes: 0 eligible offline,
1 blocked, 2 malformed input/I/O failure. Existing planner commands are unchanged.

## Topology and integration boundaries

| Layer | Meaning | Current implementation |
|---|---|---|
| TopoPlan | Spatial route topology | Homology-style signatures and bounded planning |
| TGA | Capability dependency topology | Directed acyclic prerequisite graph |
| ResearchOS/TECT | Claim/evidence assurance | Existing independent controller and evidence workflow |
| Physical control | Feasible robot state transitions | Not implemented by this pilot |

These structures retain distinct semantics. No theorem transfers automatically
from spatial signatures to capability graphs or epistemic claim states. TGA
output can be reviewed as evidence in the existing TECT workflow; this release
does not automatically submit it, promote a claim, or bypass domain attacks,
independent reproduction, evidence freshness or reviewer requirements.

The proposed full loop is observe → model → identify gap → plan → verify → act →
measure → learn. Implemented scope ends at offline assessment and analysis.
Contradiction blocks the proposed mission transition. On future balancing
hardware, a certified safe controller must continue maintaining stability;
blocking an action must not be interpreted as abruptly freezing actuators.

## Gap closure and reality feedback

`select_intervention(candidates)` accepts candidate IDs, nonnegative costs and
complete predicted post-intervention scenarios. It returns the cheapest eligible
candidate (ID breaks ties), or None. It never modifies scenarios or executes an
intervention. Candidate effects and authority declarations require independent
validation; this is bounded enumeration, not autonomous capability acquisition.

`reality_convergence(predicted, observed, scales)` requires identical nonempty
observable keys and positive scales in each observable's unit. It computes:

    RC = 1 - mean(min(1, |predicted / scale - observed / scale|))

RC lies in [0,1]. It measures normalized agreement for supplied observables,
not truth, calibration, safety or guaranteed convergence. Large errors saturate;
the caller should retain raw residuals for scientific analysis.

## Evaluation and roadmap

Current tests exercise eligible navigation, evidence contradictions and absence,
propagated deficits, hard gates, disconnected maps, malformed dependencies and
numbers, intervention choice and bounded prediction scoring. Synthetic tests
validate these software behaviors only.

Next research stages: versioned sensor evidence and timestamps; typed world and
mission ontologies; active evidence gathering; simulation of intervention effects;
matched experiments and ablations; controller integration; hardware-in-the-loop;
bounded physical trials; configuration-specific skill qualification; fleet transfer.
Humanoid dynamics, manipulation, learned skills and general-purpose autonomy
remain proposed work.

Compare identical task sets and planning budgets with and without TGA, then
ablate contradiction checks, prerequisites and intervention selection. Report
safe task success, intervention cost, latency, coverage, prediction residuals,
and uncertainty intervals. Report unsafe approvals/all approvals separately from
unsafe approvals/all unsafe candidates. Measure false rejections against all valid
candidates. No superiority or certification follows from passing unit tests.

Core invariant: agents may optimize within validated authority constraints;
capability does not grant authority to redefine those constraints.
