"""Ω-TGA v0.1: offline capability assurance over a typed dependency graph.

All measurements and authority flags are caller declarations, not authenticated
sensor evidence. An eligible result never authorizes hardware execution.
"""
import argparse
import json
import math
from pathlib import Path

from . import Grid

CATEGORIES = ('physical', 'perceptual', 'cognitive', 'computational', 'energy',
              'environmental', 'verification', 'governance')


def number(value, name, lower=0, upper=None):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(name + ' must be finite numeric data')
    if value < lower or (upper is not None and value > upper):
        raise ValueError(name + ' is outside its allowed range')
    return value


def assess(scenario):
    """Evaluate minimum-capability goals with noncompensable hard blockers.

    Each capability's evidence is a list of closed intervals in the same unit
    as its positive minimum requirement. Empty intersections are contradictions.
    Dependencies are directed prerequisite edges, not spatial homology.
    """
    if scenario.get('schema_version') != 1:
        raise ValueError('schema_version must be 1')
    caps = scenario['capabilities']
    if not isinstance(caps, dict) or not caps or len(caps) > 256:
        raise ValueError('capabilities must contain 1–256 entries')
    blockers, gaps, valid = [], {}, {}
    edges = {}
    for name, cap in caps.items():
        if not isinstance(name, str) or not name.strip():
            raise ValueError('capability identifiers must be nonempty strings')
        if cap['category'] not in CATEGORIES:
            raise ValueError('unknown capability category')
        required = number(cap['required'], name + '.required')
        if required == 0:
            raise ValueError('minimum requirements must be positive')
        if not isinstance(cap['unit'], str) or not cap['unit'].strip():
            raise ValueError('capability unit is required')
        dependencies = cap.get('depends_on', [])
        if not isinstance(dependencies, list) or any(d not in caps for d in dependencies):
            raise ValueError('dependencies must reference known capabilities')
        edges[name] = dependencies
        evidence = cap.get('evidence', [])
        if not isinstance(evidence, list):
            raise ValueError('evidence must be a list of intervals')
        intervals = []
        for interval in evidence:
            if not isinstance(interval, list) or len(interval) != 2:
                raise ValueError('evidence interval must have two endpoints')
            lo, hi = (number(x, name + '.evidence') for x in interval)
            if lo > hi:
                raise ValueError('reversed evidence interval')
            intervals.append((lo, hi))
        if not intervals:
            gap, ok = 1.0, False
            blockers.append(name + ': missing evidence')
        elif max(i[0] for i in intervals) > min(i[1] for i in intervals):
            gap, ok = 1.0, False
            blockers.append(name + ': contradictory evidence')
        else:
            # Intersection is a declared consistency envelope, not a statistical CI.
            available = max(i[0] for i in intervals)
            gap = max(0.0, (required - available) / required)
            ok = gap == 0
            if not ok:
                blockers.append(name + ': capability deficit')
        gaps[name], valid[name] = gap, ok
    # Cycles have no declared foundational prerequisite; reject rather than infer.
    visiting, visited = set(), set()
    def visit(name):
        if name in visiting:
            raise ValueError('cyclic capability dependencies')
        if name in visited:
            return
        visiting.add(name)
        for dep in edges[name]:
            visit(dep)
            if not valid[dep]:
                valid[name] = False
                blockers.append(name + ': unmet prerequisite ' + dep)
        visiting.remove(name)
        visited.add(name)
    for name in caps:
        visit(name)
    authorized = scenario['authorized']
    if type(authorized) is not bool:
        raise ValueError('authorized must be Boolean')
    if not authorized:
        blockers.append('governance: authorization absent')
    risk = number(scenario['risk'], 'risk', upper=1)
    limit = number(scenario['risk_limit'], 'risk_limit', upper=1)
    if risk > limit:
        blockers.append('risk: threshold exceeded')
    navigation = scenario['navigation']
    grid = Grid(navigation['grid'], navigation['radius'])
    plan = grid.plan(navigation['start'], navigation['goal'], 1)
    if not plan['routes']:
        blockers.append('navigation: ' + plan['status'])
    vector = {category: max((gaps[n] for n in caps if caps[n]['category'] == category), default=0.0)
              for category in CATEGORIES}
    return {'schema_version': 1, 'decision': 'BLOCKED' if blockers else 'ELIGIBLE_OFFLINE',
            'blockers': sorted(set(blockers)), 'capability_gaps': gaps,
            'gap_vector': vector, 'gap_score': sum(vector.values()) / len(vector),
            'plan': plan, 'hardware_authorized': False,
            'scope': 'Declared synthetic model only; no physical safety certification or TECT ASSURED claim'}


def select_intervention(candidates):
    """Select cheapest eligible *declared* post-intervention scenario.

    Does not execute interventions or prove their predicted effects realizable.
    Authority is supplied independently in each scenario; no authority is granted.
    """
    eligible = []
    for candidate in candidates:
        cost = number(candidate['cost'], 'intervention cost')
        result = assess(candidate['scenario'])
        if result['decision'] == 'ELIGIBLE_OFFLINE':
            eligible.append({'id': candidate['id'], 'cost': cost, 'assessment': result})
    return min(eligible, key=lambda c: (c['cost'], c['id'])) if eligible else None


def reality_convergence(predicted, observed, scales):
    """Bounded score over matched numeric observables and declared unit scales."""
    if not predicted or predicted.keys() != observed.keys() or predicted.keys() != scales.keys():
        raise ValueError('nonempty matching observable keys required')
    errors = []
    for key in predicted:
        scale = number(scales[key], 'scale')
        if scale == 0:
            raise ValueError('scale must be positive')
        p = number(predicted[key], 'prediction', lower=-math.inf)
        o = number(observed[key], 'observation', lower=-math.inf)
        errors.append(min(1.0, abs(p / scale - o / scale)))
    return 1 - sum(errors) / len(errors)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', type=Path)
    args = parser.parse_args(argv)
    try:
        result = assess(json.loads(args.scenario.read_text()))
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result['decision'] == 'ELIGIBLE_OFFLINE' else 1
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(2, 'tga: ' + str(exc) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
