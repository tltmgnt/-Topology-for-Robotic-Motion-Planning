"""Research experiment regressions: independent proofs, receipts and model boundaries."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from topoplan import Grid
from topoplan.geometry import Space, independent_clear, route_metrics, center_clearance, clearance_sensitivity
from topoplan.continuous import rrt
from topoplan.temporal import timed_plan, prioritized_plan, separated, verify_timed_path
from topoplan.audit import reference_grid, reference_timed
from topoplan.experiments import normalize,execute,summarize,check_experiment
from topoplan.evidence import canonical,decode,contents,create_bundle,verify_bundle
from topoplan.benchmark import standard_study
from topoplan.baselines import ompl_plan


def task(**changes):
    result={'id':'test','kind':'static','grid':['.......','...#...','...#...','...#...','.......'],
            'start':[0,2],'goal':[6,2],'radius':.2,'planner':'astar','routes':2}
    result.update(changes)
    return result


def experiment(**changes):
    return normalize({'schema_version':1,'experiments':[task(**changes)]})['experiments'][0]


class ProofTests(unittest.TestCase):
    def test_opposite_side_diversity_and_independent_costs(self):
        exp=experiment();receipt=execute(exp);report=check_experiment(exp,receipt)
        self.assertEqual(report['distinct_signatures'],2)
        self.assertEqual(report['distinct_waypoint_sequences'],2)
        self.assertEqual([r['length'] for r in report['routes']],[10,10])
        self.assertTrue(report['verification']['optimality_verified'])
        self.assertTrue(all(r['minimum_surface_clearance']>0 for r in report['routes']))

    def test_limit_is_not_exhaustion(self):
        exp=experiment(max_expansions=1);report=check_experiment(exp,execute(exp))
        self.assertEqual(report['termination'],'expansion_limit')
        self.assertFalse(report['verification']['exhaustion_verified'])
        self.assertGreater(report['verification']['reachable_goal_signatures'],0)

    def test_exhaustion_proves_only_finite_graph(self):
        exp=experiment(grid=['..#..']*3,start=[0,1],goal=[4,1],routes=1,radius=0)
        report=check_experiment(exp,execute(exp))
        self.assertEqual(report['termination'],'bounded_search_exhausted')
        self.assertTrue(report['verification']['exhaustion_verified'])
        self.assertEqual(report['verification']['reachable_goal_signatures'],0)

    def test_false_exhaustion_and_false_path_are_rejected(self):
        exp=experiment(max_expansions=1);receipt=execute(exp)
        receipt['result']['termination']='bounded_search_exhausted'
        with self.assertRaises(ValueError):check_experiment(exp,receipt)
        exp=experiment();receipt=execute(exp)
        receipt['result']['routes'][0]['path']=[[0,2],[6,2]]
        with self.assertRaises(ValueError):check_experiment(exp,receipt)

    def test_independent_collision_and_metric_checks(self):
        grid=Grid(['.....','..#..','.....'],.2)
        with self.assertRaises(ValueError):route_metrics(grid,[[.5,1.5],[4.5,1.5]],[.5,1.5],[4.5,1.5])
        self.assertFalse(independent_clear(grid,(.5,.5),(2.5,1.5)))
        metric=route_metrics(grid,[[.5,.5],[4.5,.5]],[.5,.5],[4.5,.5])
        self.assertEqual(metric['length'],4)
        self.assertAlmostEqual(metric['minimum_surface_clearance'],.3)

    def test_corner_conservatism_witness(self):
        # Use an interior diagonal neighbor, away from workspace boundary.
        rows=['.......']*2+['...#...']+['.......']*3
        grid=Grid(rows,.55);p=(2.5,1.5)
        self.assertFalse(Space(grid).valid(p))
        self.assertGreater(center_clearance(grid,p,p),.55)
        witnesses=clearance_sensitivity(rows,[.55])[0]['disk_feasible_but_box_rejected_centers']
        self.assertIn([2,1],witnesses)

    def test_segment_checker_crosschecks_production(self):
        grid=Grid(['.....','..#..','.....'],.2)
        points=[(.5,.5),(4.5,.5),(.5,2.5),(4.5,2.5),(2.5,1.5),(2,.8)]
        for a in points:
            for b in points:
                self.assertEqual(independent_clear(grid,a,b),Space(grid).clear(a,b))

    def test_rrt_continuous_endpoints_and_repeatability(self):
        grid=Grid(['.....']*4,.1)
        a,b=[.7,.8],[4.2,2.7]
        result=rrt(grid,a,b,seed=17)
        self.assertEqual(result,rrt(grid,a,b,seed=17))
        metric=route_metrics(grid,result['routes'][0]['path'],a,b)
        self.assertAlmostEqual(metric['length'],math.dist(a,b))
        self.assertFalse(result['exhaustive'])

    def test_rrt_failure_is_budget_not_infeasibility(self):
        result=rrt(Grid(['..#..']*3),[.5,1.5],[4.5,1.5],iterations=10)
        self.assertEqual(result['termination'],'iteration_limit')
        self.assertEqual(result['routes'],[])
        self.assertFalse(result['exhaustive'])

    def test_bounded_signature_exhaustion_reference(self):
        grid=Grid(['.....','..#..','.....'])
        targets,count=reference_grid(grid,[0,1],[4,1],True)
        report=grid.plan([0,1],[4,1],10)
        self.assertEqual(report['status'],'bounded_search_exhausted')
        self.assertEqual(len(targets),len(report['routes']))
        self.assertGreater(count,0)


class TemporalTests(unittest.TestCase):
    def test_swap_and_midsegment_collision(self):
        self.assertFalse(separated((0,0),(1,0),(1,0),(0,0),.1))
        self.assertFalse(separated((0,1),(2,1),(1,0),(1,2),0))
        self.assertTrue(separated((0,0),(1,0),(0,1),(1,1),.5))

    def test_wait_to_avoid_crossing(self):
        grid=Grid(['.....']*3,.2)
        obstacles=[{'radius':.2,'path':[[2,0],[2,1],[2,2]]}]
        result=timed_plan(grid,[0,1],[4,1],obstacles,horizon=8)
        self.assertEqual(result['termination'],'solution_found')
        self.assertTrue(verify_timed_path(grid,result['path'],[0,1],[4,1],8,obstacles))
        earliest,_=reference_timed(grid,[0,1],[4,1],8,obstacles)
        self.assertEqual(result['arrival_tick'],earliest)

    def test_wait_is_an_explicit_move(self):
        # A separate two-dimensional case where direct movement needs waiting/detour.
        grid=Grid(['.....']*3,.2)
        obstacles=[{'radius':.2,'path':[[2,0],[2,0],[2,1],[2,2]]}]
        result=timed_plan(grid,[0,1],[4,1],obstacles,horizon=8)
        self.assertGreaterEqual(result['arrival_tick'],5)
        self.assertTrue(verify_timed_path(grid,result['path'],[0,1],[4,1],8,obstacles))

    def test_horizon_exhaustion_is_not_unbounded_failure(self):
        grid=Grid(['.....']*3,.2)
        result=timed_plan(grid,[0,1],[4,1],horizon=2)
        self.assertEqual(result['termination'],'horizon_exhausted')
        self.assertEqual(reference_timed(grid,[0,1],[4,1],2,[])[0],None)
        self.assertEqual(timed_plan(grid,[0,1],[4,1],horizon=8)['termination'],'solution_found')

    def test_goal_must_remain_clear_through_horizon(self):
        grid=Grid(['.....']*3,.2)
        obstacle=[{'radius':.2,'path':[[4,0],[4,0],[4,0],[4,1]]}]
        result=timed_plan(grid,[3,1],[4,1],obstacle,horizon=4)
        self.assertEqual(result['termination'],'horizon_exhausted')

    def test_joint_swap_avoids_robot_collisions(self):
        grid=Grid(['.....']*3,.2)
        robots=[{'id':'a','start':[0,1],'goal':[4,1]},{'id':'b','start':[4,1],'goal':[0,1]}]
        result=prioritized_plan(grid,robots,horizon=10)
        self.assertTrue(result['complete'])
        self.assertTrue(verify_timed_path(grid,result['robots'][1]['path'],[4,1],[0,1],10,
                        [{'radius':.2,'path':result['robots'][0]['path']}]))

    def test_priority_failure_does_not_claim_joint_infeasibility(self):
        exp=normalize({'schema_version':1,'experiments':[{'id':'swap','kind':'multi','grid':['.....'],'radius':.2,'horizon':8,
              'robots':[{'id':'a','start':[0,0],'goal':[4,0]},{'id':'b','start':[4,0],'goal':[0,0]}]}]})['experiments'][0]
        receipt=execute(exp);report=check_experiment(exp,receipt)
        self.assertEqual(report['termination'],'prioritized_failure')
        self.assertFalse(report['joint_infeasibility_claimed'])

    def test_invalid_overlap_and_temporal_limit(self):
        grid=Grid(['.....']*3,.2)
        with self.assertRaises(ValueError):prioritized_plan(grid,[{'id':'a','start':[0,1],'goal':[4,1]},{'id':'b','start':[0,1],'goal':[4,1]}])
        result=timed_plan(grid,[0,1],[4,1],horizon=8,max_expansions=1)
        self.assertEqual(result['termination'],'expansion_limit')
        self.assertFalse(result['exhaustive'])


class EvidenceTests(unittest.TestCase):
    def test_bundle_replay_and_manifest_tampering(self):
        study={'schema_version':1,'experiments':[task()]}
        with tempfile.TemporaryDirectory() as directory:
            dest=Path(directory)/'bundle';original=create_bundle(study,dest)
            self.assertEqual(verify_bundle(dest),original)
            with self.assertRaises(FileExistsError):create_bundle(study,dest)
            (dest/'report.json').write_text('{}')
            with self.assertRaises(ValueError):verify_bundle(dest)

    def test_symlinks_are_rejected_and_html_labels_escaped(self):
        study={'schema_version':1,'experiments':[task(id='<script>alert(1)</script>')]}
        with tempfile.TemporaryDirectory() as directory:
            dest=Path(directory)/'bundle';create_bundle(study,dest)
            html=(dest/'review.html').read_text()
            self.assertNotIn('<script>',html)
            self.assertIn('&lt;script&gt;',html)
            original=(dest/'receipts.json').read_bytes()
            (dest/'receipts.json').unlink()
            target=Path(directory)/'external.json';target.write_bytes(original)
            (dest/'receipts.json').symlink_to(target)
            with self.assertRaises((ValueError,OSError)):verify_bundle(dest)

    def test_captured_ompl_receipt_verifies_without_native_installation(self):
        exp=experiment(planner='ompl:RRT',grid=['.....']*3,start=[0,1],goal=[4,1],routes=1)
        receipt={'id':'test','wall_seconds':.1,'runtime':{'python':'3.12','implementation':'CPython'},'result':{'routes':[{'path':[[.5,1.5],[4.5,1.5]],'signature':[]}],
                 'termination':'requested_routes_found','exhaustive':False}}
        study={'schema_version':1,'experiments':[exp]}
        with patch('topoplan.baselines.ompl_plan',side_effect=AssertionError('must not run native code during verification')):
            _,report=contents(study,[receipt])
        self.assertEqual(report['experiments'][0]['verification']['routes_checked'],1)
        receipt['result']['exhaustive']=True
        with self.assertRaises(ValueError):contents(study,[receipt])

    def test_duplicate_json_keys_nonfinite_and_unknown_fields(self):
        for raw in [b'{"a":1,"a":2}',b'{"a":NaN}',b'{"a":1e999}']:
            with self.assertRaises(ValueError):decode(raw)
        bad=task();bad['unknown']=1
        with self.assertRaises(ValueError):normalize({'schema_version':1,'experiments':[bad]})
        with self.assertRaises(ValueError):normalize({'schema_version':True,'experiments':[task()]})

    def test_signature_bound_and_work_budget(self):
        bad=task(grid=['.#.#.#.#.#.#.#.']*3,start=[0,0],goal=[14,0])
        with self.assertRaises(ValueError):normalize({'schema_version':1,'experiments':[bad]})
        with self.assertRaises(ValueError):normalize({'schema_version':1,'experiments':[task(planner='rrt',iterations=10000)]})

    def test_invalid_endpoint_outcome_is_explicit(self):
        exp=experiment(start=[3,2]);receipt=execute(exp)
        self.assertEqual(receipt['result']['termination'],'invalid_input')
        self.assertTrue(check_experiment(exp,receipt)['verification']['input_invalidity_verified'])

    def test_standard_core_study_runs_offline(self):
        study=normalize(standard_study());receipts=[execute(e) for e in study['experiments']]
        report=summarize(study,receipts)
        self.assertEqual(len(report['experiments']),20)
        self.assertFalse(report['execution_authorized'])

    def test_missing_and_crashing_native_backend_are_not_no_path(self):
        with patch('topoplan.baselines.importlib.util.find_spec',return_value=None):
            result=ompl_plan(Grid(['...']),[.5,.5],[2.5,.5])
        self.assertEqual(result['termination'],'backend_unavailable')
        fake=subprocess.CompletedProcess([],9,'','crashed')
        with patch('topoplan.baselines.importlib.util.find_spec',return_value=True),patch('topoplan.baselines.subprocess.run',return_value=fake):
            result=ompl_plan(Grid(['...']),[.5,.5],[2.5,.5])
        self.assertEqual(result['termination'],'backend_error')


@unittest.skipUnless(importlib.util.find_spec('ompl'),'optional native OMPL backend absent')
class NativeTests(unittest.TestCase):
    def test_real_prm_rrt_and_rrtstar_routes(self):
        for planner in ('PRM','RRT','RRTstar'):
            with self.subTest(planner=planner):
                grid=Grid(['.....']*3,.1)
                result=ompl_plan(grid,[.5,1.5],[4.5,1.5],planner=planner,seconds=.3)
                self.assertEqual(result['termination'],'requested_routes_found',result)
                self.assertTrue(result['backend_version'].startswith('2.'))
                self.assertTrue(route_metrics(grid,result['routes'][0]['path'],[.5,1.5],[4.5,1.5])['collision_valid'])
