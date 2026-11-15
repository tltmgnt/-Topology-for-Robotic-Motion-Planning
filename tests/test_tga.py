import copy
import json
from pathlib import Path
import unittest
from topoplan.tga import assess, select_intervention, reality_convergence


class TGATests(unittest.TestCase):
    def setUp(self):
        self.scenario = json.loads((Path(__file__).parents[1] / 'examples/tga/navigation.json').read_text())

    def test_eligible_is_not_hardware_authority(self):
        result = assess(self.scenario)
        self.assertEqual(result['decision'], 'ELIGIBLE_OFFLINE')
        self.assertFalse(result['hardware_authorized'])
        self.assertTrue(result['plan']['routes'])

    def test_deficit_propagates(self):
        self.scenario['capabilities']['battery']['evidence'] = [[5, 10]]
        result = assess(self.scenario)
        self.assertEqual(result['gap_vector']['energy'], .75)
        self.assertIn('navigation: unmet prerequisite battery', result['blockers'])

    def test_missing_and_contradictory_evidence(self):
        for evidence, reason in [([], 'missing'), ([[25, 30], [10, 15]], 'contradictory')]:
            self.scenario['capabilities']['battery']['evidence'] = evidence
            result = assess(self.scenario)
            self.assertTrue(any(reason in b for b in result['blockers']))

    def test_hard_gates_cannot_be_averaged_away(self):
        self.scenario['authorized'] = False
        self.scenario['risk'] = .9
        result = assess(self.scenario)
        self.assertEqual(result['gap_score'], 0)
        self.assertEqual(result['decision'], 'BLOCKED')
        self.assertEqual(len(result['blockers']), 2)

    def test_no_route_blocks(self):
        self.scenario['navigation']['grid'] = ['...#...', '...#...', '...#...', '...#...', '...#...']
        self.assertEqual(assess(self.scenario)['decision'], 'BLOCKED')

    def test_bad_dependencies_and_numbers(self):
        for change in ('cycle', 'unknown', 'nan', 'bool'):
            s = copy.deepcopy(self.scenario)
            if change == 'cycle':
                s['capabilities']['battery']['depends_on'] = ['navigation']
            elif change == 'unknown':
                s['capabilities']['battery']['depends_on'] = ['absent']
            else:
                s['risk'] = float('nan') if change == 'nan' else True
            with self.assertRaises(ValueError):
                assess(s)

    def test_interventions_are_predictions_without_mutation(self):
        original = copy.deepcopy(self.scenario)
        blocked = copy.deepcopy(original)
        blocked['authorized'] = False
        candidates = [{'id': 'blocked', 'cost': 0, 'scenario': blocked},
                      {'id': 'expensive', 'cost': 5, 'scenario': original},
                      {'id': 'cheap', 'cost': 2, 'scenario': original}]
        self.assertEqual(select_intervention(candidates)['id'], 'cheap')
        self.assertEqual(self.scenario, original)
        self.assertIsNone(select_intervention(candidates[:1]))

    def test_reality_metric(self):
        self.assertAlmostEqual(reality_convergence({'load': 4}, {'load': 7}, {'load': 10}), .7)
        self.assertEqual(reality_convergence({'load': 4}, {'load': 70}, {'load': 10}), 0)
        with self.assertRaises(ValueError):
            reality_convergence({'load': 4}, {}, {'load': 10})
        with self.assertRaises(ValueError):
            reality_convergence({'load': 4}, {'load': 7}, {'load': 0})
