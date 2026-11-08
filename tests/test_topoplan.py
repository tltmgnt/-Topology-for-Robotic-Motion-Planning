from collections import deque
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from topoplan import Grid


class PlanningTests(unittest.TestCase):
    def test_open_grid_shortest_path(self):
        report=Grid(['.....']*4).plan([0,0],[4,3])
        self.assertEqual(report['routes'][0]['length'],7)
        self.assertEqual(report['routes'][0]['path'][0],[0,0])
        self.assertEqual(report['routes'][0]['path'][-1],[4,3])

    def test_astar_matches_bfs_all_small_maps(self):
        # Every obstacle pattern outside fixed endpoints of a 3x3 grid.
        candidates=[(x,y) for y in range(3) for x in range(3) if (x,y) not in [(0,0),(2,2)]]
        for mask in range(1<<len(candidates)):
            blocked={p for i,p in enumerate(candidates) if mask&(1<<i)}
            rows=[''.join('#' if (x,y) in blocked else '.' for x in range(3)) for y in range(3)]
            grid=Grid(rows); distances={(0,0):0}; queue=deque([(0,0)])
            while queue:
                x,y=queue.popleft()
                for p in [(x-1,y),(x+1,y),(x,y-1),(x,y+1)]:
                    if 0<=p[0]<3 and 0<=p[1]<3 and p not in blocked and p not in distances:
                        distances[p]=distances[(x,y)]+1; queue.append(p)
            report=grid.plan((0,0),(2,2))
            if (2,2) in distances: self.assertEqual(report['routes'][0]['length'],distances[(2,2)])
            else: self.assertEqual(report['routes'],[])

    def test_two_sides_of_obstacle(self):
        grid=Grid(['.......','...#...','...#...','...#...','.......'],0.2)
        result=grid.plan([0,2],[6,2],2)
        self.assertEqual(result['status'],'requested_routes_found')
        self.assertEqual(len(result['routes']),2)
        self.assertNotEqual(result['routes'][0]['signature'],result['routes'][1]['signature'])
        for route in result['routes']:
            self.assertEqual(route['length'],10)
            self.assertTrue(all(grid.clear_segment(a,b) for a,b in zip(route['path'],route['path'][1:])))

    def test_blocked_and_expansion_limit(self):
        grid=Grid(['..#..']*3)
        self.assertEqual(grid.plan([0,1],[4,1])['status'],'bounded_search_exhausted')
        self.assertEqual(Grid(['.....']*5).plan([0,0],[4,4],max_expansions=1)['status'],'expansion_limit')

    def test_radius_changes_feasibility(self):
        rows=['#######','.......','#######']
        self.assertTrue(Grid(rows,0.49).plan([0,1],[6,1])['routes'])
        with self.assertRaises(ValueError): Grid(rows,0.5).plan([1,1],[5,1])

    def test_segment_obstacle_and_boundary(self):
        grid=Grid(['.....','..#..','.....'])
        self.assertFalse(grid.clear_segment((0,1),(4,1)))
        self.assertTrue(grid.clear_segment((0,0),(4,0)))
        self.assertFalse(grid.clear_segment((-1,0),(0,0)))
        self.assertFalse(grid.clear_segment((0,0),(4,2)))

    def test_same_endpoint(self):
        route=Grid(['...']).plan([1,0],[1,0])['routes'][0]
        self.assertEqual(route['length'],0); self.assertEqual(route['path'],[[1,0]])

    def test_signature_reversal_and_concatenation(self):
        grid=Grid(['.....','..#..','.....'])
        path=[(0,0),(4,0),(4,2),(0,2),(0,0)]
        sig=grid.signature(path)
        self.assertEqual(abs(sig[0]),1)
        self.assertEqual(grid.signature(list(reversed(path))),tuple(-s for s in sig))
        self.assertEqual(grid.signature(path+path[1:]),tuple(2*s for s in sig))

    def test_validation(self):
        for rows in [[],['..','.'],['x'],['']]:
            with self.assertRaises(ValueError): Grid(rows)
        for radius in [-1,float('nan'),True]:
            with self.assertRaises(ValueError): Grid(['...'],radius)
        grid=Grid(['.#.'])
        for start in [[1,0],[-1,0],[0.2,0],[True,0]]:
            with self.assertRaises(ValueError): grid.plan(start,[2,0])
        with self.assertRaises(ValueError): grid.plan([0,0],[2,0],routes=0)


class CliTests(unittest.TestCase):
    def test_exports(self):
        with tempfile.TemporaryDirectory() as directory:
            report=Path(directory)/'report.json'; svg=Path(directory)/'route.svg'
            proc=subprocess.run([sys.executable,'-m','topoplan','examples/obstacle.json','--routes','2','--output',str(report),'--svg',str(svg)],capture_output=True,text=True,check=True)
            self.assertEqual(json.loads(proc.stdout),json.loads(report.read_text()))
            self.assertEqual(len(json.loads(proc.stdout)['routes']),2)
            ET.parse(svg)

    def test_no_route_cli_exit(self):
        proc=subprocess.run([sys.executable,'-m','topoplan','examples/disconnected.json'],capture_output=True,text=True)
        self.assertEqual(proc.returncode,2)
        self.assertEqual(json.loads(proc.stdout)['routes'],[])

    def test_invalid_cli(self):
        proc=subprocess.run([sys.executable,'-m','topoplan','examples/obstacle.json','--routes','0'],capture_output=True,text=True)
        self.assertEqual(proc.returncode,2); self.assertNotIn('Traceback',proc.stderr)


if __name__=='__main__': unittest.main()
