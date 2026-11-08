"""Scoped synthetic domain adapter; no external execution."""
from topoplan import Grid

def check():
    rows=['.......','...#...','...#...','...#...','.......']
    grid=Grid(rows,0.2); report=grid.plan([0,2],[6,2],2)
    routes=report['routes']
    passed=len(routes)==2 and routes[0]['signature']!=routes[1]['signature'] and all(r['length']==10 and all(grid.clear_segment(a,b) for a,b in zip(r['path'],r['path'][1:])) for r in routes)
    return {'domain':'robotic-planning','statement':'The synthetic grid admits two length-10 clearance-checked routes with different signatures.',
            'passed':passed,'configuration':{'grid':rows,'radius':0.2,'start':[0,2],'goal':[6,2]},
            'observed':report,'scope':'Static grid software check; no hardware execution or safety certification'}
