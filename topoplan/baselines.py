"""Optional native baselines are isolated; missing, crashing and timed-out are distinct."""
import importlib.util
import json
import subprocess
import sys
from .geometry import Space, number, point


def ompl_plan(grid,start,goal,planner='PRM',routes=1,seconds=.2,seed=0):
    if planner not in ('PRM','RRT','RRTstar'):raise ValueError('unsupported OMPL baseline')
    number(seconds,.001,10)
    if type(routes) is not int or not 1<=routes<=10:raise ValueError('routes outside bounds')
    if type(seed) is not int or not 0<=seed<2**32:raise ValueError('seed outside bounds')
    start,goal=point(start),point(goal);space=Space(grid)
    if not space.valid(start) or not space.valid(goal):raise ValueError('continuous endpoints have insufficient clearance')
    if importlib.util.find_spec('ompl') is None:
        return {'routes':[],'termination':'backend_unavailable','exhaustive':False,'backend':'ompl'}
    task={'grid':list(grid.rows),'radius':grid.radius,'start':list(start),'goal':list(goal),
          'planner':planner,'routes':routes,'seconds':seconds,'seed':seed}
    try:
        process=subprocess.run([sys.executable,'-m','topoplan.ompl_worker'],input=json.dumps(task),
                               capture_output=True,text=True,timeout=seconds+15)
    except subprocess.TimeoutExpired:
        return {'routes':[],'termination':'backend_timeout','exhaustive':False,'backend':'ompl'}
    if process.returncode:
        return {'routes':[],'termination':'backend_error','exhaustive':False,'backend':'ompl',
                'returncode':process.returncode}
    try:
        result=json.loads(process.stdout)
        if not isinstance(result,dict) or 'routes' not in result or 'termination' not in result:raise ValueError()
        return result
    except (ValueError,TypeError):
        return {'routes':[],'termination':'backend_error','exhaustive':False,'backend':'ompl','reason':'invalid worker receipt'}
