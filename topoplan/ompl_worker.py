"""Isolated optional OMPL 2.x backend. JSON input/output; no robot execution."""
import json
import sys
import time
import importlib.metadata
from .core import Grid
from .geometry import Space


def solve(task):
    from ompl import base as ob, geometric as og, util as ou
    grid=Grid(task['grid'],task['radius']);geometry=Space(grid)
    ou.setLogLevel(ou.LogLevel.LOG_NONE)
    ou.RNG.setSeed(task['seed'])
    planner_class={'PRM':og.PRM,'RRT':og.RRT,'RRTstar':og.RRTstar}[task['planner']]
    found=[];signatures=set();statuses=[];deadline=time.monotonic()+task['seconds'];attempt=0
    while attempt < task['routes']*4 and time.monotonic()<deadline and len(found)<task['routes']:
        attempt+=1
        space=ob.RealVectorStateSpace(2);bounds=ob.RealVectorBounds(2)
        bounds.setLow(0);bounds.setHigh(0,grid.width);bounds.setHigh(1,grid.height);space.setBounds(bounds)
        setup=og.SimpleSetup(space);si=setup.getSpaceInformation()
        setup.setStateValidityChecker(lambda state:geometry.valid((state[0],state[1])))
        class ExactMotion(ob.MotionValidator):
            def __init__(self,info):super().__init__(info)
            def checkMotion(self,a,b):return geometry.clear((a[0],a[1]),(b[0],b[1]))
        validator=ExactMotion(si);si.setMotionValidator(validator)
        start=space.allocState();goal=space.allocState()
        for i in (0,1):start[i]=task['start'][i];goal[i]=task['goal'][i]
        setup.setStartAndGoalStates(start,goal,1e-9)
        planner=planner_class(si);setup.setPlanner(planner);setup.setup()
        allowance=max(.000001,(deadline-time.monotonic())/(task['routes']*4-attempt+1))
        status=setup.solve(allowance);statuses.append(str(status))
        if setup.haveExactSolutionPath():
            path=[[state[0],state[1]] for state in setup.getSolutionPath().getStates()]
            # Validate/append the literal requested goal, rather than treating tolerance as identity.
            if not geometry.clear(path[-1],task['goal']):continue
            if path[-1]!=task['goal']:path.append(task['goal'])
            signature=geometry.signature(path)
            if tuple(signature) not in signatures:
                signatures.add(tuple(signature));found.append({'path':path,'signature':signature})
    return {'routes':found,'termination':'requested_routes_found' if len(found)>=task['routes'] else 'time_or_attempt_limit',
            'requested_routes':task['routes'],'exhaustive':False,'seed':task['seed'],
            'backend_version':importlib.metadata.version('ompl'),'attempts':attempt,'native_statuses':statuses,
            'budget_seconds':task['seconds'],'scope':'OMPL continuous 2D translation with exact conservative segment callback; approximate solutions excluded'}


def main():
    try:
        result=solve(json.load(sys.stdin))
    except Exception as error:
        print(json.dumps({'backend_error':type(error).__name__+': '+str(error)}))
        return 2
    print(json.dumps(result,allow_nan=False))
    return 0


if __name__=='__main__':raise SystemExit(main())
