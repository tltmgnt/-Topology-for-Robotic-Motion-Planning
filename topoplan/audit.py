"""Independent finite-graph proofs and route metrics; no production planner calls."""
from collections import deque
import math
from .geometry import independent_clear, route_metrics
from .temporal import independent_timed_clear


def reference_grid(grid,start,goal,augmented):
    markers=grid.markers if augmented else []
    if grid.width*grid.height*3**len(markers)>200000:
        raise ValueError('independent graph proof exceeds state budget')
    def crossing(a,b):
        values=[]
        for x,y in markers:
            # Centers are converted here independently of Grid.crossing.
            ax,ay,bx,by=a[0]+.5,a[1]+.5,b[0]+.5,b[1]+.5
            sign=int(ay<=y<by)-int(by<=y<ay)
            values.append(sign if sign and ax+(y-ay)*(bx-ax)/(by-ay)>x else 0)
        return tuple(values)
    initial=(tuple(start),(0,)*len(markers));distances={initial:0};queue=deque([initial]);targets={}
    while queue:
        cell,signature=queue.popleft();cost=distances[(cell,signature)]
        if cell==tuple(goal):targets[signature]=cost
        x,y=cell
        for other in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            a,b=(x+.5,y+.5),(other[0]+.5,other[1]+.5)
            if not independent_clear(grid,a,b):continue
            sig=tuple(a+b for a,b in zip(signature,crossing(cell,other)))
            state=(other,sig)
            if any(abs(v)>1 for v in sig) or state in distances:continue
            distances[state]=cost+1;queue.append(state)
    return targets,len(distances)


def reference_timed(grid,start,goal,horizon,obstacles):
    reached={tuple(start)};count=0
    for tick in range(horizon+1):
        count+=len(reached)
        if tuple(goal) in reached and all(independent_timed_clear(grid,goal,goal,t,obstacles) for t in range(tick,horizon)):
            return tick,count
        following=set()
        if tick<horizon:
            for x,y in reached:
                for other in ((x,y),(x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                    if independent_timed_clear(grid,(x,y),other,tick,obstacles):following.add(other)
        reached=following
    return None,count


def analyze_static(grid,experiment,result):
    mode=experiment['coordinate_mode'];start=experiment['start'];goal=experiment['goal']
    if mode=='cells':start=[p+.5 for p in start];goal=[p+.5 for p in goal]
    discrete=experiment['planner']=='astar'
    metrics=[]
    for route in result['routes']:
        if (not isinstance(route,dict) or not isinstance(route.get('signature'),list)
                or any(type(v) is not int for v in route['signature'])):
            raise ValueError('integer signature array required')
        path=route['path']
        if discrete:path=[[v+.5 for v in p] for p in path]
        item=route_metrics(grid,path,start,goal,discrete)
        if item['signature']!=route['signature']:raise ValueError('reported signature mismatch')
        if discrete and item['length']!=route['length']:raise ValueError('reported length mismatch')
        if discrete and experiment['routes']>1:
            accum=[0]*len(grid.markers)
            for a,b in zip(path,path[1:]):
                for i,(mx,my) in enumerate(grid.markers):
                    sign=int(a[1]<=my<b[1])-int(b[1]<=my<a[1])
                    if sign and a[0]+(my-a[1])*(b[0]-a[0])/(b[1]-a[1])>mx:accum[i]+=sign
                if any(abs(v)>1 for v in accum):raise ValueError('route violates intermediate signature bound')
        metrics.append({k:v for k,v in item.items() if k!='geometric_path_key'})
    signatures={tuple(m['signature']) for m in metrics}
    if len(signatures)!=len(metrics):raise ValueError('duplicate returned signature')
    geometric={tuple(tuple(p) for p in r['path']) for r in result['routes']}
    proof={'route_checks':'PASS' if metrics else 'NOT_APPLICABLE','routes_checked':len(metrics),
           'exhaustion_verified':False,'optimality_verified':False}
    if discrete:
        targets,states=reference_grid(grid,experiment['start'],experiment['goal'],experiment['routes']>1)
        for metric in metrics:
            sig=tuple(metric['signature']) if experiment['routes']>1 else ()
            if targets.get(sig)!=metric['length']:raise ValueError('route is not shortest in declared finite graph')
        expected=min(experiment['routes'],len(targets))
        if result['termination']=='bounded_search_exhausted':
            if len(metrics)!=expected:raise ValueError('false finite-graph exhaustion claim')
            proof['exhaustion_verified']=True
        proof.update(optimality_verified=bool(metrics),independently_reachable_states=states,reachable_goal_signatures=len(targets))
    return {'routes':metrics,'distinct_signatures':len(signatures),'distinct_waypoint_sequences':len(geometric),
            'signature_model':'Homology-style ray-crossing vector; not a complete homotopy classifier',
            'verification':proof}
