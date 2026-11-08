"""Finite-horizon grid planning with swept disk collisions and explicit waits."""
import heapq
import math
from .core import Grid
from .geometry import number, point_segment_distance, independent_clear


def at(path, tick):
    return path[min(tick,len(path)-1)]


def separated(a,b,c,d,radius):
    """Minimum squared relative distance over simultaneous linear motion."""
    x,y=a[0]-c[0],a[1]-c[1]
    dx,dy=(b[0]-a[0])-(d[0]-c[0]),(b[1]-a[1])-(d[1]-c[1])
    denominator=dx*dx+dy*dy
    t=max(0,min(1,-(x*dx+y*dy)/denominator)) if denominator else 0
    return (x+t*dx)**2+(y+t*dy)**2 > radius*radius


def validate_task(grid,start,goal,horizon,max_expansions,obstacles):
    start,goal=grid.point(start),grid.point(goal)
    if start not in grid.free or goal not in grid.free: raise ValueError('timed endpoints have insufficient clearance')
    if type(horizon) is not int or not 0 <= horizon <= 128: raise ValueError('horizon must be in [0,128]')
    if type(max_expansions) is not int or not 1 <= max_expansions <= 200000: raise ValueError('expansion limit outside bounds')
    if (horizon+1)*grid.width*grid.height > 200000: raise ValueError('time-expanded graph exceeds 200000 states')
    if not isinstance(obstacles,list) or len(obstacles)>16: raise ValueError('at most 16 moving disks permitted')
    for obstacle in obstacles:
        if not isinstance(obstacle,dict) or set(obstacle)!={'path','radius'}: raise ValueError('moving disk requires path and radius')
        number(obstacle['radius'])
        path=obstacle['path']
        if not isinstance(path,list) or not 1 <= len(path) <= horizon+1: raise ValueError('bounded timed obstacle path required')
        obstacle_grid=Grid(list(grid.rows),obstacle['radius'])
        cells=[obstacle_grid.point(p) for p in path]
        if any(p not in obstacle_grid.free for p in cells) or any(not obstacle_grid.clear_segment(a,b) for a,b in zip(cells,cells[1:])):
            raise ValueError('moving obstacle intersects static geometry')
    if any(not separated(start,start,at(o['path'],0),at(o['path'],0),grid.radius+o['radius']) for o in obstacles):
        raise ValueError('start collides with a moving disk')
    return start,goal


def timed_plan(grid,start,goal,obstacles=None,horizon=32,max_expansions=100000):
    obstacles=[] if obstacles is None else obstacles
    start,goal=validate_task(grid,start,goal,horizon,max_expansions,obstacles)
    def allowed(a,b,tick):
        return all(separated(a,b,at(o['path'],tick),at(o['path'],tick+1),grid.radius+o['radius']) for o in obstacles)
    def can_hold(tick):
        return all(allowed(goal,goal,t) for t in range(tick,horizon))
    queue=[(abs(start[0]-goal[0])+abs(start[1]-goal[1]),0,start)]
    parents={(start,0):None};expanded=0
    while queue and expanded<max_expansions:
        _,tick,cell=heapq.heappop(queue);expanded+=1
        state=(cell,tick)
        if cell==goal and can_hold(tick):
            path=[];cursor=state
            while cursor is not None:
                path.append(list(cursor[0]));cursor=parents[cursor]
            path.reverse();arrival=tick
            path += [list(goal) for _ in range(horizon-tick)]
            return {'termination':'solution_found','path':path,'arrival_tick':arrival,'expanded_states':expanded,
                    'horizon':horizon,'goal_policy':'remain at goal through horizon','exhaustive':False}
        if tick==horizon:continue
        for other in [cell,*grid.neighbors(cell)]:
            nxt=(other,tick+1)
            if nxt not in parents and allowed(cell,other,tick):
                parents[nxt]=state
                heuristic=abs(other[0]-goal[0])+abs(other[1]-goal[1])
                heapq.heappush(queue,(tick+1+heuristic,tick+1,other))
    return {'termination':'expansion_limit' if queue else 'horizon_exhausted','path':None,'arrival_tick':None,
            'expanded_states':expanded,'horizon':horizon,'exhaustive':not queue,
            'scope':'Only the declared four-neighbor/wait, synchronized linear-motion graph through this horizon'}


def prioritized_plan(grid,robots,horizon=32,max_expansions=100000,obstacles=None):
    if not isinstance(robots,list) or not 1 <= len(robots) <= 4:raise ValueError('one to four robots required')
    ids=set()
    for robot in robots:
        if not isinstance(robot,dict) or set(robot)!={'id','start','goal'}: raise ValueError('robot requires id/start/goal')
        name=robot['id']
        if not isinstance(name,str) or not name.strip() or len(name)>100 or name in ids: raise ValueError('unique robot ID required')
        ids.add(name)
    # Reject overlapping initial disks independently of priority order.
    starts=[grid.point(r['start']) for r in robots]
    if any(math.dist(a,b)<=2*grid.radius for i,a in enumerate(starts) for b in starts[i+1:]):
        raise ValueError('robot initial disks overlap or touch')
    reservations=list(obstacles or []);results=[]
    for robot in robots:
        report=timed_plan(grid,robot['start'],robot['goal'],reservations,horizon,max_expansions)
        results.append({'id':robot['id'],**report})
        if report['path'] is None:
            return {'termination':'prioritized_failure','robots':results,'complete':False,
                    'scope':'Failure under this fixed priority and previous reservations; not joint infeasibility'}
        reservations.append({'path':report['path'],'radius':grid.radius})
    return {'termination':'solution_found','robots':results,'complete':True,
            'scope':'Fixed input priority; synchronized linear motion; goal occupied until horizon; not joint optimality'}


def independent_timed_clear(grid,a,b,tick,obstacles):
    # Cell indices differ from world centers by the same constant, which cancels in relative motion.
    world=lambda p:(p[0]+.5,p[1]+.5)
    if not independent_clear(grid,world(a),world(b)):return False
    for obstacle in obstacles:
        c,d=at(obstacle['path'],tick),at(obstacle['path'],tick+1)
        relative_a=(a[0]-c[0],a[1]-c[1]);relative_b=(b[0]-d[0],b[1]-d[1])
        if point_segment_distance((0,0),relative_a,relative_b)<=grid.radius+obstacle['radius']:return False
    return True


def verify_timed_path(grid,path,start,goal,horizon,obstacles):
    if not isinstance(path,list) or len(path)!=horizon+1 or path[0]!=list(start) or path[-1]!=list(goal):
        raise ValueError('timed route endpoint/horizon mismatch')
    for p in path:grid.point(p)
    for tick,(a,b) in enumerate(zip(path,path[1:])):
        if abs(a[0]-b[0])+abs(a[1]-b[1])>1 or not independent_timed_clear(grid,a,b,tick,obstacles):
            raise ValueError('independent timed route collision/move check failed')
    if horizon==0 and not independent_timed_clear(grid,path[0],path[0],0,obstacles):
        raise ValueError('invalid zero-horizon route')
    return True
