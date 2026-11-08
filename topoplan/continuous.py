"""Seeded bounded continuous RRT baseline; not kinodynamic planning."""
import math
import random
from .geometry import Space, point, number


def rrt(grid, start, goal, routes=1, iterations=2000, seed=0, step=.75):
    space=Space(grid); start=point(start); goal=point(goal)
    number(step,.01,10)
    if not space.valid(start) or not space.valid(goal):
        raise ValueError('continuous endpoints have insufficient clearance')
    if type(iterations) is not int or not 1 <= iterations <= 200000:
        raise ValueError('iterations outside bounds')
    if type(routes) is not int or not 1 <= routes <= 10:
        raise ValueError('routes outside bounds')
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError('seed outside bounds')
    rng=random.Random(seed); nodes=[start]; parents=[None]; found=[]; signatures=set(); used=0
    def accept(path):
        signature=space.signature(path)
        if tuple(signature) not in signatures:
            signatures.add(tuple(signature));found.append({'path':[list(p) for p in path],'signature':signature})
    if space.clear(start,goal):
        accept([start] if start==goal else [start,goal])
    while used < iterations and len(found) < routes:
        used+=1
        sample=goal if rng.random()<.15 else (rng.uniform(0,grid.width),rng.uniform(0,grid.height))
        index=min(range(len(nodes)),key=lambda i:math.dist(nodes[i],sample))
        parent=nodes[index]; length=math.dist(parent,sample)
        if length==0: continue
        p=tuple(a+(b-a)*min(1,step/length) for a,b in zip(parent,sample))
        if not space.clear(parent,p): continue
        nodes.append(p);parents.append(index);last=len(nodes)-1
        if math.dist(p,goal)<=step and space.clear(p,goal):
            path=[]
            while last is not None:
                path.append(nodes[last]);last=parents[last]
            path.reverse()
            if path[-1]!=goal:path.append(goal)
            accept(path)
    return {'routes':found,'termination':'requested_routes_found' if len(found)>=routes else 'iteration_limit',
            'iterations':used,'seed':seed,'requested_routes':routes,'exhaustive':False,
            'scope':'Continuous 2D translation, conservative expanded squares; no orientation or dynamics'}
