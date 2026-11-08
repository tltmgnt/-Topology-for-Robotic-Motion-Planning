"""Grid motion planning with conservative disk clearance and branch-cut signatures."""
from collections import deque
import heapq
import math


class Grid:
    """Rows run top to bottom; coordinates are (column, row) cell indices."""
    def __init__(self, rows, radius=0.0):
        if not isinstance(rows,list) or not rows or any(not isinstance(r,str) or not r for r in rows):
            raise ValueError('grid must contain nonempty string rows')
        self.width,self.height=len(rows[0]),len(rows)
        if self.width*self.height>10000 or any(len(r)!=self.width or set(r)-set('.#') for r in rows):
            raise ValueError('Grid must be rectangular, contain only . and #, and have at most 10000 cells')
        if isinstance(radius,bool) or not isinstance(radius,(int,float)) or not math.isfinite(radius) or radius<0:
            raise ValueError('radius must be a finite nonnegative number')
        self.rows=tuple(rows); self.radius=float(radius)
        self.blocked={(x,y) for y,row in enumerate(rows) for x,c in enumerate(row) if c=='#'}
        if len(self.blocked)>2000: raise ValueError('At most 2000 obstacle cells are supported')
        self.boxes=[(x-radius,y-radius,x+1+radius,y+1+radius) for x,y in sorted(self.blocked)]
        self.free={p for y in range(self.height) for x in range(self.width) if self.valid(p:=(x,y))}
        # One representative point inside each 4-connected occupied component.
        remaining=set(self.blocked); self.markers=[]
        while remaining:
            root=min(remaining); remaining.remove(root); queue=deque([root])
            while queue:
                x,y=queue.popleft()
                for p in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                    if p in remaining: remaining.remove(p); queue.append(p)
            self.markers.append((root[0]+0.5,root[1]+0.623))

    def point(self, cell):
        if not isinstance(cell,(list,tuple)) or len(cell)!=2 or any(type(v) is not int for v in cell):
            raise ValueError('Start and goal must be integer [column, row] pairs')
        return tuple(cell)

    def valid(self, cell):
        x,y=cell[0]+0.5,cell[1]+0.5; r=self.radius
        if not (r<x<self.width-r and r<y<self.height-r): return False
        return not any(a<=x<=c and b<=y<=d for a,b,c,d in self.boxes)

    def clear_segment(self, a, b):
        """Closed segment vs expanded closed obstacle boxes, using slab intervals."""
        p=(a[0]+0.5,a[1]+0.5); q=(b[0]+0.5,b[1]+0.5)
        if not self.valid(a) or not self.valid(b): return False
        for xmin,ymin,xmax,ymax in self.boxes:
            low,high=0.0,1.0
            for origin,end,lower,upper in zip(p,q,(xmin,ymin),(xmax,ymax)):
                delta=end-origin
                if delta==0:
                    if not lower<=origin<=upper: low,high=1.0,0.0; break
                else:
                    t0,t1=sorted(((lower-origin)/delta,(upper-origin)/delta))
                    low,high=max(low,t0),min(high,t1)
            if low<=high: return False
        return True

    def neighbors(self, cell):
        x,y=cell
        for p in ((x-1,y),(x,y-1),(x,y+1),(x+1,y)):
            if p in self.free and self.clear_segment(cell,p): yield p

    def crossing(self, a, b):
        """Signed crossings of horizontal rays to the right of marker points."""
        x0,y0=a[0]+0.5,a[1]+0.5; x1,y1=b[0]+0.5,b[1]+0.5
        values=[]
        for mx,my in self.markers:
            sign=1 if y0<=my<y1 else (-1 if y1<=my<y0 else 0)
            if sign:
                intersection=x0+(my-y0)*(x1-x0)/(y1-y0)
                if intersection<=mx: sign=0
            values.append(sign)
        return tuple(values)

    def signature(self, path):
        result=[0]*len(self.markers)
        for a,b in zip(path,path[1:]):
            for i,v in enumerate(self.crossing(a,b)): result[i]+=v
        return tuple(result)

    def plan(self, start, goal, routes=1, max_expansions=100000):
        """A* on (cell, signature), with each signature coordinate in [-1,1].

        Returns shortest routes per reached signature in this bounded state graph.
        A route can contain loops. Signatures distinguish some homology classes,
        not all homotopy classes. Coordinates use image-style positive-down y.
        """
        start,goal=self.point(start),self.point(goal)
        if start not in self.free or goal not in self.free:
            raise ValueError('Start or goal has insufficient clearance or lies outside the grid')
        if type(routes) is not int or not 1<=routes<=10: raise ValueError('routes must be from 1 to 10')
        if type(max_expansions) is not int or not 1<=max_expansions<=200000:
            raise ValueError('max_expansions must be from 1 to 200000')
        if routes>1 and len(self.markers)>6: raise ValueError('Alternative-route search supports at most 6 obstacle components')
        # Ordinary A* does not need an augmented state for a single route.
        zero=(0,)*len(self.markers) if routes>1 else ()
        initial=(start,zero); distance={initial:0}; parent={}; queue=[]
        heuristic=lambda p:abs(p[0]-goal[0])+abs(p[1]-goal[1])
        heapq.heappush(queue,(heuristic(start),0,initial)); results=[]; expanded=0
        while queue and expanded<max_expansions and len(results)<routes:
            _,cost,state=heapq.heappop(queue)
            if cost!=distance[state]: continue
            expanded+=1; cell,sig=state
            if cell==goal:
                path=[]; cursor=state
                while True:
                    path.append(cursor[0])
                    if cursor==initial: break
                    cursor=parent[cursor]
                path.reverse()
                results.append({'path':[list(p) for p in path],'length':cost,'signature':list(self.signature(path))})
            for neighbor in self.neighbors(cell):
                new_sig=tuple(a+b for a,b in zip(sig,self.crossing(cell,neighbor))) if routes>1 else ()
                if any(abs(v)>1 for v in new_sig): continue
                nxt=(neighbor,new_sig)
                if cost+1<distance.get(nxt,math.inf):
                    distance[nxt]=cost+1; parent[nxt]=state
                    heapq.heappush(queue,(cost+1+heuristic(neighbor),cost+1,nxt))
        if len(results)>=routes: status='requested_routes_found'
        elif queue: status='expansion_limit'
        else: status='bounded_search_exhausted'
        return {'routes':results,'status':status,'expanded_states':expanded,
                'requested_routes':routes,'radius':self.radius,'markers':[list(m) for m in self.markers],
                'signature_bound':1 if routes>1 else None,
                'model':'Four-neighbor grid; expanded square obstacles conservatively bound disk collisions'}

    def svg(self, report):
        colors=['#2563eb','#dc2626','#16a34a','#9333ea','#ea580c']
        parts=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.width} {self.height}"><title>Grid routes: x right, y down</title>',f'<rect width="{self.width}" height="{self.height}" fill="white"/>']
        if self.radius:
            for a,b,c,d in self.boxes:
                parts.append(f'<rect x="{a}" y="{b}" width="{c-a}" height="{d-b}" fill="#fecaca"/>')
        for x,y in sorted(self.blocked): parts.append(f'<rect x="{x}" y="{y}" width="1" height="1" fill="#334155"/>')
        for i,route in enumerate(report['routes']):
            points=' '.join(f'{x+0.5},{y+0.5}' for x,y in route['path'])
            parts.append(f'<polyline points="{points}" fill="none" stroke="{colors[i%len(colors)]}" stroke-width="0.12" opacity="0.8"/>')
        if report['routes']:
            path=report['routes'][0]['path']
            for p,color in [(path[0],'#16a34a'),(path[-1],'#dc2626')]:
                parts.append(f'<circle cx="{p[0]+0.5}" cy="{p[1]+0.5}" r="0.22" fill="{color}"/>')
        return ''.join(parts)+'</svg>\n'
