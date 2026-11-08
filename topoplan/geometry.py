"""Shared continuous workspace plus independent route and clearance measurements."""
import math
from .core import Grid


def number(value, low=0, high=1e6):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError('bounded finite number required')
    return float(value)


def point(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('2D point required')
    return tuple(number(v, -1e6) for v in value)


class Space:
    """Continuous (x,y) cell-width units, using the legacy conservative geometry."""
    def __init__(self, grid):
        self.grid = grid

    def valid(self, p):
        return self.grid.valid((p[0] - .5, p[1] - .5))

    def clear(self, a, b):
        return self.grid.clear_segment((a[0] - .5, a[1] - .5), (b[0] - .5, b[1] - .5))

    def signature(self, path):
        return list(self.grid.signature([(p[0] - .5, p[1] - .5) for p in path]))


def orientation(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])


def on_segment(a, b, p):
    return orientation(a,b,p) == 0 and min(a[0],b[0]) <= p[0] <= max(a[0],b[0]) and min(a[1],b[1]) <= p[1] <= max(a[1],b[1])


def intersects(a, b, c, d):
    ab_c, ab_d = orientation(a,b,c), orientation(a,b,d)
    cd_a, cd_b = orientation(c,d,a), orientation(c,d,b)
    return ((ab_c > 0) != (ab_d > 0) and (cd_a > 0) != (cd_b > 0)
            or on_segment(a,b,c) or on_segment(a,b,d) or on_segment(c,d,a) or on_segment(c,d,b))


def box_collision(a, b, box):
    """Independent orientation/edge checks, not production slab intervals."""
    x0,y0,x1,y1 = box
    inside = lambda p: x0 <= p[0] <= x1 and y0 <= p[1] <= y1
    corners = [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
    return inside(a) or inside(b) or any(intersects(a,b,c,d) for c,d in zip(corners,corners[1:]+corners[:1]))


def independent_clear(grid, a, b):
    r = grid.radius
    if any(not (r < p[0] < grid.width-r and r < p[1] < grid.height-r) for p in (a,b)):
        return False
    return not any(box_collision(a,b,(x-r,y-r,x+1+r,y+1+r)) for x,y in grid.blocked)


def point_segment_distance(p, a, b):
    dx,dy = b[0]-a[0], b[1]-a[1]
    squared = dx*dx+dy*dy
    t = max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/squared)) if squared else 0
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def center_clearance(grid, a, b):
    clearance = min(p[0] for p in (a,b))
    clearance = min(clearance, *(min(p[1],grid.width-p[0],grid.height-p[1]) for p in (a,b)))
    for x,y in grid.blocked:
        corners = [(x,y),(x+1,y),(x+1,y+1),(x,y+1)]
        if box_collision(a,b,(x,y,x+1,y+1)):
            return 0.0
        for c,d in zip(corners,corners[1:]+corners[:1]):
            clearance = min(clearance,point_segment_distance(a,c,d),point_segment_distance(b,c,d),
                            point_segment_distance(c,a,b),point_segment_distance(d,a,b))
    return clearance


def route_metrics(grid, path, start, goal, discrete=False):
    if not isinstance(path,list) or not 1 <= len(path) <= 200001:
        raise ValueError('bounded route required')
    points = [point(p) for p in path]
    if math.dist(points[0],start) > 1e-8 or math.dist(points[-1],goal) > 1e-8:
        raise ValueError('route endpoints do not match task')
    segments = list(zip(points,points[1:])) or [(points[0],points[0])]
    if not all(independent_clear(grid,a,b) for a,b in segments):
        raise ValueError('independent route collision check failed')
    if discrete and any(not all(float(v-.5).is_integer() for v in p) for p in points):
        raise ValueError('grid route must use cell centers')
    if discrete and any(not math.isclose(math.dist(a,b),1) for a,b in zip(points,points[1:])):
        raise ValueError('grid route must use four-neighbor unit moves')
    # Independent ray-crossing summation using original marker locations.
    signature = [0]*len(grid.markers)
    for a,b in zip(points,points[1:]):
        for i,(mx,my) in enumerate(grid.markers):
            sign = int(a[1] <= my < b[1]) - int(b[1] <= my < a[1])
            if sign and a[0]+(my-a[1])*(b[0]-a[0])/(b[1]-a[1]) > mx:
                signature[i] += sign
    return {'length': sum(math.dist(a,b) for a,b in zip(points,points[1:])), 'signature': signature,
            'minimum_surface_clearance': min(center_clearance(grid,a,b) for a,b in segments)-grid.radius,
            'geometric_path_key': [[p[0],p[1]] for p in points], 'collision_valid': True}


def clearance_sensitivity(rows, radii):
    results=[]
    for radius in radii:
        grid=Grid(rows,radius)
        rejected=[]
        for y in range(grid.height):
            for x in range(grid.width):
                p=(x+.5,y+.5)
                if (x,y) not in grid.free and center_clearance(grid,p,p) > grid.radius:
                    rejected.append([x,y])
        results.append({'radius':grid.radius,'conservative_free_centers':len(grid.free),
                        'disk_feasible_but_box_rejected_centers':rejected,
                        'scope':'Cell-center witnesses only; not a continuous path feasibility proof'})
    return results
