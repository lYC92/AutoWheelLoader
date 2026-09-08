"""Articulated body envelopes, swept collision checks and bounded route search."""
from dataclasses import dataclass, field
import heapq
import itertools
import math
from trajectory_control import Limits, Pose, transfer_path, wrap


def rectangle(x,y,yaw,lo,hi,half_width,margin=0):
    c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*a-s*b,y+s*a+c*b) for a,b in
            [(lo-margin,-half_width-margin),(hi+margin,-half_width-margin),
             (hi+margin,half_width+margin),(lo-margin,half_width+margin)]]


def intersects(a,b):
    if (max(p[0] for p in a)<min(p[0] for p in b) or max(p[0] for p in b)<min(p[0] for p in a)
        or max(p[1] for p in a)<min(p[1] for p in b) or max(p[1] for p in b)<min(p[1] for p in a)):
        return False
    for polygon in (a,b):
        for p,q in zip(polygon,polygon[1:]+polygon[:1]):
            axis=(q[1]-p[1],p[0]-q[0])
            aa=[axis[0]*x+axis[1]*y for x,y in a]
            bb=[axis[0]*x+axis[1]*y for x,y in b]
            if max(aa)<min(bb) or max(bb)<min(aa): return False
    return True


@dataclass
class SafetyMap:
    bounds: tuple=(-20,20,-10,28)
    obstacles: list=field(default_factory=list)
    margin: float=0.35
    dynamic_obstacles: list=field(default_factory=list)
    tracking_reserve: float=0.0
    parking_endpoints: tuple=()

    def __post_init__(self):
        if len(self.bounds)!=4 or not all(math.isfinite(v) for v in (*self.bounds,self.margin)) or self.margin<0:
            raise ValueError('invalid worksite bounds or margin')
        if self.bounds[0]>=self.bounds[1] or self.bounds[2]>=self.bounds[3]:raise ValueError('empty worksite')
        self.validate_obstacles(self.obstacles)

    @staticmethod
    def validate_obstacles(polygons):
        for polygon in polygons:
            if len(polygon)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in polygon):
                raise ValueError('invalid obstacle coordinates')
            turns=[]
            for a,b,c in zip(polygon,polygon[1:]+polygon[:1],polygon[2:]+polygon[:2]):
                turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
            if not (all(t>0 for t in turns) or all(t<0 for t in turns)):
                raise ValueError('obstacles must be strictly convex ordered polygons')

    def clear(self,pose,articulation):
        if not all(math.isfinite(v) for v in (*vars(pose).values(),articulation)): return False
        # Preserve the mandatory stop envelope everywhere. Extra front-swing
        # allowance grows with steering and distance from parking endpoints;
        # straight exits retain their measured body clearance.
        scale=min([1.0]+[math.hypot(pose.x-p.x,pose.y-p.y)/3.0 for p in self.parking_endpoints])
        margin=self.margin+self.tracking_reserve*scale*min(1.,abs(articulation)/.15)
        # Rear axle reference. Front envelope includes boom and bucket overhang.
        joint=(pose.x+1.55*math.cos(pose.yaw),pose.y+1.55*math.sin(pose.yaw))
        bodies=[rectangle(pose.x,pose.y,pose.yaw,-2.0,1.55,1.55,margin),
                rectangle(*joint,pose.yaw+articulation,-0.25,4.8,1.55,margin)]
        xmin,xmax,ymin,ymax=self.bounds
        return all(all(xmin<=x<=xmax and ymin<=y<=ymax for x,y in body)
                   and not any(intersects(body,o) for o in self.obstacles+self.dynamic_obstacles) for body in bodies)

    def clear_path(self,path,gear):
        limits=Limits()
        for a,b in zip(path,path[1:]):
            length=math.hypot(b.x-a.x,b.y-a.y)
            curvature=wrap(b.yaw-a.yaw)/(gear*max(length,1e-9))
            if abs(curvature)>limits.curvature(limits.articulation)+1e-5: return False
            angle=limits.steering(curvature)
            for i in range(max(1,math.ceil(length/0.1))+1):
                t=i/max(1,math.ceil(length/0.1))
                if not self.clear(Pose(a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),wrap(a.yaw+t*wrap(b.yaw-a.yaw))),angle):
                    return False
        return True

    def stopping_corridor_clear(self,pose,articulation,speed):
        limits=Limits()
        distance=speed*speed/(2*limits.stopping_deceleration)+abs(speed)*0.3+0.3
        direction=1 if speed>=0 else -1
        p=pose
        for _ in range(math.ceil(distance/0.05)+1):
            if not self.clear(p,articulation): return False
            ds=direction*0.05
            p=Pose(p.x+ds*math.cos(p.yaw),p.y+ds*math.sin(p.yaw),wrap(p.yaw+ds*limits.curvature(articulation)))
        return True


def plan_route(start,goal,gear,site,max_expansions=12000,initial_articulation=0.):
    """Try the short analytic connector, then a one-gear Hybrid A* search.

    Search returns a sampled feasible route or fails explicitly. It never
    returns the unsafe direct connector when its planning budget is exhausted.
    """
    limits=Limits(articulation=.5)  # Reserve steering authority for tracking corrections.
    def connector(p,steering=0.):
        def compatible(path):
            a,b=path[:2];length=math.hypot(b.x-a.x,b.y-a.y)
            first_angle=limits.steering(wrap(b.yaw-a.yaw)/(gear*max(length,1e-9)))
            return (abs(steering)<.04 or abs(first_angle-steering)<.08) and site.clear_path(path,gear)
        try:
            path=transfer_path(p,goal,gear,limits)
            if compatible(path):return path
        except ValueError:pass
        for straight,radius in ((3.,10.),(2.,12.)):
            try:
                path=transfer_path(p,goal,gear,limits,terminal_straight=straight,preferred_radius=radius)
                if compatible(path):return path
            except ValueError:pass
        # In a narrow parking approach, remove residual heading before the
        # front overhang enters the corridor. A full-length cubic may swing
        # the bucket into an obstacle even though both endpoints are clear.
        if abs(wrap(p.yaw-goal.yaw))<math.radians(15):
            for straight in (6.,5.,4.,3.5,3.):
                entry=Pose(goal.x-gear*straight*math.cos(goal.yaw),goal.y-gear*straight*math.sin(goal.yaw),goal.yaw)
                try:
                    path=transfer_path(p,entry,gear,limits)+transfer_path(entry,goal,gear,limits)[1:]
                    if compatible(path):return path
                except ValueError:pass
        return None
    direct=connector(start,initial_articulation)
    if direct is not None: return direct
    if not site.clear(start,initial_articulation) or not site.clear(goal,0):
        raise RuntimeError("start or parking body envelope is obstructed")
    serial=itertools.count()
    def key(p,angle): return (round(p.x/0.5),round(p.y/0.5),round(wrap(p.yaw)/math.radians(10)),round(angle/0.15))
    def heuristic(p): return math.hypot(p.x-goal.x,p.y-goal.y)+2*abs(wrap(p.yaw-goal.yaw))
    queue=[(heuristic(start),next(serial),0,start,initial_articulation,[start])]; costs={key(start,initial_articulation):0}
    for _ in range(max_expansions):
        if not queue: break
        _,_,cost,p,steering,path=heapq.heappop(queue)
        if cost>costs.get(key(p,steering),math.inf)+1e-9: continue
        shot=connector(p,steering)
        if shot is not None: return path+shot[1:]
        for target in (-0.5,-0.25,0,0.25,0.5):
            q=p; angle=steering; segment=[]
            for step in range(20):
                dt=0.1; speed=gear*0.6
                rate=max(-limits.articulation_rate,min(limits.articulation_rate,(target-angle)/dt))
                angle+=rate*dt
                # Integrate the articulated primitive, including rear-frame
                # counter-yaw while the steering joint moves. Its envelope
                # is checked with the actual primitive articulation below.
                yaw_rate=(speed*math.sin(angle)-limits.front_axle_to_joint*rate)/(limits.front_axle_to_joint+limits.rear_axle_to_joint*math.cos(angle))
                q=Pose(q.x+speed*math.cos(q.yaw)*dt,q.y+speed*math.sin(q.yaw)*dt,wrap(q.yaw+yaw_rate*dt))
                if not site.clear(q,angle): break
                segment.append(q)
            else:
                new_cost=cost+1.2+0.4*abs(angle-steering)+0.05*abs(angle)
                state=key(q,angle)
                if new_cost<costs.get(state,math.inf):
                    costs[state]=new_cost
                    heapq.heappush(queue,(new_cost+heuristic(q),next(serial),new_cost,q,angle,path+segment))
    raise RuntimeError("no collision-free route within planning budget")
