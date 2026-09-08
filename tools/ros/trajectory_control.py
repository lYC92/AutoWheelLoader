"""Nominal rear-axle path geometry and feedback control, independent of ROS.

Paths use rear-axle XY and rear-frame yaw, never the bucket or articulation
origin. The ROS adapter must perform that reference-point conversion.
"""
from dataclasses import dataclass
import math


def wrap(angle):
    return math.remainder(angle, 2*math.pi)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Control:
    articulation: float
    target_speed: float
    brake: bool
    reached: bool
    cross_track_error: float


@dataclass(frozen=True)
class Limits:
    rear_axle_to_joint: float = 1.55
    front_axle_to_joint: float = 1.65
    articulation: float = 0.60
    articulation_rate: float = 0.25
    # Forward uses the front axle; reverse uses the rear axle.
    front_lookahead: float = 2.0
    reverse_lookahead: float = 2.0
    maximum_speed: float = 0.6
    maximum_deviation: float = 1.0
    stopping_deceleration: float = 0.5
    parking_distance: float = 0.25
    parking_yaw: float = math.radians(5)

    def curvature(self, articulation, front=False):
        # Rear-axle curvature with constant articulation and no lateral slip.
        a,b=(self.rear_axle_to_joint,self.front_axle_to_joint) if front else (self.front_axle_to_joint,self.rear_axle_to_joint)
        return math.sin(articulation)/(a+b*math.cos(articulation))

    def steering(self, curvature, front=False):
        low, high = -self.articulation, self.articulation
        for _ in range(45):
            mid = (low+high)/2
            if self.curvature(mid,front) < curvature:
                low = mid
            else:
                high = mid
        return (low+high)/2


def transfer_path(start: Pose, goal: Pose, gear: int, limits=Limits(), spacing=0.1, terminal_straight=4.0):
    """Tangent straight/arc/straight connector on open ground.

    Reject infeasible corners; callers must provide intermediate poses for
    obstacles, parallel offsets or U-turns. This is not collision checking.
    """
    if gear not in (-1,1) or spacing <= 0:
        raise ValueError("path requires forward/reverse gear and positive spacing")
    if not all(math.isfinite(v) for v in (*vars(start).values(),*vars(goal).values(),spacing)):
        raise ValueError("path inputs must be finite")
    d0=(gear*math.cos(start.yaw),gear*math.sin(start.yaw))
    d1=(gear*math.cos(goal.yaw),gear*math.sin(goal.yaw))
    delta=(goal.x-start.x,goal.y-start.y)
    cross=lambda a,b:a[0]*b[1]-a[1]*b[0]
    theta=wrap(goal.yaw-start.yaw)
    def line(a,b):
        distance=math.hypot(b.x-a.x,b.y-a.y)
        count=max(1,math.ceil(distance/spacing))
        return [Pose(a.x+(b.x-a.x)*i/count,a.y+(b.y-a.y)*i/count,a.yaw) for i in range(count+1)]
    if abs(theta)<math.radians(15):
        distance=math.hypot(*delta)
        if distance<0.5 or delta[0]*d0[0]+delta[1]*d0[1]<=0:
            raise ValueError("target is not ahead in the selected gear")
        # Small heading/lateral residuals after parking need a smooth local
        # connector; an exact parallel-line intersection is ill-conditioned.
        p=[(start.x,start.y),(start.x+distance*d0[0]/3,start.y+distance*d0[1]/3),
           (goal.x-distance*d1[0]/3,goal.y-distance*d1[1]/3),(goal.x,goal.y)]
        count=math.ceil(sum(math.dist(a,b) for a,b in zip(p,p[1:]))/spacing)
        result=[]
        for i in range(count+1):
            t,u=i/count,1-i/count
            xy=[u**3*p[0][k]+3*u*u*t*p[1][k]+3*u*t*t*p[2][k]+t**3*p[3][k] for k in (0,1)]
            d=[3*u*u*(p[1][k]-p[0][k])+6*u*t*(p[2][k]-p[1][k])+3*t*t*(p[3][k]-p[2][k]) for k in (0,1)]
            dd=[6*u*(p[2][k]-2*p[1][k]+p[0][k])+6*t*(p[3][k]-2*p[2][k]+p[1][k]) for k in (0,1)]
            norm=math.hypot(*d)
            if norm<1e-6 or abs(cross(d,dd))/norm**3>limits.curvature(limits.articulation):
                raise ValueError("near-parallel connector exceeds turning constraint")
            result.append(Pose(*xy,wrap(math.atan2(d[1],d[0])+(math.pi if gear<0 else 0))))
        return result
    if abs(theta)>math.radians(150):
        raise ValueError("U-turn requires intermediate waypoints")
    determinant=cross(d0,d1)
    a,b=cross(delta,d1)/determinant,cross(d0,delta)/determinant
    tangent=math.tan(abs(theta)/2)
    radius=min(8.0,(min(a,b)-terminal_straight)/tangent)
    if radius < 1/limits.curvature(limits.articulation):
        raise ValueError("corner has insufficient turning radius or terminal straight")
    trim=radius*tangent
    entry=Pose(start.x+(a-trim)*d0[0],start.y+(a-trim)*d0[1],start.yaw)
    exit=Pose(goal.x-(b-trim)*d1[0],goal.y-(b-trim)*d1[1],goal.yaw)
    sign=1 if theta>0 else -1
    center=(entry.x-sign*radius*d0[1],entry.y+sign*radius*d0[0])
    angle=math.atan2(entry.y-center[1],entry.x-center[0])
    count=math.ceil(radius*abs(theta)/spacing)
    arc=[Pose(center[0]+radius*math.cos(angle+theta*i/count),
              center[1]+radius*math.sin(angle+theta*i/count),
              wrap(start.yaw+theta*i/count)) for i in range(1,count+1)]
    return line(start,entry)+arc+line(exit,goal)[1:]


class Tracker:
    """One-gear pure-pursuit leg. Errors fail closed; never change gear in motion."""
    def __init__(self, path, gear, limits=Limits()):
        if len(path) < 2 or gear not in (-1,1):
            raise ValueError("a tracking leg needs a path and one direction")
        self.path, self.gear, self.limits = path, gear, limits
        self.progress = 0
        self.last_articulation = 0.0

    def update(self, pose, speed, dt, actual_articulation=None):
        if dt <= 0 or dt > 0.2 or not all(math.isfinite(v) for v in (*vars(pose).values(),speed,dt)):
            raise RuntimeError("invalid or stale tracking state")
        limit = self.limits
        # Restrict search to the current forward part of the ordered path.
        candidates = range(self.progress, min(len(self.path),self.progress+80))
        index = min(candidates, key=lambda i: math.hypot(self.path[i].x-pose.x,self.path[i].y-pose.y))
        error = math.hypot(self.path[index].x-pose.x,self.path[index].y-pose.y)
        if error > limit.maximum_deviation:
            raise RuntimeError("path deviation exceeds safety limit")
        self.progress = index
        if speed*self.gear < -0.05:
            return Control(self.last_articulation,0,True,False,error)
        goal = self.path[-1]
        distance = math.hypot(goal.x-pose.x,goal.y-pose.y)
        if distance < limit.parking_distance:
            if abs(speed) < 0.05 and abs(wrap(goal.yaw-pose.yaw)) > limit.parking_yaw:
                raise RuntimeError("parking heading outside tolerance; replan while stopped")
            return Control(self.last_articulation,0,True,abs(speed)<0.05,error)
        longitudinal=math.cos(pose.yaw)*(goal.x-pose.x)+math.sin(pose.yaw)*(goal.y-pose.y)
        if index == len(self.path)-1 and self.gear*longitudinal < -limit.parking_distance:
            raise RuntimeError("passed parking target; stop and replan")
        tracking_pose=pose
        # Extend the terminal tangent in the selected gear. A target collapsing
        # onto the rear axle magnifies centimetre-scale localization noise into
        # maximum steering just before reverse parking.
        tracking_path=self.path+[Pose(goal.x+self.gear*i*.1*math.cos(goal.yaw),
                                     goal.y+self.gear*i*.1*math.sin(goal.yaw),goal.yaw) for i in range(1,41)]
        tracking_index=index
        if self.gear>0:
            # Forward steering acts directly on the front heading. Tracking
            # the rear heading instead causes an initial opposite rear swing.
            articulation=self.last_articulation if actual_articulation is None else actual_articulation
            if not math.isfinite(articulation):
                raise RuntimeError("invalid articulation feedback")
            heading=pose.yaw+articulation
            tracking_pose=Pose(pose.x+limit.rear_axle_to_joint*math.cos(pose.yaw)+limit.front_axle_to_joint*math.cos(heading),
                               pose.y+limit.rear_axle_to_joint*math.sin(pose.yaw)+limit.front_axle_to_joint*math.sin(heading),heading)
            # The front axle continues beyond the rear-axle parking target.
            tracking_path=self.path+[Pose(goal.x+i*0.1*math.cos(goal.yaw),goal.y+i*0.1*math.sin(goal.yaw),goal.yaw) for i in range(1,41)]
            candidates=range(index,min(len(tracking_path),index+100))
            tracking_index=min(candidates,key=lambda i:math.hypot(tracking_path[i].x-tracking_pose.x,tracking_path[i].y-tracking_pose.y))
        target_index, arc = tracking_index, 0.0
        lookahead=limit.front_lookahead if self.gear>0 else limit.reverse_lookahead
        while target_index+1 < len(tracking_path) and arc < lookahead:
            a,b = tracking_path[target_index:target_index+2]
            arc += math.hypot(b.x-a.x,b.y-a.y)
            target_index += 1
        target = tracking_path[target_index]
        dx,dy = target.x-tracking_pose.x,target.y-tracking_pose.y
        lateral = -math.sin(tracking_pose.yaw)*dx+math.cos(tracking_pose.yaw)*dy
        desired = limit.steering(2*lateral/max(dx*dx+dy*dy,0.01),front=self.gear>0)
        step = limit.articulation_rate*dt
        steering = max(self.last_articulation-step,min(self.last_articulation+step,desired))
        self.last_articulation = steering
        target_speed = min(limit.maximum_speed,math.sqrt(2*limit.stopping_deceleration*max(0,distance-limit.parking_distance)))
        return Control(steering,self.gear*target_speed,False,False,error)
