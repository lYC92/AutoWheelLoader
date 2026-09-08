#!/usr/bin/env python3
"""Force-controlled reverse/forward turning validation before A/B integration."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import String
from loader_sim_msgs.msg import VehicleCommand, VehicleState
from trajectory_control import Limits, Pose, Tracker, transfer_path
from site_navigation import plan_route
from test_loader_soil_coupling import joint_position


TRACE_COLUMNS=["sim_time","gear","rear_x","rear_y","yaw","target_articulation","path_error","actual_articulation","measured_speed","target_speed"]

class Driver(Node):
    def __init__(self, topic):
        super().__init__("loader_transfer_validation", parameter_overrides=[Parameter("use_sim_time",value=True)])
        self.state = self.odom = None
        self.safety_map=None
        self.navigation_error=None;self.obstacle_stamp=-1.
        self.create_subscription(String,'/loader/navigation/obstacles',self.receive_obstacles,10)
        self.requires_localization_health=topic!="/loader/ground_truth/odometry"
        self.localization_health={};self.health_received=0.
        if self.requires_localization_health:
            self.create_subscription(String,"/loader/localization/health",self.receive_health,1)
        self.hold_integral={"lift_joint":0.0,"bucket_tilt_joint":0.0}
        self.last_hold_time=0.0
        self.speed_integral=0.0
        self.lift_target,self.tilt_target=0.35,0.25
        self.manual_tilt=None
        self.traction_limit=8000.0
        self.received = {"state": 0.0, "odom": 0.0}
        self.create_subscription(VehicleState,"/loader/state",lambda m:self.receive("state",m),1)
        self.create_subscription(Odometry,topic,lambda m:self.receive("odom",m),1)
        self.publisher=self.create_publisher(VehicleCommand,"/loader/command",10)
        self.path_publisher=self.create_publisher(RosPath,"/loader/planned_path",10)

    def receive_obstacles(self,message):
        if self.safety_map is None:return
        try:
            data=json.loads(message.data)
            if data['frame']!='world' or not 0<=self.now()-data['stamp']<.3:
                raise ValueError('obstacle frame/timestamp mismatch')
            self.safety_map.validate_obstacles(data['polygons'])
            self.safety_map.dynamic_obstacles=data['polygons'];self.obstacle_stamp=data['stamp']
        except (ValueError,KeyError,TypeError) as error:self.navigation_error=str(error)

    def receive_health(self,message):
        try: self.localization_health=json.loads(message.data)
        except (ValueError,TypeError): self.localization_health={}
        self.health_received=time.monotonic()

    def localization_ready(self):
        if not self.requires_localization_health: return True
        health=self.localization_health
        return (health.get("status")=="tracking" and time.monotonic()-self.health_received<.5
                and 0<=self.now()-health.get("sim_time",-1)<.3)

    def receive(self,key,message):
        setattr(self,key,message)
        self.received[key]=time.monotonic()

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def pose(self):
        if self.odom.header.frame_id != "world":
            raise RuntimeError("pose must be transformed to world before tracking")
        p,q=self.odom.pose.pose.position,self.odom.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return Pose(p.x-1.55*math.cos(yaw),p.y-1.55*math.sin(yaw),yaw)

    def check(self):
        # Consume ready latest-value feedback before comparing its timestamp.
        # Keep-last-one prevents large terrain/path work from accumulating old
        # control states. Freshness limits themselves remain unchanged.
        for _ in range(4):rclpy.spin_once(self,timeout_sec=0.)
        if self.navigation_error or (self.safety_map and self.safety_map.dynamic_obstacles and self.now()-self.obstacle_stamp>.5):
            self.send(emergency=True)
            raise RuntimeError(self.navigation_error or 'dynamic obstacle observation expired')
        if not self.localization_ready():
            # Continue brake/hold commands while waiting for qualified fusion
            # output. No estimator frame or pose-source fallback is permitted.
            deadline=time.monotonic()+30
            held_steering=joint_position(self.state,"articulation_joint") if self.state else 0.
            while not self.localization_ready() and time.monotonic()<deadline:
                self.send(brake=True,steering=held_steering)
                rclpy.spin_once(self,timeout_sec=.01)
            if not self.localization_ready():
                raise RuntimeError("localization unavailable: stopped and recovery timed out")
        for key in ("state","odom"):
            m=getattr(self,key)
            if m is None or time.monotonic()-self.received[key]>1.0:
                raise RuntimeError(f"{key} feedback missing/stale: wall_age={time.monotonic()-self.received[key]:.3f}s, sim_time={self.now():.3f}s")
            age=self.now()-(m.header.stamp.sec+m.header.stamp.nanosec/1e9)
            if age < -0.05 or age > 0.3:
                raise RuntimeError(f"{key} timestamp is stale: {age}")
        if self.state.fault_flags or self.state.emergency_stop_active:
            raise RuntimeError("vehicle safety state prevents driving")

    def send(self, gear=0, target_speed=0.0, steering=0.0, brake=True, emergency=False):
        cmd=VehicleCommand()
        cmd.header.stamp=self.get_clock().now().to_msg()
        cmd.gear=gear
        cmd.target_articulation_angle_rad=float(steering)
        cmd.emergency_stop=emergency
        if self.state:
            dt=max(0.0,min(0.1,self.now()-self.last_hold_time))
            self.last_hold_time=self.now()
            def hold(name,target,bias):
                index=list(self.state.joint_state.name).index(name)
                error=target-self.state.joint_state.position[index]
                velocity=self.state.joint_state.velocity[index]
                raw=bias+3*error-0.8*velocity+self.hold_integral[name]
                if abs(raw)<1 or raw*error<0:
                    self.hold_integral[name]=max(-0.3,min(0.3,self.hold_integral[name]+0.4*error*dt))
                return max(-1.0,min(1.0,raw))
            cmd.lift_valve_command=hold("lift_joint",self.lift_target,0.15)
            cmd.tilt_valve_command=hold("bucket_tilt_joint",self.tilt_target,0.08) if self.manual_tilt is None else float(self.manual_tilt)
            speed=self.state.longitudinal_speed_mps
            error=abs(target_speed)-gear*speed
            self.speed_integral=0.0 if brake else max(0,min(4000,self.speed_integral+2000*error*dt))
            cmd.traction_torque_nm=float(gear*min(self.traction_limit,max(0,1200+8000*error+self.speed_integral))) if not brake else 0.0
            cmd.brake_command=1.0 if brake else float(min(0.3,max(0,-error*0.2)))
        else:
            cmd.brake_command=1.0
        self.publisher.publish(cmd)


def connect(node):
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        node.send()
        rclpy.spin_once(node,timeout_sec=0.02)
        if (node.state and node.odom and node.now()>0 and not node.state.fault_flags
                and not node.state.emergency_stop_active and node.localization_ready()):
            break
    if (node.state is None or node.odom is None or node.state.fault_flags
            or node.state.emergency_stop_active or not node.localization_ready()):
        raise RuntimeError("pose/control startup timeout")


def prepare_pose(node,lift,tilt):
    node.lift_target,node.tilt_target=lift,tilt
    node.manual_tilt=None
    node.hold_integral={"lift_joint":0.0,"bucket_tilt_joint":0.0}
    ready_since=None
    deadline=node.now()+10
    wall_deadline=time.monotonic()+120
    while node.now()<deadline and time.monotonic()<wall_deadline:
        node.send()
        rclpy.spin_once(node,timeout_sec=0.02)
        node.check()
        error=max(abs(joint_position(node.state,"lift_joint")-lift),abs(joint_position(node.state,"bucket_tilt_joint")-tilt))
        ready_since=node.now() if error<0.06 and ready_since is None else ready_since
        if error>=0.06:
            ready_since=None
        if ready_since is not None and node.now()-ready_since>0.5:
            break
    else:
        raise RuntimeError(f"travel bucket pose preparation timeout: lift={joint_position(node.state, 'lift_joint'):.4f}, tilt={joint_position(node.state, 'bucket_tilt_joint'):.4f}")


def publish_path(node,poses):
    path=RosPath()
    path.header.frame_id="world"
    path.header.stamp=node.get_clock().now().to_msg()
    for p in poses:
        pose=PoseStamped(); pose.header=path.header
        pose.pose.position.x=p.x; pose.pose.position.y=p.y
        pose.pose.orientation.z=math.sin(p.yaw/2); pose.pose.orientation.w=math.cos(p.yaw/2)
        path.poses.append(pose)
    node.path_publisher.publish(path)


def compute_path(node,goal,gear):
    start=node.pose();site=copy.deepcopy(node.safety_map)
    # Search can take longer than a command watchdog interval. Keep the
    # vehicle braked and process feedback while the bounded search runs.
    with ThreadPoolExecutor(max_workers=1) as executor:
        future=executor.submit(plan_route,start,goal,gear,site) if site else executor.submit(transfer_path,start,goal,gear)
        while not future.done():
            node.send(brake=True,steering=joint_position(node.state,'articulation_joint'))
            rclpy.spin_once(node,timeout_sec=.01)
        return future.result()


def drive_leg(node,goal,gear,records):
    path=compute_path(node,goal,gear)
    tracker=Tracker(path,gear)
    tracker.last_articulation=joint_position(node.state,"articulation_joint")
    publish_path(node,tracker.path)
    deadline=node.now()+90
    wall_deadline=time.monotonic()+600
    previous=node.now()
    errors=[]
    progress_pose=node.pose()
    progress_time=node.now()
    while node.now()<deadline and time.monotonic()<wall_deadline:
        rclpy.spin_once(node,timeout_sec=0.01)
        node.check()
        dt=node.now()-previous
        if dt<0.02:
            continue
        if dt>0.2:
            # The vehicle was held while localization recovered. Replan from
            # its current estimate, preserving the same destination and gear.
            remaining=tracker.path[tracker.progress:]
            nearest=min(remaining,key=lambda p:math.hypot(p.x-node.pose().x,p.y-node.pose().y))
            if len(remaining)>=2 and math.hypot(nearest.x-node.pose().x,nearest.y-node.pose().y)<.3 and (not node.safety_map or node.safety_map.clear_path(remaining,gear)):
                renewed=remaining
            else:renewed=compute_path(node,goal,gear)
            tracker=Tracker(renewed,gear)
            tracker.last_articulation=joint_position(node.state,"articulation_joint")
            publish_path(node,renewed)
            deadline+=dt
            previous=node.now();progress_time=node.now();progress_pose=node.pose()
            continue
        if node.safety_map and not node.safety_map.stopping_corridor_clear(node.pose(),
                joint_position(node.state,"articulation_joint"),node.state.longitudinal_speed_mps):
            node.send(emergency=True)
            raise RuntimeError("obstacle or worksite boundary in stopping corridor")
        command=tracker.update(node.pose(),node.state.longitudinal_speed_mps,dt,
                               joint_position(node.state,"articulation_joint"))
        previous=node.now()
        node.send(gear,command.target_speed,command.articulation,command.brake)
        errors.append(command.cross_track_error)
        if math.hypot(node.pose().x-progress_pose.x,node.pose().y-progress_pose.y)>0.1:
            progress_pose=node.pose(); progress_time=node.now()
        elif node.now()-progress_time>8 and not command.brake:
            raise RuntimeError("vehicle made no path progress for 8 s; drive stopped")
        pose=node.pose()
        records.append([node.now(),gear,pose.x,pose.y,pose.yaw,command.articulation,command.cross_track_error,
                        joint_position(node.state,"articulation_joint"),
                        node.state.longitudinal_speed_mps,command.target_speed])
        if command.reached:
            break
    else:
        raise RuntimeError("transfer tracking timed out")
    rmse=math.sqrt(sum(e*e for e in errors)/len(errors))
    if rmse>0.3:
        raise RuntimeError(f"cross-track RMSE {rmse} exceeds 0.3 m")
    result={"gear":gear,"rmse_m":rmse,"goal":vars(goal),"final_pose":vars(node.pose()),
            "parking_error_m":math.hypot(node.pose().x-goal.x,node.pose().y-goal.y),
            "parking_yaw_error_deg":math.degrees(abs(math.remainder(node.pose().yaw-goal.yaw,2*math.pi)))}
    print(f"PASS turning transfer leg: gear={gear}, RMSE={rmse:.3f}m",flush=True)
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--pose-source",choices=["ground_truth","estimated"],required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    topic="/loader/ground_truth/odometry" if args.pose_source=="ground_truth" else "/loader/localization/odometry"
    rclpy.init()
    node=Driver(topic)
    records=[]
    outcome={"status":"running","pose_source":args.pose_source,"legs":[]}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(outcome),encoding="utf-8")
    try:
        print(f"INFO explicit {args.pose_source} tracking validation; not final localization acceptance",flush=True)
        connect(node)
        prepare_pose(node,0.35,0.25)
        origin=node.pose()
        # On open ground outside the soil pile. Validate gear changes only
        # after parking; A/B task routing and obstacle envelopes come later.
        goals=[(-1,Pose(origin.x-16,origin.y-16,math.pi/2)),(1,origin)]
        for gear,goal in goals:
            outcome["legs"].append(drive_leg(node,goal,gear,records))
        outcome["status"]="passed"
    except Exception as error:
        outcome.update(status="failed",reason=str(error))
        raise
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.send(emergency=True)
                rclpy.spin_once(node,timeout_sec=0.02)
        if node.state:
            outcome["final_joints"]=dict(zip(node.state.joint_state.name,node.state.joint_state.position))
        outcome["trace_columns"]=TRACE_COLUMNS
        outcome["trace"]=records
        args.output.write_text(json.dumps(outcome,indent=2),encoding="utf-8")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=="__main__":
    main()
