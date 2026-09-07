#!/usr/bin/env python3
"""Force-controlled reverse/forward turning validation before A/B integration."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from loader_sim_msgs.msg import VehicleCommand, VehicleState
from trajectory_control import Limits, Pose, Tracker, transfer_path
from test_loader_soil_coupling import joint_position


class Driver(Node):
    def __init__(self, topic):
        super().__init__("loader_transfer_validation", parameter_overrides=[Parameter("use_sim_time",value=True)])
        self.state = self.odom = None
        self.hold_integral={"lift_joint":0.0,"bucket_tilt_joint":0.0}
        self.last_hold_time=0.0
        self.speed_integral=0.0
        self.received = {"state": 0.0, "odom": 0.0}
        self.create_subscription(VehicleState,"/loader/state",lambda m:self.receive("state",m),10)
        self.create_subscription(Odometry,topic,lambda m:self.receive("odom",m),10)
        self.publisher=self.create_publisher(VehicleCommand,"/loader/command",10)
        self.path_publisher=self.create_publisher(RosPath,"/loader/planned_path",10)

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
        for key in ("state","odom"):
            m=getattr(self,key)
            if m is None or time.monotonic()-self.received[key]>1.0:
                raise RuntimeError(f"{key} feedback missing/stale")
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
            cmd.lift_valve_command=hold("lift_joint",0.35,0.15)
            cmd.tilt_valve_command=hold("bucket_tilt_joint",0.25,0.08)
            speed=self.state.longitudinal_speed_mps
            error=abs(target_speed)-gear*speed
            self.speed_integral=0.0 if brake else max(0,min(4000,self.speed_integral+2000*error*dt))
            cmd.traction_torque_nm=float(gear*min(8000,max(0,1200+8000*error+self.speed_integral))) if not brake else 0.0
            cmd.brake_command=1.0 if brake else float(min(0.3,max(0,-error*0.2)))
        else:
            cmd.brake_command=1.0
        self.publisher.publish(cmd)


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
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            node.send()
            rclpy.spin_once(node,timeout_sec=0.02)
            if (node.state and node.odom and node.now()>0 and not node.state.fault_flags
                    and not node.state.emergency_stop_active):
                break
        if (node.state is None or node.odom is None or node.state.fault_flags
                or node.state.emergency_stop_active):
            raise RuntimeError("pose/control startup timeout")
        ready_since=None
        deadline=node.now()+10
        wall_deadline=time.monotonic()+120
        while node.now()<deadline and time.monotonic()<wall_deadline:
            node.send()
            rclpy.spin_once(node,timeout_sec=0.02)
            node.check()
            error=max(abs(joint_position(node.state,"lift_joint")-0.35),abs(joint_position(node.state,"bucket_tilt_joint")-0.25))
            ready_since=node.now() if error<0.06 and ready_since is None else ready_since
            if error>=0.06:
                ready_since=None
            if ready_since is not None and node.now()-ready_since>0.5:
                break
        else:
            raise RuntimeError(f"travel bucket pose preparation timeout: lift={joint_position(node.state, 'lift_joint'):.4f}, tilt={joint_position(node.state, 'bucket_tilt_joint'):.4f}")
        origin=node.pose()
        # On open ground outside the soil pile. Validate gear changes only
        # after parking; A/B task routing and obstacle envelopes come later.
        goals=[(-1,Pose(origin.x-16,origin.y-16,math.pi/2)),(1,origin)]
        for gear,goal in goals:
            tracker=Tracker(transfer_path(node.pose(),goal,gear),gear)
            tracker.last_articulation=joint_position(node.state,"articulation_joint")
            path=RosPath()
            path.header.frame_id="world"
            path.header.stamp=node.get_clock().now().to_msg()
            for p in tracker.path:
                pose=PoseStamped(); pose.header=path.header
                pose.pose.position.x=p.x; pose.pose.position.y=p.y
                pose.pose.orientation.z=math.sin(p.yaw/2); pose.pose.orientation.w=math.cos(p.yaw/2)
                path.poses.append(pose)
            node.path_publisher.publish(path)
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
            outcome["legs"].append({"gear":gear,"rmse_m":rmse,"final_pose":vars(node.pose())})
            print(f"PASS turning transfer leg: gear={gear}, RMSE={rmse:.3f}m",flush=True)
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
        outcome["trace_columns"]=["sim_time","gear","rear_x","rear_y","yaw","target_articulation","path_error","actual_articulation","measured_speed","target_speed"]
        outcome["trace"]=records
        args.output.write_text(json.dumps(outcome,indent=2),encoding="utf-8")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=="__main__":
    main()
