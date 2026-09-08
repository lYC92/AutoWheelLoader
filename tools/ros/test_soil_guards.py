#!/usr/bin/env python3
"""Whole-vehicle full-bucket and domain-exit protection, without teleporting."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from run_ab_cycle import CycleDriver
from run_transfer_scenario import connect, prepare_pose, drive_leg, TRACE_COLUMNS
from trajectory_control import Pose
from test_loader_soil_coupling import joint_position


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--capacity",type=float,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    if not math.isfinite(args.capacity) or args.capacity<=0:
        raise ValueError("capacity must be positive and finite")
    rclpy.init()
    node=CycleDriver("/loader/ground_truth/odometry")
    result={"status":"running","pose_source":"ground_truth","test_capacity_m3":args.capacity,"checks":[],"legs":[]}
    trace=[]
    args.output.parent.mkdir(parents=True,exist_ok=True)

    def wait(condition,action,seconds):
        deadline=node.now()+seconds
        wall=time.monotonic()+max(120,seconds*10)
        while node.now()<deadline and time.monotonic()<wall:
            rclpy.spin_once(node,timeout_sec=0.01)
            node.check(); node.check_material()
            if condition(): return
            action()
        raise RuntimeError("guard scenario timed out")

    def settle(seconds,action=lambda:node.send()):
        start=node.now()
        wait(lambda:node.now()-start>=seconds,action,seconds+2)

    def checked(name):
        result["checks"].append(name)
        print("PASS "+name,flush=True)

    try:
        connect(node)
        deadline=time.monotonic()+10
        while (node.bucket is None or node.terrain is None) and time.monotonic()<deadline:
            node.send(); rclpy.spin_once(node,timeout_sec=0.02)
        node.check_material()
        prepare_pose(node,0,0)
        node.traction_limit=18000.0
        wait(lambda:node.bucket.bucket_material_volume_m3>=args.capacity-1e-8,
             lambda:node.send(1,0.8,0.0,False),25)
        settle(0.5)
        before=list(node.terrain.height_grid_m)
        start_x=node.pose().x
        settle(1.0,lambda:node.send(1,0.8,0.0,False))
        settle(0.5)
        if node.pose().x-start_x<0.05:
            raise RuntimeError("full-bucket test did not move the cutting edge")
        if before!=list(node.terrain.height_grid_m) or abs(node.bucket.bucket_material_volume_m3-args.capacity)>1e-8:
            raise RuntimeError("full bucket continued removing soil")
        result["full_bucket_extra_travel_m"]=node.pose().x-start_x
        checked("full bucket keeps terrain and payload unchanged during further forward motion")

        prepare_pose(node,0.35,0.25)
        node.traction_limit=8000.0
        result["legs"].append(drive_leg(node,Pose(-8,0,0),-1,trace))
        # Move the rear axle itself beyond Y=24, with the front frame and
        # cutting edge facing further out. This is an intentional domain exit.
        result["legs"].append(drive_leg(node,Pose(4,26,math.pi/2),1,trace))
        ymax=node.terrain.origin_y_m+node.terrain.rows*node.terrain.cell_size_m
        if node.pose().y<=ymax+0.5:
            raise RuntimeError("vehicle did not leave the terrain domain")
        result["outside_rear_pose"]=vars(node.pose())
        before=list(node.terrain.height_grid_m)
        dumped_before=node.terrain.dumped_volume_m3
        node.manual_tilt=-1.0
        settle(3.0)
        if joint_position(node.state,"bucket_tilt_joint")> -0.35:
            raise RuntimeError("bucket did not reach the unloading tilt threshold")
        result["outside_tilt_rad"]=joint_position(node.state,"bucket_tilt_joint")
        if before!=list(node.terrain.height_grid_m) or abs(node.bucket.bucket_material_volume_m3-args.capacity)>1e-8:
            raise RuntimeError("domain-exit unloading lost payload or changed terrain")
        if node.terrain.dumped_volume_m3!=dumped_before:
            raise RuntimeError("domain-exit unloading changed the deposit ledger")
        checked("outside-domain unloading preserves the full payload and terrain")

        prepare_pose(node,0.35,0.25)
        result["legs"].append(drive_leg(node,Pose(4,12,math.pi/2),-1,trace))
        node.manual_tilt=-1.0
        wait(lambda:node.bucket.bucket_material_volume_m3<1e-8,lambda:node.send(),8)
        settle(0.5)
        deposited=node.terrain.dumped_volume_m3-dumped_before
        if abs(deposited-args.capacity)>1e-8:
            raise RuntimeError("retained material was not deposited after returning inside")
        checked("returning inside allows the retained payload to unload")
        result.update(status="passed",deposited_m3=deposited,
                      relative_conservation_error=node.terrain.relative_volume_conservation_error)
    except BaseException as error:
        result.update(status="failed",reason=str(error))
        raise
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.send(emergency=True); rclpy.spin_once(node,timeout_sec=0.02)
        result.update(trace_columns=TRACE_COLUMNS,trace=trace)
        args.output.write_text(json.dumps(result,indent=2),encoding="utf-8")
        node.destroy_node(); rclpy.try_shutdown()


if __name__=="__main__":
    main()
