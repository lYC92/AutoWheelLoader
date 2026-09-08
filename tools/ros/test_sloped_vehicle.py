#!/usr/bin/env python3
"""Actual unequal blade heights while driving over an elevated wheel track."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer,TransformListener,TransformException
from run_ab_cycle import CycleDriver
from run_transfer_scenario import connect,prepare_pose


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    rclpy.init();node=CycleDriver('/loader/ground_truth/odometry');buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    samples=[];result={'status':'running','fixture':'0.15 m elevated left wheel track; flat rigid soil base'}
    try:
        connect(node);prepare_pose(node,0,0);node.traction_limit=18000.
        deadline=node.now()+25;wall=time.monotonic()+180
        while node.now()<deadline and time.monotonic()<wall:
            rclpy.spin_once(node,timeout_sec=.01);node.check();node.check_material()
            try:tf=buffer.lookup_transform('base_link','bucket',Time()).transform
            except TransformException:continue
            q=tf.rotation;t=tf.translation;rotation=Rotation.from_quat([q.x,q.y,q.z,q.w])
            q=node.odom.pose.pose.orientation;p=node.odom.pose.pose.position
            world=Rotation.from_quat([q.x,q.y,q.z,q.w])
            blade=world.apply(rotation.apply([[1.025,-1.35,-.625],[1.025,1.35,-.625]])+[t.x,t.y,t.z])+[p.x,p.y,p.z]
            if node.bucket.material_inflow_m3ps>1e-5:
                samples.append([node.now(),*blade[0],*blade[1],node.bucket.bucket_material_volume_m3])
            if node.bucket.bucket_material_volume_m3>.2:break
            node.send(1,.6,0.,False)
        if not samples or node.bucket.bucket_material_volume_m3<.1:raise RuntimeError('sloped fixture did not excavate material')
        height_difference=max(abs(s[3]-s[6]) for s in samples)
        if height_difference<.05:raise RuntimeError('blade remained level during cutting')
        if abs(node.terrain.relative_volume_conservation_error)>1e-6:raise RuntimeError('sloped material conservation failed')
        result.update(status='passed',maximum_blade_height_difference_m=height_difference,payload_m3=node.bucket.bucket_material_volume_m3,
                      relative_conservation_error=node.terrain.relative_volume_conservation_error)
        print('PASS sloped vehicle blade: '+json.dumps(result),flush=True)
    except BaseException as error:result.update(status='failed',reason=str(error));raise
    finally:
        for _ in range(5):node.send(emergency=True);rclpy.spin_once(node,timeout_sec=.02)
        result['samples_time_left_xyz_right_xyz_payload']=samples
        args.output.write_text(json.dumps(result,indent=2),encoding='utf-8');node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
