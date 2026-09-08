#!/usr/bin/env python3
"""Inject a timestamped observed obstacle into a moving vehicle's stopping corridor."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from run_transfer_scenario import Driver,connect,prepare_pose,drive_leg,TRACE_COLUMNS
from trajectory_control import Pose
from site_navigation import SafetyMap,rectangle


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    rclpy.init();node=Driver('/loader/ground_truth/odometry');node.safety_map=SafetyMap()
    result={'status':'running'};records=[];obstacle=None;injection=None;start=None
    publisher=node.create_publisher(String,'/loader/navigation/obstacles',1)
    def observe():
        nonlocal obstacle,injection
        if start is None or node.now()-start<2.:return
        if obstacle is None:
            pose=node.pose();injection=(node.now(),pose,node.state.longitudinal_speed_mps)
            obstacle=rectangle(pose.x-3.15*math.cos(pose.yaw),pose.y-3.15*math.sin(pose.yaw),pose.yaw,-.2,.2,1.6)
        message=String();message.data=json.dumps({'frame':'world','stamp':node.now(),'polygons':[obstacle]});publisher.publish(message)
    timer=node.create_timer(.05,observe)
    try:
        connect(node);prepare_pose(node,.35,.25);start=node.now()
        try:drive_leg(node,Pose(-8.,0.,0.),-1,records)
        except RuntimeError as error:
            if 'stopping corridor' not in str(error):raise
            result['stop_reason']=str(error)
        else:raise RuntimeError('obstacle did not stop the vehicle')
        deadline=node.now()+2.;wall=time.monotonic()+20
        while node.now()<deadline and time.monotonic()<wall:
            node.send(emergency=True);rclpy.spin_once(node,timeout_sec=.01)
        travel=math.hypot(node.pose().x-injection[1].x,node.pose().y-injection[1].y)
        if abs(injection[2])<.2:raise RuntimeError('hazard was not injected while moving')
        if abs(node.state.longitudinal_speed_mps)>.03 or travel>.6:raise RuntimeError('vehicle did not stop within the clear distance')
        if not node.safety_map.clear(node.pose(),0.):raise RuntimeError('stopped vehicle overlaps observed obstacle')
        result.update(status='passed',speed_at_detection_mps=injection[2],stopping_distance_m=travel,final_speed_mps=node.state.longitudinal_speed_mps)
        print('PASS dynamic obstacle braking: '+json.dumps(result),flush=True)
    except BaseException as error:result.update(status='failed',reason=str(error));raise
    finally:
        for _ in range(5):node.send(emergency=True);rclpy.spin_once(node,timeout_sec=.02)
        result.update(trace_columns=TRACE_COLUMNS,trace=records);args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
