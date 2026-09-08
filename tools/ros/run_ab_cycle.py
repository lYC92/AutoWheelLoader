#!/usr/bin/env python3
"""Nominal feedback-driven A/B cycle, explicitly selecting its pose source."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
import yaml
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from loader_sim_msgs.msg import BucketInteraction, TerrainState
from run_transfer_scenario import Driver, connect, prepare_pose, drive_leg, TRACE_COLUMNS
from trajectory_control import Pose
from site_navigation import SafetyMap


class CycleDriver(Driver):
    def __init__(self,topic):
        super().__init__(topic)
        self.bucket=self.terrain=None
        self.peak_force_n=0.
        self.image=None
        self.create_subscription(Image,"/loader/yard_camera",lambda m:setattr(self,"image",m),qos_profile_sensor_data)
        self.create_subscription(BucketInteraction,"/loader/bucket_interaction",self.receive_bucket,1)
        self.create_subscription(TerrainState,"/loader/terrain_state",lambda m:setattr(self,"terrain",m),1)
        self.phase_publisher=self.create_publisher(String,"/loader/task_state",10)

    def receive_bucket(self,message):
        self.bucket=message;f=message.bucket_wrench.force
        self.peak_force_n=max(self.peak_force_n,math.sqrt(f.x*f.x+f.y*f.y+f.z*f.z))

    def check(self):
        super().check()
        if self.bucket is not None and self.terrain is not None:
            self.check_material()

    def check_material(self):
        for name in ("bucket","terrain"):
            message=getattr(self,name)
            if message is None:
                raise RuntimeError(f"missing {name} feedback")
            age=self.now()-message.header.stamp.sec-message.header.stamp.nanosec/1e9
            if not -0.05<=age<=0.4:
                raise RuntimeError(f"stale {name} feedback")
        if self.terrain.schema_version!=2:
            raise RuntimeError("A/B requires the XY heightfield")
        if abs(self.terrain.relative_volume_conservation_error)>1e-6:
            raise RuntimeError("material conservation error")


def execute_cycle(node,config,output,pose_source='ground_truth'):
    """One cycle in an existing world. Preserve terrain and payload on return."""
    output=Path(output)
    node.safety_map=SafetyMap(**config['navigation']) if 'navigation' in config else None
    records=[]
    outcome={"status":"running","pose_source":pose_source,"config":config,"phases":[],"legs":[]}
    output.parent.mkdir(parents=True,exist_ok=True)
    def transition(name):
        if node.image is not None:
            from PIL import Image as PillowImage
            msg=node.image
            if msg.encoding in ("rgb8","bgr8"):
                capture_dir=output.parent/"yard_frames"
                capture_dir.mkdir(exist_ok=True)
                previous=outcome.get("phase","initial")
                PillowImage.frombytes("RGB",(msg.width,msg.height),bytes(msg.data),
                                      "raw","RGB" if msg.encoding=="rgb8" else "BGR",msg.step).save(capture_dir/f"{previous}.png")
        outcome["phase"]=name
        outcome["phases"].append({"name":name,"sim_time":node.now()})
        output.write_text(json.dumps(outcome,indent=2),encoding="utf-8")
        message=String(); message.data=json.dumps({"phase":name,"pose_source":pose_source})
        node.phase_publisher.publish(message)
        print(f"STATE {name}",flush=True)
    def until(condition,action,timeout):
        deadline=node.now()+timeout
        wall_deadline=time.monotonic()+max(120,timeout*10)
        while node.now()<deadline and time.monotonic()<wall_deadline:
            rclpy.spin_once(node,timeout_sec=0.01)
            node.check()
            node.check_material()
            if condition():
                return
            action()
        raise RuntimeError(f"{outcome['phase']} feedback transition timed out")
    def travel(name,goal,gear):
        transition(name)
        node.check_material()
        outcome["legs"].append(drive_leg(node,goal,gear,records))
        node.check_material()
    try:
        transition("connecting")
        connect(node)
        wait=time.monotonic()+10
        while (node.terrain is None or node.bucket is None) and time.monotonic()<wait:
            node.send(); rclpy.spin_once(node,timeout_sec=0.02)
        node.check_material()
        origin=node.pose()
        node.peak_force_n=0.
        outcome["initial_volume_m3"]=node.terrain.initial_volume_m3
        transition("prepare_at_A")
        prepare_pose(node,0,0)
        transition("dig_at_A")
        node.traction_limit=18000.0
        # A slow speed loop builds enough traction to overcome nominal sand
        # resistance. Keep the requested torques below the vehicle contract.
        def dig():
            progress=(node.pose().x-origin.x)*math.cos(origin.yaw)+(node.pose().y-origin.y)*math.sin(origin.yaw)
            if progress>config.get('maximum_dig_advance_m',config['maximum_dig_rear_x_m']-origin.x):
                raise RuntimeError("passed the allowed digging region without filling")
            node.send(1,0.8,0.0,False)
        until(lambda:node.bucket.bucket_material_volume_m3>=config["target_payload_m3"],
              dig,config["maximum_dig_time_s"])
        transition("curl_and_lift")
        prepare_pose(node,0.35,0.25)
        loaded=node.bucket.bucket_material_volume_m3
        outcome["loaded_volume_m3"]=loaded
        node.traction_limit=8000.0
        staging=Pose(*config["staging_pose"])
        parking=Pose(*config["unload_pose"])
        travel("reverse_clear_A",staging,-1)
        travel("forward_turn_to_B",parking,1)
        transition("unload_at_B")
        before=list(node.terrain.height_grid_m)
        before_dumped=node.terrain.dumped_volume_m3
        node.manual_tilt=-1.0
        until(lambda:node.bucket.bucket_material_volume_m3<=loaded*0.01,
              lambda:node.send(brake=True),8)
        node.manual_tilt=None
        # Require a terrain publication after the empty-bucket observation.
        empty_stamp=node.bucket.header.stamp.sec+node.bucket.header.stamp.nanosec/1e9
        until(lambda:node.terrain.header.stamp.sec+node.terrain.header.stamp.nanosec/1e9>=empty_stamp,
              lambda:node.send(brake=True),2)
        center=config["unload_center"]
        inside=total=0.0
        terrain=node.terrain
        for i,(a,b) in enumerate(zip(before,terrain.height_grid_m)):
            volume=max(0,b-a)*terrain.cell_size_m**2
            x=terrain.domain_min_m+(i%terrain.columns+0.5)*terrain.cell_size_m
            y=terrain.origin_y_m+(i//terrain.columns+0.5)*terrain.cell_size_m
            total+=volume
            if math.hypot(x-center[0],y-center[1])<=config["unload_radius_m"]:
                inside+=volume
        unloaded=terrain.dumped_volume_m3-before_dumped
        if unloaded<loaded*0.95 or total<=0 or inside/total<0.9:
            raise RuntimeError(f"B deposition failed: loaded={loaded}, unloaded={unloaded}, inside={inside}, total={total}")
        if abs(total-unloaded)>1e-6:
            raise RuntimeError("deposited grid volume disagrees with the transfer ledger")
        outcome.update(unloaded_volume_m3=unloaded,inside_B_fraction=inside/total,
                       relative_conservation_error=terrain.relative_volume_conservation_error)
        transition("raise_after_unloading")
        prepare_pose(node,0.35,0.25)
        travel("reverse_clear_B",staging,-1)
        travel("forward_return_to_A",Pose(*config["return_pose"]) if "return_pose" in config else origin,1)
        transition("complete")
        outcome["status"]="passed"
        print(f"PASS A/B cycle: loaded={loaded:.6f}m3 unloaded={unloaded:.6f}m3 inside_B={inside/total:.1%}",flush=True)
    except BaseException as error:
        outcome.update(status="failed",reason=str(error))
        raise
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.send(emergency=outcome["status"]!="passed"); rclpy.spin_once(node,timeout_sec=0.02)
        if node.odom is not None:
            outcome["final_pose"]=vars(node.pose())
        if node.bucket is not None:
            outcome["final_payload_m3"]=node.bucket.bucket_material_volume_m3
        outcome["peak_force_n"]=node.peak_force_n
        outcome["last_sim_time"]=node.now()
        outcome["trace_columns"]=TRACE_COLUMNS
        outcome["trace"]=records
        output.write_text(json.dumps(outcome,indent=2),encoding="utf-8")
    return outcome


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--pose-source',choices=['ground_truth','estimated'],required=True)
    parser.add_argument('--config',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();config=yaml.safe_load(args.config.read_text(encoding='utf-8'))
    topic='/loader/ground_truth/odometry' if args.pose_source=='ground_truth' else '/loader/localization/odometry'
    rclpy.init();node=CycleDriver(topic)
    try:execute_cycle(node,config,args.output,args.pose_source)
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
