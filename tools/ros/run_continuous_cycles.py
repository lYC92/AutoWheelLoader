#!/usr/bin/env python3
"""Continuous vehicle endurance: alternate actual A/B material transfer.

No world reset, terrain reset, artificial replenishment or vehicle teleport.
A->B batches build the receiving pile; B->A batches carry that same soil back.
"""
import argparse,copy,json,time
from pathlib import Path
import rclpy,yaml
from run_ab_cycle import CycleDriver,execute_cycle
from run_transfer_scenario import connect,prepare_pose,drive_leg,TRACE_COLUMNS
from trajectory_control import Pose
from site_navigation import SafetyMap


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cycles',type=int,required=True)
    parser.add_argument('--batch-size',type=int,default=2)
    parser.add_argument('--pose-source',choices=['ground_truth','estimated'],required=True)
    parser.add_argument('--config',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.cycles<1 or args.batch_size<1:raise ValueError('cycle and batch counts must be positive')
    config=yaml.safe_load(args.config.read_text(encoding='utf-8'));args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=CycleDriver('/loader/ground_truth/odometry' if args.pose_source=='ground_truth' else '/loader/localization/odometry')
    node.safety_map=SafetyMap(**config['navigation']);runs=[];transitions=[]
    result={'status':'running','requested_cycles':args.cycles,'pose_source':args.pose_source,
            'terrain_reset':False,'material_replenishment':False,'vehicle_teleport':False,'cycles':runs}
    def save():(args.output/'endurance.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    try:
        connect(node);prepare_pose(node,.35,.25);initial=node.pose();direction='A_to_B'
        while len(runs)<args.cycles:
            selected='A_to_B' if (len(runs)//args.batch_size)%2==0 else 'B_to_A'
            approach=initial if selected=='A_to_B' else Pose(4.,10.,1.5707963267948966)
            if selected!=direction:
                prepare_pose(node,.35,.25)
                drive_leg(node,Pose(-8.,0.,0.),-1,transitions)
                drive_leg(node,approach,1,transitions)
                direction=selected
            task=copy.deepcopy(config);task['return_pose']=[approach.x,approach.y,approach.yaw]
            task['maximum_dig_advance_m']=5.5 if selected=='A_to_B' else 4.
            if selected=='B_to_A':
                task.update(source_center=config['unload_center'],unload_center=config['source_center'],
                            unload_pose=[1.2,0.,0.],staging_pose=[-8.,0.,0.])
            start=time.monotonic();sim_start=node.now()
            report=execute_cycle(node,task,args.output/f'cycle_{len(runs)+1:03d}.json',args.pose_source)
            runs.append({'cycle':len(runs)+1,'direction':selected,'status':report['status'],
                         'loaded_m3':report['loaded_volume_m3'],'unloaded_m3':report['unloaded_volume_m3'],
                         'inside_destination_fraction':report['inside_B_fraction'],'peak_force_n':report['peak_force_n'],
                         'conservation_error':report['relative_conservation_error'],
                         'sim_seconds':node.now()-sim_start,'wall_seconds':time.monotonic()-start})
            result['completed_cycles']=len(runs);save()
        result['status']='passed';print(f'PASS {len(runs)} continuous whole-vehicle material cycles',flush=True)
    except BaseException as error:result.update(status='failed',reason=str(error));raise
    finally:
        for _ in range(5):node.send(emergency=True);rclpy.spin_once(node,timeout_sec=.02)
        result['transition_trace_columns']=TRACE_COLUMNS;result['transition_trace']=transitions;save()
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
