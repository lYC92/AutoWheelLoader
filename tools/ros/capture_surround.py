#!/usr/bin/env python3
"""Exact-stamp four-camera dataset capture. Never substitutes adjacent frames."""
import argparse
import json
from pathlib import Path
import time
import signal
from collections import deque
import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import serialize_message
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.clock import Clock as RosClock,ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
from scipy.spatial.transform import Rotation,Slerp
from nav_msgs.msg import Odometry
from loader_sim_msgs.msg import TerrainState
from ros_gz_interfaces.srv import ControlWorld

NAMES=('front','left','rear','right')
STREAMS={'rgb':'raw/image','depth':'raw/depth_image','semantic':'semantic/labels_map'}

def decode(message):
    if message.encoding in ('rgb8','bgr8'):
        array=np.frombuffer(message.data,np.uint8).reshape(message.height,message.step)[:,:message.width*3].reshape(message.height,message.width,3)
        return array[...,::-1].copy() if message.encoding=='bgr8' else array.copy()
    if message.encoding in ('mono8','8UC1'):
        return np.frombuffer(message.data,np.uint8).reshape(message.height,message.step)[:,:message.width].copy()
    if message.encoding=='32FC1':
        dtype='>f4' if message.is_bigendian else '<f4'
        return np.ndarray((message.height,message.width),dtype=dtype,buffer=bytes(message.data),strides=(message.step,4)).astype(np.float32)
    raise ValueError(f'unsupported image encoding {message.encoding}')

class Capture(Node):
    def __init__(self,output,count,interval,world_pose_source,lockstep=False):
        super().__init__('loader_surround_capture',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.world_pose_source=world_pose_source;self.world_poses=deque(maxlen=300);self.terrain_history=deque(maxlen=100)
        self.lockstep=lockstep;self.capture_enabled=not lockstep;self.minimum_stamp=0
        self.last_step_wall=0.
        self.world_control=self.create_client(ControlWorld,'/world/loader_soil_slice/control') if lockstep else None
        if world_pose_source!='none':
            topic='/loader/ground_truth/odometry' if world_pose_source=='ground_truth' else '/loader/localization/odometry'
            self.create_subscription(Odometry,topic,self.pose,qos_profile_sensor_data)
        self.create_subscription(TerrainState,'/loader/terrain_state',lambda m:self.terrain_history.append(m),5)
        self.finished=False
        self.output=output;self.count=count;self.interval=interval;self.frames=[];self.pending={};self.last=-1e20;self.dropped=0
        storage_config=output/'mcap_storage.yaml';storage_config.write_text('compression: Zstd\ncompressionLevel: Fast\n')
        self.writer=rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(output/'mcap'),storage_id='mcap',storage_config_uri=str(storage_config)),rosbag2_py.ConverterOptions('',''))
        self.topics=set();self.info_messages={}
        self.calibration={}
        self.phase='initial';self.create_subscription(String,'/loader/task_state',lambda m:setattr(self,'phase',m.data),10)
        self.tf=Buffer(node=self);self.listener=TransformListener(self.tf,self)
        for name in NAMES:
            self.create_subscription(CameraInfo,f'/loader/cameras/{name}/raw/camera_info',lambda m,n=name:self.info(n,m),qos_profile_sensor_data)
            for stream,suffix in STREAMS.items():
                self.create_subscription(Image,f'/loader/cameras/{name}/{suffix}',lambda m,k=f'{name}_{stream}':self.receive(k,m),qos_profile_sensor_data)
        self.create_timer(.05,self.drain,clock=RosClock(clock_type=ClockType.STEADY_TIME))

    def pose(self,message):
        if message.header.frame_id!='world' or message.child_frame_id!='base_link':return
        p=message.pose.pose.position;q=message.pose.pose.orientation;matrix=np.eye(4)
        matrix[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();matrix[:3,3]=[p.x,p.y,p.z]
        self.world_poses.append((message.header.stamp.sec+message.header.stamp.nanosec*1e-9,matrix))

    def world_pose(self,stamp):
        samples=list(self.world_poses);t=stamp*1e-9
        for (a,p),(b,q) in zip(samples,samples[1:]):
            if a<=t<=b and 0<b-a<.1:
                matrix=np.eye(4);matrix[:3,3]=p[:3,3]+(q[:3,3]-p[:3,3])*(t-a)/(b-a)
                matrix[:3,:3]=Slerp([a,b],Rotation.from_matrix([p[:3,:3],q[:3,:3]]))(t).as_matrix()
                return matrix
        return None

    def info(self,name,message):
        self.info_messages[name]=message
        self.calibration[name]={'width':message.width,'height':message.height,'K':list(message.k),
          'D':list(message.d),'P':list(message.p),'distortion_model':message.distortion_model,'frame_id':message.header.frame_id}

    def record(self,topic,message,stamp):
        kind=message.__class__.__module__.split('.')[0]+'/msg/'+message.__class__.__name__
        if topic not in self.topics:
            self.writer.create_topic(rosbag2_py.TopicMetadata(id=len(self.topics),name=topic,type=kind,serialization_format='cdr'))
            self.topics.add(topic)
        self.writer.write(topic,serialize_message(message),stamp)

    def receive(self,key,message):
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if stamp<self.minimum_stamp or stamp/1e9-self.last<self.interval-1e-6: return
        self.pending.setdefault(stamp,{})[key]=message
        while len(self.pending)>10:
            del self.pending[min(self.pending)];self.dropped+=1

    def drain(self):
        if not self.capture_enabled:return
        # Sensor rendering and 50/100 Hz TF/odometry publication complete on
        # different iterations. A bounded extra 2 ms step provides the next
        # interpolation endpoint instead of substituting an older pose.
        if self.lockstep and time.monotonic()-self.last_step_wall>.5:
            request=ControlWorld.Request();request.world_control.pause=True;request.world_control.multi_step=1
            self.step_future=self.world_control.call_async(request);self.last_step_wall=time.monotonic()
        for stamp in sorted(list(self.pending)):
            group=self.pending[stamp]
            if len(group)!=12 or len(self.calibration)!=4 or stamp/1e9-self.last<self.interval-1e-6: continue
            world=self.world_pose(stamp) if self.world_pose_source!='none' else None
            if self.world_pose_source!='none' and world is None:continue
            terrain=[m for m in self.terrain_history if 0<=stamp-(m.header.stamp.sec*10**9+m.header.stamp.nanosec)<=150_000_000]
            if not terrain:continue
            try:
                bucket_tf=self.tf.lookup_transform('base_link','bucket',Time(nanoseconds=stamp)).transform
                q=bucket_tf.rotation;t=bucket_tf.translation;bucket=np.eye(4)
                bucket[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();bucket[:3,3]=[t.x,t.y,t.z]
                transforms=[]
                for name in NAMES:
                    tf=self.tf.lookup_transform('base_link',f'camera_{name}_link',Time(nanoseconds=stamp)).transform
                    q=tf.rotation;t=tf.translation;matrix=np.eye(4)
                    matrix[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()@np.array([[0,0,1],[-1,0,0],[0,-1,0]])
                    matrix[:3,3]=[t.x,t.y,t.z];transforms.append(matrix)
            except TransformException: continue
            data={key:decode(value) for key,value in group.items()}
            for name in NAMES:
                key=f'{name}_semantic'
                if data[key].ndim!=3:raise ValueError('panoptic labels must have three bytes per pixel')
                packed=data[key];data[f'{name}_instance']=packed[...,0].astype(np.uint16)+256*packed[...,1].astype(np.uint16)
                data[key]=packed[...,2].copy()
            for name in NAMES:
                for stream,suffix in STREAMS.items():self.record(f'/loader/cameras/{name}/{suffix}',group[f'{name}_{stream}'],stamp)
                self.record(f'/loader/cameras/{name}/raw/camera_info',self.info_messages[name],stamp)
            tf_group=TFMessage()
            frame_transforms=[('base_link',f'camera_{name}_optical',matrix) for name,matrix in zip(NAMES,transforms)]
            frame_transforms.append(('base_link','bucket',bucket))
            if world is not None:frame_transforms.append(('world','base_link',world))
            for parent,child,matrix in frame_transforms:
                tf=TransformStamped();tf.header.stamp=Time(nanoseconds=stamp).to_msg();tf.header.frame_id=parent;tf.child_frame_id=child
                tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,matrix[:3,3])
                q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
                tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q)
                tf_group.transforms.append(tf)
            self.record('/tf',tf_group,stamp);self.record('/clock',Clock(clock=Time(nanoseconds=stamp).to_msg()),stamp)
            extras={'base_from_bucket':bucket}
            if world is not None:extras['world_from_base']=world
            if terrain:
                m=terrain[-1]
                extras.update(terrain=np.asarray(m.height_grid_m).reshape(m.rows,m.columns),
                    terrain_origin=[m.domain_min_m,m.origin_y_m],terrain_resolution=m.cell_size_m,
                    terrain_stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec)
                self.record('/loader/terrain_state',m,stamp)
            filename=f'frame_{len(self.frames):04d}.npz'
            # MCAP is already compressed. Avoid synchronous DEFLATE of large
            # convenience arrays starving the exact-stamp image callbacks.
            np.savez(self.output/filename,**data,transforms=np.array(transforms),stamp_ns=stamp,**extras)
            self.frames.append({'file':filename,'stamp_ns':stamp,'phase':self.phase,'streams':12,'max_skew_ns':0})
            self.last=stamp/1e9
            self.pending={k:v for k,v in self.pending.items() if k>stamp}
            self.save();print(f'CAPTURE {filename}: 12 matched streams at {self.last:.3f}s',flush=True)
            if self.lockstep and (self.count==0 or len(self.frames)<self.count):self.step()
            break

    def step(self):
        remaining=50 if self.last<0 else max(1,min(50,round((self.last+.1-self.get_clock().now().nanoseconds/1e9)/.002)))
        request=ControlWorld.Request();request.world_control.pause=True;request.world_control.multi_step=remaining
        self.step_future=self.world_control.call_async(request)
        self.last_step_wall=time.monotonic()

    def save(self):
        gaps=np.diff([f['stamp_ns'] for f in self.frames])*1e-9
        (self.output/'manifest.json').write_text(json.dumps({'schema_version':1,'status':'complete' if self.finished or (self.count>0 and len(self.frames)>=self.count) else 'recording',
          'recording':'MCAP CDR plus NPZ convenience arrays','label_encoding':'Gazebo panoptic: byte 2 class, bytes 1:0 instance (frame-local)',
          'projection':'perspective input; unified equidistant offline','input_size':[640,640],'fish_size':[640,480],
          'horizontal_fov_deg':160,'nominal_hz':10,'sample_interval_s':self.interval,'depth_convention':'optical Z metres',
          'lockstep':self.lockstep,
          'recorded_sim_hz':float(1/np.mean(gaps)) if len(gaps) else None,
          'maximum_frame_gap_s':float(max(gaps)) if len(gaps) else None,
          'consecutive_10hz':bool(len(gaps) and np.all(np.abs(gaps-.1)<=.0021)),
          'optical_axes':'+X right, +Y down, +Z forward','transform':'T_base_link_camera_optical at image timestamp',
          'calibration':self.calibration,'world_pose_source':self.world_pose_source,
          'labels':{'0':'unknown','1':'rigid_ground','2':'soil','3':'obstacle','4':'vehicle'},
          'dropped_incomplete_groups':self.dropped,'frames':self.frames},indent=2),encoding='utf-8')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--frames',type=int,default=5);parser.add_argument('--interval',type=float,default=1.)
    parser.add_argument('--wall-timeout',type=float,default=240.)
    parser.add_argument("--world-pose-source",choices=["none","ground_truth","estimated"],default="none")
    parser.add_argument('--lockstep',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if args.lockstep:args.interval=.1
    rclpy.init();node=Capture(args.output,args.frames,args.interval,args.world_pose_source,args.lockstep);deadline=time.monotonic()+args.wall_timeout
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        if args.lockstep:
            if not node.world_control.wait_for_service(timeout_sec=10):raise RuntimeError('missing world control bridge')
            request=ControlWorld.Request();request.world_control.pause=True;future=node.world_control.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=5)
            if not future.done() or not future.result().success:raise RuntimeError('cannot pause for lockstep capture')
            node.minimum_stamp=node.get_clock().now().nanoseconds;node.pending.clear();node.capture_enabled=True;node.step()
        while running and (args.frames==0 or len(node.frames)<args.frames) and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        node.finished=bool(node.frames) and ((args.frames>0 and len(node.frames)>=args.frames) or (args.frames==0 and not running))
        if not node.finished:raise RuntimeError(f'exact camera synchronization timeout: {len(node.frames)}/{args.frames} groups')
    finally:
        node.capture_enabled=False;node.save();node.writer.close()
        if args.lockstep and node.world_control.service_is_ready():
            request=ControlWorld.Request();request.world_control.pause=False
            rclpy.spin_until_future_complete(node,node.world_control.call_async(request),timeout_sec=3)
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
