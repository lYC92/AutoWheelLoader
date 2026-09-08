#!/usr/bin/env python3
"""Exact-stamp four-camera dataset capture. Never substitutes adjacent frames."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
from scipy.spatial.transform import Rotation

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
    def __init__(self,output,count,interval):
        super().__init__('loader_surround_capture',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.output=output;self.count=count;self.interval=interval;self.frames=[];self.pending={};self.last=-1e20;self.dropped=0
        self.calibration={}
        self.phase='initial';self.create_subscription(String,'/loader/task_state',lambda m:setattr(self,'phase',m.data),10)
        self.tf=Buffer(node=self);self.listener=TransformListener(self.tf,self)
        for name in NAMES:
            self.create_subscription(CameraInfo,f'/loader/cameras/{name}/raw/camera_info',lambda m,n=name:self.info(n,m),qos_profile_sensor_data)
            for stream,suffix in STREAMS.items():
                self.create_subscription(Image,f'/loader/cameras/{name}/{suffix}',lambda m,k=f'{name}_{stream}':self.receive(k,m),qos_profile_sensor_data)
        self.create_timer(.05,self.drain)

    def info(self,name,message):
        self.calibration[name]={'width':message.width,'height':message.height,'K':list(message.k),
          'D':list(message.d),'P':list(message.p),'distortion_model':message.distortion_model,'frame_id':message.header.frame_id}

    def receive(self,key,message):
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if stamp/1e9-self.last<self.interval: return
        self.pending.setdefault(stamp,{})[key]=message
        while len(self.pending)>10:
            del self.pending[min(self.pending)];self.dropped+=1

    def drain(self):
        for stamp in sorted(list(self.pending)):
            group=self.pending[stamp]
            if len(group)!=12 or len(self.calibration)!=4 or stamp/1e9-self.last<self.interval: continue
            try:
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
                if data[key].ndim==3: data[key]=data[key][...,0]
            filename=f'frame_{len(self.frames):04d}.npz'
            np.savez_compressed(self.output/filename,**data,transforms=np.array(transforms),stamp_ns=stamp)
            self.frames.append({'file':filename,'stamp_ns':stamp,'phase':self.phase,'streams':12,'max_skew_ns':0})
            self.last=stamp/1e9
            self.pending={k:v for k,v in self.pending.items() if k>stamp}
            self.save();print(f'CAPTURE {filename}: 12 matched streams at {self.last:.3f}s',flush=True)
            break

    def save(self):
        (self.output/'manifest.json').write_text(json.dumps({'schema_version':1,'status':'complete' if len(self.frames)>=self.count else 'recording',
          'projection':'perspective input; unified equidistant offline','input_size':[640,640],'fish_size':[640,480],
          'horizontal_fov_deg':160,'nominal_hz':10,'sample_interval_s':self.interval,'depth_convention':'optical Z metres',
          'optical_axes':'+X right, +Y down, +Z forward','transform':'T_base_link_camera_optical at image timestamp',
          'calibration':self.calibration,
          'labels':{'0':'unknown','1':'rigid_ground','2':'soil','3':'obstacle','4':'vehicle'},
          'dropped_incomplete_groups':self.dropped,'frames':self.frames},indent=2),encoding='utf-8')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--frames',type=int,default=5);parser.add_argument('--interval',type=float,default=1.)
    parser.add_argument('--wall-timeout',type=float,default=240.)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Capture(args.output,args.frames,args.interval);deadline=time.monotonic()+args.wall_timeout
    try:
        while len(node.frames)<args.frames and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if len(node.frames)<args.frames:raise RuntimeError(f'exact camera synchronization timeout: {len(node.frames)}/{args.frames} groups')
    finally:node.save();node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
