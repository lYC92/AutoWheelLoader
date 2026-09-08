#!/usr/bin/env python3
"""Fixed surveyed-frame map frontend; receives clouds and predicted pose only.

The initial map is built from a stationary scan at the configured spawn pose.
A saved map can be reused. No Gazebo truth, vehicle pose commands or ground
truth geometry are used to estimate motion. Changing material zones are masked
when creating the static map using the known task regions.
"""
import argparse,json,time,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer,TransformListener,TransformException
from loader_sensor_effects.lidar_effects_node import cloud_dtype
from map_localization import ReferenceMap,transform,ground_plane

class MapOdometry(Node):
    def __init__(self,path):
        super().__init__('loader_map_odometry',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.path=path;self.map=ReferenceMap.load(path) if path.exists() else None
        self.pose=np.eye(4);self.pose[:3,3]=self.declare_parameter('initial_position',[0.,0.,.15]).value
        self.pose[:3,:3]=Rotation.from_euler('z',self.declare_parameter('initial_yaw',0.).value).as_matrix()
        self.drop_start=float(self.declare_parameter("drop_start_s",-1.).value)
        self.drop_duration=float(self.declare_parameter("drop_duration_s",0.).value)
        self.first_scan=None
        self.prediction=None;self.last=-1.;self.failures=0
        self.tf=Buffer(node=self);self.listener=TransformListener(self.tf,self)
        self.publisher=self.create_publisher(Odometry,'/loader/localization/lidar_odometry',10)
        self.create_subscription(Odometry,'/loader/localization/odometry',self.predict,qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/loader/localization/points',self.cloud,qos_profile_sensor_data)

    def predict(self,message):
        q=message.pose.pose.orientation;p=message.pose.pose.position
        pose=np.eye(4);pose[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();pose[:3,3]=[p.x,p.y,p.z]
        self.prediction=(message.header.stamp.sec+message.header.stamp.nanosec*1e-9,pose)

    def cloud(self,message):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if self.first_scan is None:self.first_scan=stamp
        if self.drop_start>=0 and self.drop_start<=stamp-self.first_scan<self.drop_start+self.drop_duration:return
        if stamp-self.last<.19:return
        self.last=stamp
        try:tf=self.tf.lookup_transform('base_link',message.header.frame_id,Time.from_msg(message.header.stamp)).transform
        except TransformException:return
        q=tf.rotation;t=tf.translation;mount=np.eye(4);mount[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();mount[:3,3]=[t.x,t.y,t.z]
        structured=np.frombuffer(bytes(message.data),dtype=cloud_dtype(message))
        scan=transform(np.column_stack([structured[k] for k in ('x','y','z')]),mount)
        if self.map is None:
            plane=ground_plane(scan-np.array([0.,0.,3.6]))
            if plane is None:return
            normal,offset=plane;nx,ny,nz=normal
            yaw=Rotation.from_matrix(self.pose[:3,:3]).as_euler('xyz')[2]
            self.pose[:3,:3]=Rotation.from_euler('xyz',[math.atan2(ny,nz),math.atan2(-nx,math.hypot(ny,nz)),yaw]).as_matrix()
            self.pose[2,3]=offset-3.6*nz
            points=transform(scan,self.pose)
            keep=np.ones(len(points),bool)
            for x,y in [(7,0),(4,17.8)]:keep&=np.linalg.norm(points[:,:2]-[x,y],axis=1)>3.5
            try:self.map=ReferenceMap(points[keep],voxel=.2)
            except ValueError:return
            self.map.save(self.path)
            self.get_logger().info(f'Built static map from stationary initialization: {len(self.map.points)} voxels')
        guess=self.prediction[1] if self.prediction and abs(self.prediction[0]-stamp)<.2 else self.pose
        # Material zones are excluded from BOTH mapping and matching. Their
        # changing tops otherwise get paired with distant ground/wall planes.
        world_scan=transform(scan,guess);keep=np.ones(len(scan),bool)
        for x,y in [(7,0),(4,17.8)]:keep&=np.linalg.norm(world_scan[:,:2]-[x,y],axis=1)>3.5
        scan=scan[keep]
        result=self.map.align_planes(scan,guess)
        if result is None:result=self.map.align_planes(scan,self.pose)
        if result is None:
            self.failures+=1
            if self.failures==1:
                np.savez_compressed(self.path.with_name('unmatched_scan.npz'),scan=scan,guess=guess,last_pose=self.pose,stamp=stamp)
            if self.failures>=3:
                candidates=[]
                for angle in (-.2,0,.2):
                    for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),(0,0)]:
                        hint=guess.copy();hint[:3,3]+=[dx,dy,0]
                        hint[:3,:3]=Rotation.from_euler('z',angle).as_matrix()@hint[:3,:3]
                        candidate=self.map.align_planes(scan,hint)
                        if candidate:candidates.append(candidate)
                if candidates:result=min(candidates,key=lambda r:r['rmse_m']+.1*(1-r['coverage']))
            if result is None:
                self.get_logger().warning('Map matching unobservable or inconsistent; no pose published',throttle_duration_sec=2.)
                return
        self.failures=0;self.pose=result['pose']
        with self.path.with_suffix('.jsonl').open('a') as log:
            log.write(json.dumps({'stamp':stamp,'position':self.pose[:3,3].tolist(),
                                 'rmse':result['rmse_m'],'coverage':result['coverage'],'condition':result['condition']})+'\n')
        out=Odometry();out.header=message.header;out.header.frame_id='world';out.child_frame_id='base_link'
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,self.pose[:3,3])
        q=Rotation.from_matrix(self.pose[:3,:3]).as_quat()
        out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,q)
        out.pose.covariance=np.diag([.03**2]*3+[.01**2]*3).ravel().tolist()
        self.publisher.publish(out)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--map',type=Path,required=True)
    args,ros_args=parser.parse_known_args();rclpy.init(args=ros_args);node=MapOdometry(args.map)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
