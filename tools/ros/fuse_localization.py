#!/usr/bin/env python3
"""LiDAR/IMU pose fusion; no ground-truth subscription or fallback."""
import json
import math
import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from loader_sim_msgs.msg import VehicleState
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from inertial_fusion import InertialFilter


def stamp(message): return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


class FusionNode(Node):
    def __init__(self):
        super().__init__('loader_lidar_imu_fusion',parameter_overrides=[Parameter('use_sim_time',value=True)])
        initial=self.declare_parameter('initial_position',[0.,0.,0.15]).value
        yaw=self.declare_parameter('initial_yaw',0.).value
        self.surveyed_ground=self.declare_parameter("surveyed_ground",True).value
        self.filter=InertialFilter(initial,yaw)
        self.origin=np.asarray(initial);self.rotation=Rotation.from_euler('z',yaw)
        self.imu_offset=np.asarray(self.declare_parameter('imu_offset',[-.2,0.,1.4]).value)
        self.last_imu_wall=self.last_lidar_wall=0.;self.last_publish=-1.
        self.stationary=False;self.recovery=[];self.relocalizations=0
        self.create_subscription(VehicleState,"/loader/state",self.vehicle,1)
        self.previous_gyro=None;self.previous_time=None;self.alpha=np.zeros(3)
        self.create_subscription(String,'/loader/localization/ground_constraint',self.ground,10)
        self.create_subscription(Imu,'/loader/sensors/imu',self.imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/loader/localization/lidar_odometry',self.lidar,qos_profile_sensor_data)
        self.publisher=self.create_publisher(Odometry,'/loader/localization/odometry',10)
        self.broadcaster=TransformBroadcaster(self)
        self.health=self.create_publisher(String,'/loader/localization/health',10)
        self.create_timer(.05,self.report)

    def vehicle(self,message):
        self.stationary=(abs(message.longitudinal_speed_mps)<.02 and
                         max(map(abs,message.joint_state.velocity),default=1)<.02)

    def imu(self,message):
        t=stamp(message);w=np.array([message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z])
        a=np.array([message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z])
        if self.previous_time is not None and 0<t-self.previous_time<=.1:
            self.alpha=.8*self.alpha+.2*(w-self.previous_gyro)/(t-self.previous_time)
        a-=np.cross(self.alpha,self.imu_offset)+np.cross(w,np.cross(w,self.imu_offset))
        self.previous_time,self.previous_gyro=t,w
        try: self.filter.predict(t,a,w)
        except ValueError as error:
            self.get_logger().warning(str(error),throttle_duration_sec=2.)
            if self.filter.time is not None and t>self.filter.time:
                self.filter.time=t;self.filter.history.clear();self.filter.accepted=0
                self.filter.P+=np.diag([.1**2]*3+[.2**2]*3+[.03**2]*3+[0.]*6)
            return
        self.last_imu_wall=time.monotonic()
        if t-self.last_publish<.01-1e-9: return
        self.last_publish=t
        f=self.filter
        out=Odometry();out.header=message.header;out.header.frame_id='world';out.child_frame_id='base_link'
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,f.p)
        q=f.q.as_quat()
        out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,q)
        ids=[0,1,2,6,7,8];out.pose.covariance=f.P[np.ix_(ids,ids)].ravel().tolist()
        velocity=f.q.inv().apply(f.v)
        out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,velocity)
        out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=map(float,w-f.bg)
        self.publisher.publish(out)
        tf=TransformStamped();tf.header=out.header;tf.child_frame_id=out.child_frame_id
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,f.p)
        tf.transform.rotation=out.pose.pose.orientation;self.broadcaster.sendTransform(tf)

    def lidar(self,message):
        if message.header.frame_id not in ('odom','world') or message.child_frame_id!='base_link':
            self.get_logger().error('LiDAR frame contract mismatch');return
        p=message.pose.pose.position;q=message.pose.pose.orientation
        position=self.origin+self.rotation.apply([p.x,p.y,p.z])
        orientation=(self.rotation*Rotation.from_quat([q.x,q.y,q.z,q.w])).as_quat()
        if message.header.frame_id=='world':
            position=np.array([p.x,p.y,p.z]);orientation=np.array([q.x,q.y,q.z,q.w])
        if (message.header.frame_id=='world' and self.stationary and self.filter.time is not None
                and self.filter.time-self.filter.last_lidar>.5 and np.linalg.norm(position-self.filter.p)>.15):
            self.recovery.append((stamp(message),position.copy(),orientation.copy()))
            self.recovery=self.recovery[-3:]
            if (len(self.recovery)==3 and self.recovery[-1][0]-self.recovery[0][0]<.7
                    and max(np.linalg.norm(p-position) for _,p,_ in self.recovery)<.05
                    and max((Rotation.from_quat(q).inv()*Rotation.from_quat(orientation)).magnitude() for _,_,q in self.recovery)<.01):
                current_time=self.filter.time
                self.filter=InertialFilter(position);self.filter.q=Rotation.from_quat(orientation);self.filter.time=current_time
                self.recovery=[];self.relocalizations+=1
                self.get_logger().info('Relocalized while stopped from three consistent map observations')
        else:self.recovery=[]
        if self.filter.correct(stamp(message),position,orientation,
                               indices=(0,1,5) if self.surveyed_ground and message.header.frame_id=='odom' else None): self.last_lidar_wall=time.monotonic()

    def ground(self,message):
        if not self.surveyed_ground:return
        try:
            data=json.loads(message.data);nx,ny,nz=data['normal'];height=data['offset']
            if nz<.8 or not .02<height<1.:return
            yaw=self.filter.q.as_euler('xyz')[2]
            q=Rotation.from_euler('xyz',[math.atan2(ny,nz),math.atan2(-nx,math.hypot(ny,nz)),yaw])
            position=self.filter.p.copy();position[2]=height
            self.filter.correct(data['stamp'],position,q.as_quat(),indices=(2,3,4),
                                variances=[.03**2]*3+[.005**2]*3,channel='ground')
        except (ValueError,KeyError,TypeError):return

    def report(self):
        now=self.get_clock().now().nanoseconds/1e9
        if now-self.filter.last_lidar>.5:self.filter.accepted=0
        tracking=(self.filter.healthy(now) and time.monotonic()-self.last_imu_wall<.3
                  and time.monotonic()-self.last_lidar_wall<1.)
        message=String();message.data=json.dumps({'status':'tracking' if tracking else 'unavailable',
            'sim_time':now,'lidar_age_s':float(now-self.filter.last_lidar) if np.isfinite(self.filter.last_lidar) else None,
            'accepted':self.filter.accepted,'rejected':self.filter.rejected,'relocalizations':self.relocalizations,
            'position_variance':float(np.max(np.diag(self.filter.P)[:3]))})
        self.health.publish(message)


def main():
    rclpy.init();node=FusionNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
