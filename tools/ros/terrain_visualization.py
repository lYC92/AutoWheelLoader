#!/usr/bin/env python3
"""Foxglove/RViz terrain surface and actual payload COM; display only."""
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Point,TransformStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker,MarkerArray
from tf2_ros import TransformBroadcaster
from loader_sim_msgs.msg import TerrainState,BucketInteraction

class Display(Node):
    def __init__(self,source):
        super().__init__('loader_terrain_display',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.terrain=None;self.payload=None;self.last_sequence=-1
        self.create_subscription(TerrainState,'/loader/terrain_state',lambda m:setattr(self,'terrain',m),1)
        self.create_subscription(BucketInteraction,'/loader/bucket_interaction',lambda m:setattr(self,'payload',m),1)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher=self.create_publisher(MarkerArray,'/loader/visualization/terrain',qos)
        self.broadcaster=TransformBroadcaster(self)
        if source=='ground_truth':self.create_subscription(Odometry,'/loader/ground_truth/odometry',self.pose,1)
        self.create_timer(.5,self.publish)

    def pose(self,m):
        if m.header.frame_id!='world' or m.child_frame_id!='base_link':return
        t=TransformStamped();t.header=m.header;t.child_frame_id=m.child_frame_id
        p=m.pose.pose.position;t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.translation.z=p.z
        t.transform.rotation=m.pose.pose.orientation;self.broadcaster.sendTransform(t)

    def publish(self):
        t=self.terrain
        if t is None or t.schema_version!=2:return
        heights=np.asarray(t.height_grid_m).reshape(t.rows,t.columns);rows,cols=np.nonzero(heights>1e-6)
        surface=Marker();surface.header=t.header;surface.ns='soil_surface';surface.id=0;surface.type=Marker.TRIANGLE_LIST
        surface.action=Marker.ADD;surface.pose.orientation.w=1.;surface.scale.x=surface.scale.y=surface.scale.z=1.
        for row,col in zip(rows,cols):
            x=t.domain_min_m+float(col)*t.cell_size_m;y=t.origin_y_m+float(row)*t.cell_size_m;z=float(heights[row,col]);r=t.cell_size_m
            color=ColorRGBA(r=.45+.3*min(z/2,1),g=.27+.25*min(z/2,1),b=.1,a=1.)
            for px,py in [(x,y),(x+r,y),(x+r,y+r),(x,y),(x+r,y+r),(x,y+r)]:
                surface.points.append(Point(x=px,y=py,z=z));surface.colors.append(color)
        markers=[surface]
        if self.payload is not None:
            payload=self.payload;center=Marker();center.header=payload.header;center.header.frame_id='bucket'
            center.ns='payload_center';center.id=0;center.type=Marker.SPHERE;center.action=Marker.ADD if payload.payload_inertia.m>0 else Marker.DELETE
            com=payload.payload_inertia.com
            center.pose.position=Point(x=com.x,y=com.y,z=com.z);center.pose.orientation.w=1.
            center.scale.x=center.scale.y=center.scale.z=.15;center.color=ColorRGBA(r=0.,g=1.,b=1.,a=1.);markers.append(center)
        self.publisher.publish(MarkerArray(markers=markers))

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--pose-source',choices=['ground_truth','estimated'],required=True)
    args=parser.parse_args();rclpy.init();node=Display(args.pose_source)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
