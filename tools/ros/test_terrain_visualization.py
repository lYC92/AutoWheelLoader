#!/usr/bin/env python3
"""Check the actual ROS triangle surface against its terrain ledger stamp."""
import argparse,json,time
from pathlib import Path
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from loader_sim_msgs.msg import TerrainState
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile,DurabilityPolicy

def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    rclpy.init();node=Node('terrain_display_test');terrain=deque(maxlen=30);surfaces=deque(maxlen=5)
    node.create_subscription(TerrainState,'/loader/terrain_state',lambda m:terrain.append(m),1)
    node.create_subscription(MarkerArray,'/loader/visualization/terrain',lambda m:surfaces.append(m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline=time.monotonic()+20;result=None
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if not surfaces:continue
            surface=surfaces[-1].markers[0];matches=[m for m in terrain if stamp(m)==stamp(surface)]
            if not matches:continue
            m=matches[-1];grid=np.asarray(m.height_grid_m);points=np.array([[p.x,p.y,p.z] for p in surface.points])
            assert surface.header.frame_id=='world' and surface.type==surface.TRIANGLE_LIST
            assert len(points)==6*np.count_nonzero(grid>1e-6)
            for offset in (0,3):
                corners=points[offset::6];ix=np.rint((corners[:,0]-m.domain_min_m)/m.cell_size_m).astype(int)
                iy=np.rint((corners[:,1]-m.origin_y_m)/m.cell_size_m).astype(int)
                np.testing.assert_allclose(corners[:,2],grid[iy*m.columns+ix],atol=1e-12)
            result={'status':'passed','stamp_ns':stamp(m),'triangles':len(points)//3,'active_cells':len(points)//6,
                    'surface':'two triangles per active terrain cell, world frame; exact ledger heights'}
            args.output.write_text(json.dumps(result,indent=2));print(json.dumps(result));break
        if result is None:raise RuntimeError('no matching live terrain and display surface')
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
