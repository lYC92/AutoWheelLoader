#!/usr/bin/env python3
"""Ray-visible XY soil cells matching enable_soil_3d xacro parameters."""
import argparse
import math
import sys
import yaml
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"soil_slice"))
from generate_soil_proxy_world import render_world

parser=argparse.ArgumentParser()
parser.add_argument("output", type=Path)
parser.add_argument("--observer-lidar", action="store_true")
args=parser.parse_args()
text=render_world(observer_lidar=args.observer_lidar, sensor_systems=args.observer_lidar)
start=text.index('    <model name="soil_column_')
end=text.index('    </model>', text.index('    <model name="soil_column_279"'))+len('    </model>')
config_path=Path(__file__).resolve().parents[2]/"ros_ws/src/loader_description/config/soil_heightfield.yaml"
config=yaml.safe_load(config_path.read_text(encoding="utf-8"))
r=config["cell_size_m"]
nx=round((config["domain_max_x_m"]-config["domain_min_x_m"])/r)
ny=round((config["domain_max_y_m"]-config["domain_min_y_m"])/r)
column_height=config["column_height_m"]
columns=[]
for iy in range(ny):
    for ix in range(nx):
        x,y=config["domain_min_x_m"]+(ix+0.5)*r,config["domain_min_y_m"]+(iy+0.5)*r
        h=max(0,config["pile_height_m"]-math.tan(math.radians(config["angle_of_repose_deg"]))*math.hypot(x-config["pile_center_x_m"],y-config["pile_center_y_m"]))
        if h <= 1e-9:
            continue
        columns.append(f'''<visual name="soil_column_{iy*nx+ix:03d}">
<pose>{x} {y} {h-column_height/2} 0 0 0</pose>
<geometry><box><size>{r} {r} {column_height}</size></box></geometry>
<material><ambient>0.58 0.38 0.18 1</ambient><diffuse>0.72 0.50 0.25 1</diffuse></material>
</visual>''')
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(text[:start]+'<model name="soil_grid"><static>true</static><link name="link">'+"\n".join(columns)+'</link></model>'+text[end:],encoding="utf-8")
print(f"Generated XY heightfield: {nx} x {ny}, {len(columns)} active visuals, {args.output}")
