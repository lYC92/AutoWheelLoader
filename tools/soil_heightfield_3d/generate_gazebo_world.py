#!/usr/bin/env python3
"""Ray-visible XY soil cells matching enable_soil_3d xacro parameters."""
import argparse
import math
import sys
import yaml
import xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"soil_slice"))
from generate_soil_proxy_world import render_world

parser=argparse.ArgumentParser()
parser.add_argument("output", type=Path)
parser.add_argument("--observer-lidar", action="store_true")
parser.add_argument("--config",type=Path)
parser.add_argument("--task-config",type=Path)
parser.add_argument("--wheel-ramp",action="store_true")
parser.add_argument("--overview-camera",action="store_true")
parser.add_argument("--sensor-systems",action="store_true")
parser.add_argument('--physics-hz',type=int,choices=[250,500],default=500)
args=parser.parse_args()
text=render_world(observer_lidar=args.observer_lidar, sensor_systems=args.observer_lidar or args.overview_camera or args.sensor_systems)
text=text.replace('<max_step_size>0.002</max_step_size>',f'<max_step_size>{1/args.physics_hz}</max_step_size>')
text=text.replace('name="loader_500hz"',f'name="loader_{args.physics_hz}hz"')
# The 3D unloading point can lie well beyond the original narrow slice view.
# Cover the whole nominal XY field with the fixed observer.
text=text.replace('<min_angle>-0.50</min_angle><max_angle>0.50</max_angle>',
                  '<min_angle>-1.10</min_angle><max_angle>1.10</max_angle>')
start=text.index('    <model name="soil_column_')
end=text.index('    </model>', text.index('    <model name="soil_column_279"'))+len('    </model>')
config_path=args.config or Path(__file__).resolve().parents[2]/"ros_ws/src/loader_description/config/soil_heightfield.yaml"
config=yaml.safe_load(config_path.read_text(encoding="utf-8"))
r=config["cell_size_m"]
nx=round((config["domain_max_x_m"]-config["domain_min_x_m"])/r)
ny=round((config["domain_max_y_m"]-config["domain_min_y_m"])/r)
if not all(math.isfinite(float(v)) for v in config.values()) or r<=0 or nx<=0 or ny<=0:
    raise ValueError("invalid finite grid configuration")
if not (math.isclose(nx*r,config["domain_max_x_m"]-config["domain_min_x_m"]) and
        math.isclose(ny*r,config["domain_max_y_m"]-config["domain_min_y_m"])):
    raise ValueError("domain extents must be whole multiples of cell size")
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
# Keep landmarks outside the manoeuvring corridor. The coloured outlines are
# paint on the ground, not bins: deposited material remains on the XY terrain.
if config_path.stem == "soil_heightfield_ab":
    text=text.replace('<size>80 40</size>', '<size>80 60</size>')
    yard=[]
    def box(name,x,y,z,sx,sy,sz,color,yaw=0):
        yard.append(f'<visual name="{name}"><pose>{x} {y} {z} 0 0 {yaw}</pose>'
                    f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
                    f'<material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>')
    task=yaml.safe_load((args.task_config or Path(__file__).resolve().parents[2]/"simulation/config/ab_task.yaml").read_text(encoding="utf-8"))
    for label,center,radius,color in [("source",task["source_center"],3.2,"0.85 0.65 0.12"),
                                      ("dump",task["unload_center"],task["unload_radius_m"],"0.15 0.60 0.75")]:
        for i in range(48):
            a=2*math.pi*i/48
            box(f"{label}_{i}",center[0]+radius*math.cos(a),center[1]+radius*math.sin(a),
                0.009,2*math.pi*radius/48,0.07,0.008,color,a+math.pi/2)
    # Low retaining walls at the yard edge provide depth and scale.
    for i in range(12):
        box(f"back_wall_{i}",-16+3*i,27,0.6,2.95,0.5,1.2,"0.48 0.47 0.43")
        box(f"side_wall_{i}",20,-7+3*i,0.6,0.5,2.95,1.2,"0.48 0.47 0.43")
    for i,polygon in enumerate(task.get('navigation',{}).get('obstacles',[])):
        xs,ys=zip(*polygon);xmin,xmax,ymin,ymax=min(xs),max(xs),min(ys),max(ys)
        if len(polygon)!=4 or set(map(tuple,polygon))!={(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax)}:
            raise ValueError('physical obstacle generation currently requires axis-aligned rectangles')
        obstacle_height=float(task.get('obstacle_height_m',1.5))
        box(f'obstacle_{i}',(xmin+xmax)/2,(ymin+ymax)/2,obstacle_height/2,xmax-xmin,ymax-ymin,obstacle_height,'0.8 0.22 0.08')
    yard_text='<model name="yard_landmarks"><static>true</static><link name="link">'+"".join(yard)+'</link></model>'
    # Insert after replacing the soil block so its offsets stay valid.
else:
    yard_text=""
if args.overview_camera:
    yard_text += '''<model name="yard_camera"><static>true</static>
    <pose>-15 -18 23 0 0.68 0.92</pose><link name="link">
    <sensor name="overview" type="camera"><always_on>true</always_on><update_rate>2</update_rate>
    <topic>/loader/yard_camera</topic><camera><horizontal_fov>1.1</horizontal_fov>
    <image><width>960</width><height>720</height><format>R8G8B8</format></image>
    <clip><near>0.1</near><far>100</far></clip></camera></sensor></link></model>'''
args.output.parent.mkdir(parents=True,exist_ok=True)
world_xml=text[:start]+'<model name="soil_grid"><static>true</static><link name="link">'+"\n".join(columns)+'</link></model>'+yard_text+text[end:]
root=ET.fromstring(world_xml)
# Gazebo reliably loads Label at model scope. Keep painted ground separate
# from solid structures, and each obstacle in its own panoptic instance.
landmarks=root.find('./world/model[@name="yard_landmarks"]')
if landmarks is not None:
    painted=ET.SubElement(root.find('world'),'model',name='yard_markings')
    ET.SubElement(painted,'static').text='true';paint_link=ET.SubElement(painted,'link',name='link')
    for visual in list(landmarks.find('link').findall('visual')):
        name=visual.get('name')
        if name.startswith(('source_','dump_')):
            landmarks.find('link').remove(visual);paint_link.append(visual)
        elif name.startswith('obstacle_'):
            obstacle=ET.SubElement(root.find('world'),'model',name=name)
            ET.SubElement(obstacle,'static').text='true';link=ET.SubElement(obstacle,'link',name='link')
            landmarks.find('link').remove(visual);link.append(visual)
for model in root.findall('./world/model'):
    name=model.get('name')
    label=2 if name=='soil_grid' else 1 if name in ('rigid_ground','yard_markings') else 3
    plugin=ET.SubElement(model,'plugin',filename='gz-sim-label-system',name='gz::sim::systems::Label')
    ET.SubElement(plugin,'label').text=str(label)
    for visual in model.findall('./link/visual'):
        vname=visual.get('name')
        # Yard walls now have physical geometry matching their visible outline.
        if (name=='yard_landmarks' and 'wall' in vname) or name.startswith('obstacle_'):
            import copy
            collision=ET.SubElement(model.find('link'),'collision',name=vname)
            collision.append(copy.deepcopy(visual.find('pose')))
            collision.append(copy.deepcopy(visual.find('geometry')))
if args.wheel_ramp:
    ramp=ET.fromstring('<model name="uneven_wheel_track"><static>true</static><pose>0 1.25 0.075 0 0 0</pose><link name="link"><collision name="track"><geometry><box><size>10 0.8 0.15</size></box></geometry></collision><visual name="track"><geometry><box><size>10 0.8 0.15</size></box></geometry><material><diffuse>0.25 0.4 0.5 1</diffuse></material></visual></link></model>')
    root.find('world').append(ramp)
ET.indent(root)
args.output.write_text(ET.tostring(root,encoding='unicode'),encoding='utf-8')
print(f"Generated XY heightfield: {nx} x {ny}, {len(columns)} active visuals, {args.output}")
