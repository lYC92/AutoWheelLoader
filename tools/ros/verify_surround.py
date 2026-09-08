#!/usr/bin/env python3
"""Independently check recorded camera geometry and lossless MCAP decoding."""
import argparse,json
from collections import Counter
from pathlib import Path
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from capture_surround import decode,NAMES,STREAMS


def verify(dataset,task_config,require_deformation=False):
    manifest=json.loads((dataset/'manifest.json').read_text())
    if manifest['status']!='complete' or not manifest['frames']:raise ValueError('incomplete capture')
    frames={entry['stamp_ns']:entry for entry in manifest['frames']}
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(dataset/'mcap'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    counts=Counter();image_groups={};cache={}
    while reader.has_next():
        topic,payload,stamp=reader.read_next();counts[topic]+=1
        if topic not in types or types[topic].__name__!='Image':continue
        message=deserialize_message(payload,types[topic]);image_stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if stamp!=image_stamp or stamp not in frames:raise ValueError('MCAP image timestamp mismatch')
        if stamp not in cache:cache={stamp:np.load(dataset/frames[stamp]['file'])}
        raw=cache[stamp];name=topic.split('/')[3]
        stream=next(k for k,v in STREAMS.items() if topic.endswith(v));decoded=decode(message)
        if stream=='semantic':
            np.testing.assert_array_equal(decoded[...,2],raw[f'{name}_semantic'])
            np.testing.assert_array_equal(decoded[...,0].astype(np.uint16)+256*decoded[...,1].astype(np.uint16),raw[f'{name}_instance'])
        else:np.testing.assert_array_equal(decoded,raw[f'{name}_{stream}'])
        image_groups.setdefault(stamp,set()).add(topic)
    if len(image_groups)!=len(frames) or any(len(v)!=12 for v in image_groups.values()):raise ValueError('MCAP missing camera streams')
    reports=[];changed_points=0;seen_classes=set();initial_terrain=None;unsafe_drivable=drivable_total=0
    task=yaml.safe_load(task_config.read_text())
    for stamp,entry in frames.items():
        raw=np.load(dataset/entry['file']);cameras=[]
        if initial_terrain is None and 'terrain' in raw:initial_terrain=raw['terrain'].copy()
        if 'world_from_base' not in raw:raise ValueError('geometry validation requires recorded world pose')
        for index,name in enumerate(NAMES):
            depth=raw[f'{name}_depth'];labels=raw[f'{name}_semantic'];K=manifest['calibration'][name]['K']
            seen_classes.update(map(int,np.unique(labels)))
            y,x=np.indices(depth.shape);valid=np.isfinite(depth)&(depth>.1)&(depth<25)
            optical=np.stack([(x-K[2])/K[0],(y-K[5])/K[4],np.ones(depth.shape)],axis=-1)
            points=optical[valid]*depth[valid,None];cls=labels[valid]
            transform=raw['world_from_base']@raw['transforms'][index]
            points=points@transform[:3,:3].T+transform[:3,3]
            ground=np.abs(points[cls==1,2]);soil=[]
            if 'terrain' in raw:
                terrain=raw['terrain'];origin=raw['terrain_origin'];resolution=float(raw['terrain_resolution'])
                soil_points=points[cls==2];indices=np.floor((soil_points[:,:2]-origin)/resolution).astype(int)
                inside=(indices>=1).all(axis=1)&(indices[:,0]<terrain.shape[1]-1)&(indices[:,1]<terrain.shape[0]-1)
                indices=indices[inside];soil_points=soil_points[inside]
                if len(indices):
                    ix,iy=indices.T;h=terrain[iy,ix]
                    # Exclude bucket-carried soil and cell walls: only upward
                    # horizontal terrain surfaces within one grid-cell height.
                    residual=np.abs(soil_points[:,2]-h)
                    soil=residual[(h>.02)&(residual<resolution)]
                    if initial_terrain is not None:
                        changed_points+=int(((np.abs(h-initial_terrain[iy,ix])>.03)&(residual<.1)).sum())
            cameras.append({'camera':name,'ground_points':len(ground),'ground_height_p95_m':float(np.percentile(ground,95)) if len(ground) else None,
                            'soil_surface_points':len(soil),'soil_surface_height_p95_m':float(np.percentile(soil,95)) if len(soil) else None,
                            'classes':list(map(int,np.unique(labels)))})
        reports.append({'frame':entry['file'],'cameras':cameras})
        products=np.load(dataset/'bev'/entry['file']);row,col=np.nonzero(products['bev_labels']==1)
        local=np.column_stack([12-(row+.5)*.1,12-(col+.5)*.1])
        world=raw['world_from_base'];yaw=np.arctan2(world[1,0],world[0,0]);c,s=np.cos(yaw),np.sin(yaw)
        xy=local@np.array([[c,s],[-s,c]])+world[:2,3];unsafe=np.zeros(len(xy),bool)
        if 'terrain' in raw:
            grid=raw['terrain'];ij=np.floor((xy-raw['terrain_origin'])/raw['terrain_resolution']).astype(int)
            inside=(ij>=0).all(axis=1)&(ij[:,0]<grid.shape[1])&(ij[:,1]<grid.shape[0])
            unsafe[inside]|=grid[ij[inside,1],ij[inside,0]]>.1
        for polygon in task.get('navigation',{}).get('obstacles',[]):
            points=np.asarray(polygon);edges=np.roll(points,-1,axis=0)-points
            cross=edges[:,0,None]*(xy[None,:,1]-points[:,1,None])-edges[:,1,None]*(xy[None,:,0]-points[:,0,None])
            unsafe|=(cross>=0).all(axis=0)|(cross<=0).all(axis=0)
        unsafe_drivable+=int(unsafe.sum());drivable_total+=len(xy)
    ground_checks=[c for r in reports for c in r['cameras'] if c['ground_points']>=100]
    passed=bool(ground_checks) and all(c['ground_height_p95_m']<.1 for c in ground_checks) and {1,2,3,4}<=seen_classes
    unsafe_fraction=unsafe_drivable/max(1,drivable_total)
    passed&=drivable_total>0 and unsafe_fraction<.01 and (not require_deformation or changed_points>=10)
    result={'status':'passed' if passed else 'failed','frames':reports,'mcap_lossless_groups':len(image_groups),'topic_counts':dict(counts),
            'ground_tolerance_m':.1,'seen_classes':sorted(seen_classes),'changed_terrain_surface_matches':changed_points,
            'drivable_cells_scored':drivable_total,'drivable_unsafe_fraction':unsafe_fraction,
            'drivable_reference':'independent recorded terrain heights and configured physical obstacle polygons; cell centers',
            'soil_note':'Diagnostic top-surface subset; not a whole-scene recall score.'}
    (dataset/'verification.json').write_text(json.dumps(result,indent=2))
    if not passed:raise RuntimeError('camera geometry tolerance or required scene classes failed')
    print(f'PASS camera geometry and MCAP round trip: {len(image_groups)} exact synchronized groups')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('dataset',type=Path)
    parser.add_argument('--task-config',type=Path,default=Path(__file__).resolve().parents[2]/'simulation/config/ab_task.yaml')
    parser.add_argument('--require-deformation',action='store_true');args=parser.parse_args()
    verify(args.dataset,args.task_config,args.require_deformation)
