#!/usr/bin/env python3
"""Offline fisheye RGB/range/semantic and metric BEV dataset conversion."""
import argparse,json,time
from pathlib import Path
import numpy as np
from PIL import Image
from fisheye_geometry import reproject,bev


def main():
    parser=argparse.ArgumentParser();parser.add_argument('dataset',type=Path)
    parser.add_argument('--base-height',type=float,default=.15,help='base_link height above flat reference ground, metres')
    args=parser.parse_args();manifest=json.loads((args.dataset/'manifest.json').read_text())
    output=args.dataset/'bev';output.mkdir(exist_ok=True);start=time.perf_counter();reports=[]
    for entry in manifest['frames']:
        raw=np.load(args.dataset/entry['file']);frames=[];transforms=raw['transforms'].copy()
        if 'world_from_base' in raw:
            world=raw['world_from_base'];yaw=np.arctan2(world[1,0],world[0,0]);c,s=np.cos(yaw),np.sin(yaw)
            # Keep vehicle yaw and horizontal origin, remove roll/pitch; z=0
            # remains the physical rigid ground datum used by the terrain.
            local=np.eye(4);local[:3,:3]=[[c,s,0],[-s,c,0],[0,0,1]]
            local[:2,3]=-local[:2,:2]@world[:2,3]
            transforms=np.array([local@world@t for t in transforms])
        else:transforms[:,2,3]+=args.base_height
        stem=Path(entry['file']).stem;panel=[];fields={};boxes={}
        for name in ('front','left','rear','right'):
            classes=raw[f'{name}_semantic'].astype(np.uint32)
            packed=(classes<<16)|raw[f'{name}_instance'].astype(np.uint32) if f'{name}_instance' in raw else classes<<16
            color,distance,packed,valid=reproject(raw[f'{name}_rgb'],raw[f'{name}_depth'],packed,intrinsics=manifest.get('calibration',{}).get(name,{}).get('K'))
            labels=(packed>>16).astype(np.uint8);instances=(packed&65535).astype(np.uint16)
            boxes[name]=[]
            for identifier in np.unique(packed[valid&(labels>1)]):
                y,x=np.nonzero(valid&(packed==identifier))
                if len(x)<4:continue
                boxes[name].append({'class':int(identifier>>16),'instance':int(identifier&65535),
                                   'visible_bbox_xyxy':[int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)],'visible_pixels':len(x)})
            fields[f'{name}_instance']=instances
            frames.append((color,distance,labels));panel.append(color)
            fields.update({f'{name}_rgb':color,f'{name}_range':distance,f'{name}_semantic':labels,f'{name}_valid':valid})
        color,labels,observed=bev(frames,transforms)
        np.savez_compressed(output/f'{stem}.npz',**fields,bev_rgb=color,bev_labels=labels,bev_observed=observed)
        Image.fromarray(np.concatenate([np.concatenate(panel[:2],axis=1),np.concatenate(panel[2:],axis=1)],axis=0)).save(output/f'{stem}_surround.png')
        (output/f'{stem}_boxes.json').write_text(json.dumps(boxes,indent=2))
        Image.fromarray(color).save(output/f'{stem}_bev.png')
        mask=np.zeros((*labels.shape,3),np.uint8);mask[labels==1]=[50,180,80];mask[labels==2]=[220,80,50]
        Image.fromarray(mask).save(output/f'{stem}_drivable.png')
        reports.append({'frame':stem,'observed_cells':int(observed.sum()),'drivable_cells':int((labels==1).sum()),'blocked_cells':int((labels==2).sum())})
    elapsed=time.perf_counter()-start
    (output/'report.json').write_text(json.dumps({'frames':reports,'seconds':elapsed,'groups_per_second':len(reports)/elapsed,
      'depth':'radial range metres','ground_reference':'world z=0, pose roll/pitch removed when recorded; otherwise configured base height','resolution_m':.1,'extent_m':12,
      'drivable_method':'observed ground labels AND ground height, non-ground occupancy overrides; unknown excluded',
      'semantic_source':'Gazebo label ground truth; this validates projection, not a learned classifier'},indent=2))
    print(f'Converted {len(reports)} synchronized groups in {elapsed:.2f}s')

if __name__=='__main__':main()
