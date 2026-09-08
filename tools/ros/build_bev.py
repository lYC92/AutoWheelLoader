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
        raw=np.load(args.dataset/entry['file']);frames=[];transforms=raw['transforms'].copy();transforms[:,2,3]+=args.base_height
        stem=Path(entry['file']).stem;panel=[];fields={}
        for name in ('front','left','rear','right'):
            color,distance,labels,valid=reproject(raw[f'{name}_rgb'],raw[f'{name}_depth'],raw[f'{name}_semantic'],intrinsics=manifest.get('calibration',{}).get(name,{}).get('K'))
            frames.append((color,distance,labels));panel.append(color)
            fields.update({f'{name}_rgb':color,f'{name}_range':distance,f'{name}_semantic':labels,f'{name}_valid':valid})
        color,labels,observed=bev(frames,transforms)
        np.savez_compressed(output/f'{stem}.npz',**fields,bev_rgb=color,bev_labels=labels,bev_observed=observed)
        Image.fromarray(np.concatenate([np.concatenate(panel[:2],axis=1),np.concatenate(panel[2:],axis=1)],axis=0)).save(output/f'{stem}_surround.png')
        Image.fromarray(color).save(output/f'{stem}_bev.png')
        mask=np.zeros((*labels.shape,3),np.uint8);mask[labels==1]=[50,180,80];mask[labels==2]=[220,80,50]
        Image.fromarray(mask).save(output/f'{stem}_drivable.png')
        reports.append({'frame':stem,'observed_cells':int(observed.sum()),'drivable_cells':int((labels==1).sum()),'blocked_cells':int((labels==2).sum())})
    elapsed=time.perf_counter()-start
    (output/'report.json').write_text(json.dumps({'frames':reports,'seconds':elapsed,'groups_per_second':len(reports)/elapsed,
      'depth':'radial range metres','ground_reference':'flat, base height configured','resolution_m':.1,'extent_m':12,
      'drivable_method':'observed ground labels AND ground height, non-ground occupancy overrides; unknown excluded',
      'semantic_source':'Gazebo label ground truth; this validates projection, not a learned classifier'},indent=2))
    print(f'Converted {len(reports)} synchronized groups in {elapsed:.2f}s')

if __name__=='__main__':main()
