import json,sys
from pathlib import Path
import numpy as np
from map_localization import *
p=Path(sys.argv[1]);s=np.load(p/'unmatched_scan.npz');m=ReferenceMap.load(p/'reference_map.npz')
scan=voxelize(s['scan'],m.voxel)
if len(scan)>4000:scan=scan[np.linspace(0,len(scan)-1,4000,dtype=int)]
tr=json.load(open(p/'localization/poses.json'))['ground_truth'][-1];pose=np.array(tr[1])
for i in range(6):
 source=transform(scan,pose);d,ids=m.tree.query(source);keep=(d<2.5)&m.planar[ids]
 a=source[keep];b=m.points[ids[keep]];n=m.normals[ids[keep]];res=np.sum(n*(a-b),axis=1)
 w=np.minimum(1.,.08/np.maximum(abs(res),1e-9));J=np.column_stack([n,np.cross(a,n)]);H=J.T@(w[:,None]*J);ev=np.linalg.eigvalsh(H)
 delta=np.linalg.solve(H,-J.T@(w*res));change=np.eye(4);change[:3,:3]=Rotation.from_rotvec(delta[3:]).as_matrix();change[:3,3]=delta[:3];pose=change@pose
 print(i,'count',keep.sum(),'eigen',ev,'ratio',ev[0]/ev[-1],'rmse',np.sqrt(np.mean(np.sort(res**2)[:int(.8*len(res))])),'delta',delta)
print('result',pose)
