"""Voxel map persistence and robust 6-DOF scan-to-map relocalization."""
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from localization_geometry import ground_plane


def voxelize(points,size=.25):
    points=np.asarray(points,dtype=float)
    points=points[np.isfinite(points).all(axis=1)]
    if not np.isfinite(size) or size<=0:raise ValueError('invalid voxel size')
    _,indices=np.unique(np.floor(points/size).astype(np.int64),axis=0,return_index=True)
    return points[np.sort(indices)]


def transform(points,pose):return points@pose[:3,:3].T+pose[:3,3]

class ReferenceMap:
    def __init__(self,points,voxel=.25):
        self.voxel=voxel;self.points=voxelize(points,voxel)
        if len(self.points)<100:raise ValueError('map needs at least 100 finite voxels')
        self.tree=cKDTree(self.points)
        _,indices=self.tree.query(self.points,k=min(32,len(self.points)))
        neighbors=self.points[indices];centered=neighbors-neighbors.mean(axis=1,keepdims=True)
        covariance=np.einsum('nki,nkj->nij',centered,centered)
        values,vectors=np.linalg.eigh(covariance)
        self.normals=vectors[:,:,0]
        self.planar=(values[:,0]/np.maximum(values[:,1],1e-9)<.1)&(values[:,1]/np.maximum(values[:,2],1e-9)>.1)
        # Structured lidar rings can make local floor neighborhoods nearly
        # collinear. Their noisy normals create fictitious XY information.
        # Fit the broad surveyed yard plane from measured map points and use
        # its common normal, retaining wall/pillar normals for XY and yaw.
        plane=ground_plane(self.points-np.array([0.,0.,3.6]))
        if plane is not None:
            normal,offset=plane;offset-=3.6*normal[2]
            residual=self.points@normal+offset;ground=np.abs(residual)<.18
            if ground.sum()>300:
                self.points[ground]-=residual[ground,None]*normal
                self.normals[ground]=normal;self.planar[ground]=True
                self.tree=cKDTree(self.points)

    def save(self,path):
        path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(path,points=self.points,voxel=self.voxel,frame='world',schema_version=1)

    @classmethod
    def load(cls,path):
        with np.load(path,allow_pickle=False) as data:
            if int(data['schema_version'])!=1 or str(data['frame'])!='world':raise ValueError('map schema/frame mismatch')
            return cls(data['points'],float(data['voxel']))

    def align(self,scan,initial,max_distance=1.5,iterations=35):
        scan=voxelize(scan,self.voxel)
        if len(scan)<100:return None
        pose=np.asarray(initial,dtype=float).copy()
        if pose.shape!=(4,4) or not np.isfinite(pose).all():raise ValueError('invalid initial map pose')
        for iteration in range(iterations):
            source=transform(scan,pose);distance,index=self.tree.query(source,workers=1)
            keep=distance<max_distance
            if keep.sum()<100:return None
            cutoff=min(max_distance,max(.1,float(np.quantile(distance[keep],.8))))
            keep&=distance<=cutoff
            a=source[keep];b=self.points[index[keep]]
            ca=a.mean(axis=0);cb=b.mean(axis=0)
            U,_,Vt=np.linalg.svd((a-ca).T@(b-cb));R=Vt.T@U.T
            if np.linalg.det(R)<0:Vt[-1]*=-1;R=Vt.T@U.T
            delta=np.eye(4);delta[:3,:3]=R;delta[:3,3]=cb-R@ca;pose=delta@pose
            if np.linalg.norm(delta[:3,3])<1e-4 and Rotation.from_matrix(R).magnitude()<1e-4:break
        distance,index=self.tree.query(transform(scan,pose),workers=1)
        keep=distance<.35;coverage=float(keep.mean())
        if keep.sum()<100:return None
        rmse=float(np.sqrt(np.mean(distance[keep]**2)))
        # A compact or nearly collinear match cannot constrain a full pose.
        eigen=np.linalg.eigvalsh(np.cov(self.points[index[keep]].T))
        if coverage<.55 or rmse>.18 or eigen[1]<.2:return None
        return {'pose':pose,'rmse_m':rmse,'coverage':coverage,'iterations':iteration+1}

    def align_planes(self,scan,initial,max_distance=2.5,iterations=30):
        scan=voxelize(scan,self.voxel)
        if len(scan)>4000:scan=scan[np.linspace(0,len(scan)-1,4000,dtype=int)]
        pose=np.asarray(initial,dtype=float).copy()
        for iteration in range(iterations):
            source=transform(scan,pose);distance,index=self.tree.query(source,workers=1)
            keep=(distance<max_distance)&self.planar[index]
            if keep.sum()<100:return None
            a=source[keep];b=self.points[index[keep]];normal=self.normals[index[keep]]
            residual=np.sum(normal*(a-b),axis=1)
            weights=np.minimum(1.,.08/np.maximum(np.abs(residual),1e-9))
            pivot=pose[:3,3].copy()
            J=np.column_stack([normal,np.cross(a-pivot,normal)])
            H=J.T@(weights[:,None]*J)
            scale=1/np.sqrt(np.maximum(np.diag(H),1e-12))
            eigen=np.linalg.eigvalsh(scale[:,None]*H*scale[None,:])
            # Metres and radians have different units. Observability is tested
            # on a dimensionless correlation matrix, not raw normal equations.
            if eigen[0]<1e-4*eigen[-1]:return None
            delta=np.linalg.solve(H,-J.T@(weights*residual))
            update=np.eye(4);update[:3,:3]=Rotation.from_rotvec(delta[3:]).as_matrix();update[:3,3]=pivot+delta[:3]-update[:3,:3]@pivot
            pose=update@pose
            if np.linalg.norm(delta)<1e-4:break
        source=transform(scan,pose);distance,index=self.tree.query(source,workers=1)
        keep=(distance<max_distance)&self.planar[index]
        if keep.sum()<100:return None
        residual=np.sum(self.normals[index[keep]]*(source[keep]-self.points[index[keep]]),axis=1)
        ordered=np.sort(residual**2)
        rmse=float(np.sqrt(np.mean(ordered[:max(1,int(.8*len(ordered)))])));coverage=float(keep.mean())
        if rmse>.12 or coverage<.4:return None
        return {'pose':pose,'rmse_m':rmse,'coverage':coverage,'iterations':iteration+1,'condition':float(eigen[-1]/eigen[0])}

    def relocalize(self,scan,hint,translation_radius=1.,yaw_radius=np.deg2rad(20)):
        """Bounded prior search; reject absent evidence, never return the hint."""
        candidates=[]
        for dx in (-translation_radius,0,translation_radius):
            for dy in (-translation_radius,0,translation_radius):
                for yaw in (-yaw_radius,0,yaw_radius):
                    guess=np.asarray(hint).copy();guess[:3,3]+=[dx,dy,0]
                    guess[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()@guess[:3,:3]
                    result=self.align(scan,guess)
                    if result:candidates.append(result)
        if not candidates:return None
        return min(candidates,key=lambda r:r['rmse_m']+.2*(1-r['coverage']))
