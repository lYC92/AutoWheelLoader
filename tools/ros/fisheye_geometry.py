"""Uniform RGB/depth/semantic reprojection and conservative metric BEV.

Optical axes: +X right, +Y down, +Z forward. Depth inputs are optical Z;
fish_depth output is radial range, in metres. Unknown/occluded BEV stays 0.
"""
import math
import numpy as np
from scipy.ndimage import map_coordinates


def rays(width=640,height=480,fov=math.radians(160)):
    focal=(width-1)/fov
    y,x=np.indices((height,width),dtype=float)
    x=(x-(width-1)/2)/focal;y=(y-(height-1)/2)/focal
    theta=np.hypot(x,y)
    scale=np.sinc(theta/np.pi)
    direction=np.stack([x*scale,y*scale,np.cos(theta)],axis=-1)
    return direction,theta<=fov/2


def reproject(rgb,depth,semantic,fov=math.radians(160),output=(640,480),intrinsics=None):
    if depth.shape!=semantic.shape or rgb.shape!=(*depth.shape,3):
        raise ValueError('RGB/depth/semantic dimensions must agree')
    h,w=depth.shape
    if h!=w: raise ValueError('capture contract requires square perspective input')
    direction,valid=rays(*output,fov)
    focal=(w/2)/math.tan(fov/2)
    # Ogre projection pixel centres use width/2 and height/2.
    fx,fy,cx,cy=(focal,focal,w/2-.5,h/2-.5) if intrinsics is None else (intrinsics[0],intrinsics[4],intrinsics[2],intrinsics[5])
    u=fx*direction[...,0]/direction[...,2]+cx
    v=fy*direction[...,1]/direction[...,2]+cy
    valid&=(u>=0)&(u<w-1)&(v>=0)&(v<h-1)
    coords=np.array([v,u])
    color=np.stack([map_coordinates(rgb[...,i],coords,order=1,mode='constant',cval=0) for i in range(3)],axis=-1)
    labels=map_coordinates(semantic,coords,order=0,mode='constant',cval=0)
    # Inverse optical depth is affine on a plane. Interpolate it only when
    # all four source pixels share the same class and a continuous surface;
    # preserve nearest depth across occlusion boundaries.
    z=map_coordinates(depth,coords,order=0,mode='constant',cval=np.nan)
    x0=np.clip(np.floor(u).astype(int),0,w-2);y0=np.clip(np.floor(v).astype(int),0,h-2)
    corners=np.stack([depth[y0,x0],depth[y0,x0+1],depth[y0+1,x0],depth[y0+1,x0+1]])
    classes=np.stack([semantic[y0,x0],semantic[y0,x0+1],semantic[y0+1,x0],semantic[y0+1,x0+1]])
    smooth=(classes==classes[0]).all(axis=0)&np.isfinite(corners).all(axis=0)&(corners>.1).all(axis=0)
    smooth&=(corners.max(axis=0)-corners.min(axis=0))<.05*np.maximum(corners.min(axis=0),.1)
    inverse=np.zeros_like(depth,dtype=float)
    np.divide(1.,depth,out=inverse,where=np.isfinite(depth)&(depth>.1))
    interpolated=map_coordinates(inverse,coords,order=1,mode='constant',cval=0.)
    np.divide(1.,interpolated,out=z,where=smooth&(interpolated>0))
    valid&=np.isfinite(z)&(z>.1)
    distance=np.where(valid,z/direction[...,2],np.nan).astype(np.float32)
    color[~valid]=0;labels[~valid]=0
    return color,distance,labels,valid


def bev(frames,transforms,resolution=.1,extent=12.,ground_label=1,ground_tolerance=.15):
    """Return RGB, class and observed mask in base coordinates (forward up).

    transforms map optical frames to a gravity-aligned base frame. Frame input
    is fish RGB/radial range/labels. Only observed ground cells are drivable;
    any non-ground point in a cell overrides ground (conservative occupancy).
    """
    n=int(round(2*extent/resolution));image=np.zeros((n,n,3),np.uint8)
    labels=np.zeros((n,n),np.uint8);observed=np.zeros((n,n),bool)
    blocked=np.zeros((n,n),bool)
    for (rgb,distance,semantic),transform in zip(frames,transforms):
        if np.asarray(transform).shape!=(4,4): raise ValueError('invalid camera transform')
        direction,valid=rays(distance.shape[1],distance.shape[0])
        valid&=np.isfinite(distance)&(distance>.1)
        xyz=direction[valid]*distance[valid,None]
        xyz=xyz@transform[:3,:3].T+transform[:3,3]
        cls=semantic[valid];color=rgb[valid]
        indices=np.floor((np.column_stack([extent-xyz[:,0],extent-xyz[:,1]]))/resolution).astype(int)
        inside=(indices>=0).all(axis=1)&(indices<n).all(axis=1)&(xyz[:,2]<4)&(xyz[:,2]>-.3)
        iy,ix=indices[inside].T;cls=cls[inside];color=color[inside];xyz=xyz[inside]
        observed[iy,ix]=True;image[iy,ix]=color
        ground=(cls==ground_label)&(np.abs(xyz[:,2])<ground_tolerance)
        labels[iy[ground],ix[ground]]=ground_label
        blocked[iy[~ground],ix[~ground]]=True
    labels[blocked]=2
    return image,labels,observed
