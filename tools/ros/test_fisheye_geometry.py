import unittest
import numpy as np
from fisheye_geometry import rays,reproject,bev

class GeometryTests(unittest.TestCase):
    def test_plane_depth_and_labels(self):
        depth=np.full((640,640),5.,np.float32)
        rgb=np.full((640,640,3),128,np.uint8);labels=np.full((640,640),3,np.uint8)
        color,distance,cls,valid=reproject(rgb,depth,labels)
        direction,_=rays()
        np.testing.assert_allclose(distance[valid]*direction[valid,2],5,atol=1e-6)
        self.assertTrue((cls[valid]==3).all());self.assertTrue((color[valid]==128).all())
        self.assertTrue(np.isnan(distance[~valid]).all())

    def test_tilted_plane_inverse_depth(self):
        import math
        y,x=np.indices((640,640),dtype=float);f=320/math.tan(math.radians(80))
        depth=5/(1+.05*(x-319.5)/f+.04*(y-319.5)/f)
        rgb=np.zeros((640,640,3),np.uint8);labels=np.ones((640,640),np.uint8)
        _,distance,_,valid=reproject(rgb,depth,labels)
        direction,_=rays();projected=direction[valid]*distance[valid,None]
        np.testing.assert_allclose(projected@np.array([.05,.04,1.]),5,atol=1e-6)

    def test_ground_bev_and_unknown(self):
        direction,valid=rays()
        # Down-looking camera: optical Z down, optical X base Y, optical Y base X.
        transform=np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,2],[0,0,0,1.]])
        distance=np.where(valid,2/direction[...,2],np.nan)
        rgb=np.full((480,640,3),100,np.uint8);labels=np.ones((480,640),np.uint8)
        _,grid,observed=bev([(rgb,distance,labels)],[transform])
        self.assertTrue((grid[observed]==1).all());self.assertTrue((grid[~observed]==0).all())
        self.assertGreater(observed.sum(),1000)
        labels[:]=4
        _,grid,observed=bev([(rgb,distance,labels)],[transform])
        self.assertTrue((grid[observed]==2).all())

if __name__=='__main__':unittest.main()
