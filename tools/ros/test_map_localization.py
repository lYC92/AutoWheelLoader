import tempfile,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from map_localization import ReferenceMap,transform

class MapTests(unittest.TestCase):
    def test_three_planes_constrain_pose(self):
        rng=np.random.default_rng(123)
        xy=rng.uniform([-12,-10],[15,20],(6000,2))
        points=np.vstack([np.column_stack([xy,np.zeros(len(xy))]),
            np.column_stack([np.full(2000,15),rng.uniform(-10,20,2000),rng.uniform(0,4,2000)]),
            np.column_stack([rng.uniform(-12,15,2000),np.full(2000,20),rng.uniform(0,4,2000)])])
        model=ReferenceMap(points)
        truth=np.eye(4);truth[:3,:3]=Rotation.from_euler('xyz',[.02,-.03,.4]).as_matrix();truth[:3,3]=[2,4,.15]
        scan=transform(points,np.linalg.inv(truth))+rng.normal(0,.01,points.shape)
        hint=truth.copy();hint[:3,3]+=[.4,-.5,.1]
        result=model.align_planes(scan,hint)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(result['pose'][:3,3]-truth[:3,3]),.02)
        self.assertIsNone(ReferenceMap(points[:6000]).align_planes(scan[:6000],hint))

    def test_saved_map_recovery_and_no_overlap(self):
        rng=np.random.default_rng(2001)
        # Asymmetric static structures; map and scan have independent noise.
        points=rng.uniform([-8,-6,0],[10,12,5],(2500,3))
        model=ReferenceMap(points)
        truth=np.eye(4);truth[:3,:3]=Rotation.from_euler('xyz',[.03,-.02,.25]).as_matrix();truth[:3,3]=[2,-1,.3]
        scan=transform(points,np.linalg.inv(truth))+rng.normal(0,.005,points.shape)
        hint=truth.copy();hint[:3,3]+=[.7,-.6,.1];hint[:3,:3]=Rotation.from_euler('z',.17).as_matrix()@hint[:3,:3]
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'map.npz';model.save(path);loaded=ReferenceMap.load(path)
            result=loaded.relocalize(scan,hint)
            self.assertIsNotNone(result)
            self.assertLess(np.linalg.norm(result['pose'][:3,3]-truth[:3,3]),.03)
            self.assertLess(Rotation.from_matrix(truth[:3,:3].T@result['pose'][:3,:3]).magnitude(),.005)
            self.assertIsNone(loaded.align(scan+100,truth))

if __name__=='__main__':unittest.main()
