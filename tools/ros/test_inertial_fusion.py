import unittest
import numpy as np
from inertial_fusion import InertialFilter

class FusionTests(unittest.TestCase):
    def test_delayed_pose_and_bias(self):
        f=InertialFilter()
        for k in range(1001):
            t=k*.005
            f.predict(t,[.02,0,9.80665],[0,0,.002])
            if k>=20 and k%20==0:
                self.assertTrue(f.correct(t-.05,[0,0,0],[0,0,0,1]))
        self.assertLess(np.linalg.norm(f.p),.02)
        self.assertTrue(f.healthy(5))
        self.assertFalse(f.healthy(5.6))
        self.assertGreaterEqual(np.linalg.eigvalsh(f.P).min(),-1e-12)
        self.assertFalse(f.correct(4.99,[100,0,0],[0,0,0,1]))
        self.assertLess(np.linalg.norm(f.p),.02)
    def test_bad_clock_and_values(self):
        f=InertialFilter();f.predict(1,[0,0,9.80665],[0,0,0])
        with self.assertRaises(ValueError):f.predict(2,[0,0,9.80665],[0,0,0])
        with self.assertRaises(ValueError):f.predict(1.01,[np.nan,0,0],[0,0,0])

if __name__=='__main__':unittest.main()
