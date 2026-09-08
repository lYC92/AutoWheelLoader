import unittest
from site_navigation import SafetyMap, rectangle, plan_route
from trajectory_control import Pose

class NavigationTests(unittest.TestCase):
    def test_overhang_and_braking(self):
        site=SafetyMap(bounds=(-30,40,-25,25),obstacles=[rectangle(7.5,0,0,-.5,.5,.5)])
        self.assertTrue(site.clear(Pose(0,0,0),0))
        self.assertFalse(site.stopping_corridor_clear(Pose(0,0,0),0,.6))
        self.assertFalse(site.clear(Pose(1,0,0),0))
    def test_unreachable_parking(self):
        site=SafetyMap(obstacles=[rectangle(4,0,0,-1,1,1)])
        with self.assertRaises(RuntimeError): plan_route(Pose(-10,0,0),Pose(4,0,0),1,site)
    def test_detour(self):
        site=SafetyMap(bounds=(-30,40,-25,25),obstacles=[rectangle(0,0,0,-1,1,1)])
        path=plan_route(Pose(-12,0,0),Pose(15,0,0),1,site,max_expansions=3000)
        self.assertGreater(max(abs(p.y) for p in path),3)
        # Every path rear reference must stay away from the actual obstacle;
        # articulated envelopes are checked during primitive integration.
        self.assertTrue(all(not(-1<=p.x<=1 and -1<=p.y<=1) for p in path))

if __name__=='__main__': unittest.main()
