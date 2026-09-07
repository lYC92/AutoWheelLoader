import math
import unittest
from trajectory_control import Limits, Pose, Tracker, transfer_path


class TrajectoryTests(unittest.TestCase):
    def test_forward_and_reverse_turns(self):
        for gear, side in ((1,1),(1,-1),(-1,1),(-1,-1)):
            with self.subTest(gear=gear,side=side):
                start=Pose(0,0,0)
                goal=Pose(gear*16,side*gear*16,side*math.pi/2)
                limits=Limits()
                tracker=Tracker(transfer_path(start,goal,gear),gear,limits)
                pose, speed, articulation = start,0.0,0.0
                errors=[]
                for _ in range(5000):
                    dt=0.02
                    command=tracker.update(pose,speed,dt,articulation)
                    errors.append(command.cross_track_error)
                    if command.reached:
                        break
                    acceleration=max(-0.8,min(0.8,2*(command.target_speed-speed)))
                    speed+=acceleration*dt
                    rate=(command.articulation-articulation)/dt
                    articulation=command.articulation
                    self.assertLessEqual(abs(rate),limits.articulation_rate+1e-9)
                    self.assertLessEqual(abs(articulation),limits.articulation+1e-9)
                    # No-slip constraints at the two axles; includes the rear
                    # frame rotation induced by changing the articulation.
                    yaw_rate=(speed*math.sin(articulation)-limits.front_axle_to_joint*rate)/(limits.front_axle_to_joint+limits.rear_axle_to_joint*math.cos(articulation))
                    pose=Pose(pose.x+speed*math.cos(pose.yaw)*dt,
                              pose.y+speed*math.sin(pose.yaw)*dt,pose.yaw+yaw_rate*dt)
                self.assertTrue(command.reached, f"failed to park: {pose}")
                self.assertLess(math.sqrt(sum(e*e for e in errors)/len(errors)),0.3)
                self.assertLess(math.hypot(pose.x-goal.x,pose.y-goal.y),0.3)

    def test_invalid_path_and_stale_feedback(self):
        with self.assertRaises(ValueError):
            transfer_path(Pose(0,0,0),Pose(1,1,math.pi/2),1)
        tracker=Tracker(transfer_path(Pose(0,0,0),Pose(10,0,0),1),1)
        with self.assertRaises(RuntimeError):
            tracker.update(Pose(0,3,0),0,0.02)
        with self.assertRaises(RuntimeError):
            tracker.update(Pose(0,0,0),0,0.5)


if __name__ == "__main__":
    unittest.main()
