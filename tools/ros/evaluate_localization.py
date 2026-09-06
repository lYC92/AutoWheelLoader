#!/usr/bin/env python3
"""Record algorithm odometry and independent Gazebo truth; align only at start.

Ground truth is consumed here for evaluation, never fed to the estimator.
Reports initial-pose-aligned translation/rotation error without trajectory fitting.
"""
import argparse
import csv
import json
import signal
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation, Slerp


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_matrix(position, orientation):
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()
    pose[:3, 3] = [position.x, position.y, position.z]
    return pose


def compare(odometry, ground_truth):
    truth = sorted({t: pose for t, pose in ground_truth}.items())
    if len(truth) < 2:
        raise RuntimeError("missing Gazebo model truth")
    times = np.array([t for t, _ in truth])
    poses = np.array([p for _, p in truth])
    rotations = Slerp(times, Rotation.from_matrix(poses[:, :3, :3]))
    aligned = []
    alignment = None
    for stamp, estimate, latency in odometry:
        if not times[0] <= stamp <= times[-1]:
            continue
        k = int(np.clip(np.searchsorted(times, stamp), 1, len(times)-1))
        if times[k] - times[k-1] > 0.1:
            continue
        truth_pose = np.eye(4)
        truth_pose[:3, :3] = rotations(stamp).as_matrix()
        truth_pose[:3, 3] = [np.interp(stamp, times, poses[:, j, 3]) for j in range(3)]
        if alignment is None:
            alignment = truth_pose @ np.linalg.inv(estimate)
        world_estimate = alignment @ estimate
        position_error = np.linalg.norm(world_estimate[:3, 3] - truth_pose[:3, 3])
        angle_error = Rotation.from_matrix(truth_pose[:3, :3].T @ world_estimate[:3, :3]).magnitude()
        aligned.append([stamp, *truth_pose[:3, 3], *world_estimate[:3, 3],
                        position_error, np.degrees(angle_error), latency])
    return np.asarray(aligned)


class Recorder(Node):
    def __init__(self):
        super().__init__("loader_localization_evaluator", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odometry, self.truth = [], []
        self.create_subscription(Odometry, "/loader/localization/odometry", self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/loader/ground_truth/odometry", self.on_truth, qos_profile_sensor_data)

    def on_odometry(self, message):
        if message.header.frame_id != "odom" or message.child_frame_id != "base_link":
            raise RuntimeError("unexpected estimator frame contract")
        stamp = seconds(message.header.stamp)
        latency = self.get_clock().now().nanoseconds/1e9-stamp
        self.odometry.append((stamp, pose_matrix(message.pose.pose.position, message.pose.pose.orientation), latency))

    def on_truth(self, message):
        if message.header.frame_id != "world" or message.child_frame_id != "base_link":
            raise RuntimeError("unexpected ground-truth frame contract")
        self.truth.append((seconds(message.header.stamp),
                           pose_matrix(message.pose.pose.position, message.pose.pose.orientation)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=120.)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Recorder()
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    deadline = time.monotonic()+args.duration
    try:
        while running and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raw = {"odometry": [[t, p.tolist(), lag] for t, p, lag in node.odometry],
           "ground_truth": [[t, p.tolist()] for t, p in node.truth]}
    (output/"poses.json").write_text(json.dumps(raw))
    rows = compare(node.odometry, node.truth)
    if len(rows) < 50 or rows[-1, 0]-rows[0, 0] < 5:
        raise RuntimeError(f"insufficient matched trajectory: {len(rows)} poses")
    if not np.isfinite(rows).all():
        raise RuntimeError("non-finite pose/error")
    travel = float(np.linalg.norm(np.diff(rows[:, 1:4], axis=0), axis=1).sum())
    if travel < 0.5:
        raise RuntimeError(f"stationary test cannot validate odometry: {travel:.3f} m")
    metrics = {
        "algorithm": "KISS-ICP 1ffa7d7512f10bfc8b1185095011fa31184019e3",
        "input": "/loader/localization/points",
        "sensor_source": "/loader/sensors/lidar/scan/points_effect",
        "preprocessing": "nominal level-ground crop: lidar z >= -2.7 m, 3 < range < 50 m",
        "alignment": "initial pose only; interpolated Gazebo truth; no trajectory fitting",
        "matched_poses": len(rows), "truth_samples": len(node.truth),
        "duration_sim_s": float(rows[-1, 0]-rows[0, 0]), "travel_m": travel,
        "translation_rmse_m": float(np.sqrt(np.mean(rows[:, 7]**2))),
        "translation_max_m": float(rows[:, 7].max()),
        "rotation_rmse_deg": float(np.sqrt(np.mean(rows[:, 8]**2))),
        "rotation_max_deg": float(rows[:, 8].max()),
        "sim_latency_p95_s": float(np.percentile(rows[:, 9], 95)),
        "odometry_sim_hz": float((len(rows)-1)/(rows[-1, 0]-rows[0, 0])),
    }
    metrics["nominal_accuracy_pass"] = metrics["translation_rmse_m"] <= 0.15
    with (output/"trajectory.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "truth_x", "truth_y", "truth_z", "estimate_x", "estimate_y", "estimate_z",
                         "position_error_m", "rotation_error_deg", "sim_latency_s"])
        writer.writerows(rows)
    (output/"metrics.json").write_text(json.dumps(metrics, indent=2)+"\n")
    print(json.dumps(metrics, indent=2))
    print("PASS  moving localization pipeline and independent truth evaluation")
    print("PASS  nominal accuracy target" if metrics["nominal_accuracy_pass"] else
          "OPEN  nominal accuracy target not met; baseline recorded, calibration remains")


if __name__ == "__main__":
    main()
