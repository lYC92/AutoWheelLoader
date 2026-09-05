#!/usr/bin/env python3
"""Verify the Foxglove monitoring chain against the live loader simulation.

Checks, in order:
  1. foxglove_bridge accepts TCP on 127.0.0.1:8765 and its node is in the graph.
  2. Every topic referenced by foxglove/loader_simulation_layout.json exists in
     the ROS graph (perception-only topics are skipped unless they exist).
  3. Every message field path used by the layout's Plot/Gauge panels resolves
     against the actual message type advertised on that topic.
  4. The manual gateway closes the loop: Teleop Twist input produces the
     expected VehicleCommand, stale input brakes, and emergency stop latches.
"""

from __future__ import annotations

import json
import math
import re
import socket
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from loader_sim_msgs.msg import VehicleCommand
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool, String

PROJECT_ROOT = Path(__file__).resolve().parents[2]
LAYOUT_PATH = PROJECT_ROOT / "foxglove" / "loader_simulation_layout.json"

PERCEPTION_ONLY_TOPICS = {
    "/loader/sensors/imu",
    "/loader/sensors/lidar/scan/points",
    "/loader_soil/observer/scan/points",
}

ARRAY_SUFFIX = re.compile(r"\[[^\]]*\]$")


class CheckFailed(RuntimeError):
    pass


def collect_layout_references(layout: dict) -> tuple[set[str], dict[str, list[str]]]:
    """Return (bare topics, {topic: [field paths]}) referenced by the layout."""
    topics: set[str] = set()
    field_paths: dict[str, list[str]] = {}
    for panel in layout.get("configById", {}).values():
        if not isinstance(panel, dict):
            continue
        for key in ("topicPath", "topic", "topicName"):
            value = panel.get(key)
            if isinstance(value, str) and value.startswith("/"):
                topics.add(value)
        if isinstance(panel.get("path"), str):
            value = panel["path"]
            topic, _, field = value.partition(".")
            topics.add(topic)
            if field:
                field_paths.setdefault(topic, []).append(field)
        for entry in panel.get("paths", []) or []:
            value = entry.get("value", "")
            if not value.startswith("/"):
                continue
            topic, _, field = value.partition(".")
            topics.add(topic)
            if field:
                field_paths.setdefault(topic, []).append(field)
        for key in panel.get("topics", {}) or {}:
            if key.startswith("/"):
                topics.add(key)
    return topics, field_paths


def resolve_field_path(message_type: str, field_path: str) -> None:
    """Walk field_path through nested message types; raise CheckFailed if invalid."""
    current = get_message(message_type)
    for segment in field_path.split("."):
        segment = ARRAY_SUFFIX.sub("", segment)
        if not segment:
            raise CheckFailed(f"empty segment in field path '{field_path}'")
        fields = current.get_fields_and_field_types()
        if segment not in fields:
            raise CheckFailed(
                f"field '{segment}' of path '{field_path}' not found in "
                f"{current.__name__}; available: {sorted(fields)}"
            )
        field_type = fields[segment]
        if "/" in field_type:
            current = get_message(field_type)


class BridgeCheck(Node):
    def __init__(self) -> None:
        super().__init__("foxglove_bridge_smoke_check")
        self.commands: list[VehicleCommand] = []
        self.statuses: list[str] = []
        self.create_subscription(
            VehicleCommand, "/loader/command", self.on_command, 50
        )
        self.create_subscription(String, "/loader/manual/status", self.on_status, 10)
        self.drive_pub = self.create_publisher(Twist, "/loader/manual/drive", 10)
        self.estop_pub = self.create_publisher(
            Bool, "/loader/manual/emergency_stop", 10
        )

    def on_command(self, message: VehicleCommand) -> None:
        self.commands.append(message)

    def on_status(self, message: String) -> None:
        self.statuses.append(message.data)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout_s: float, description: str):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise CheckFailed(f"timed out waiting for: {description}")


def check_bridge_socket() -> None:
    with socket.create_connection(("127.0.0.1", 8765), timeout=5.0):
        pass
    print("PASS  foxglove_bridge accepts TCP connections on 127.0.0.1:8765")


def check_layout_topics(node: BridgeCheck) -> None:
    topics, field_paths = collect_layout_references(
        json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
    )
    node.wait_for(
        lambda: any(t == "/loader/state" for t, _ in node.get_topic_names_and_types()),
        30.0,
        "/loader/state to appear in the ROS graph",
    )
    graph = dict()
    for name, types in node.get_topic_names_and_types():
        graph.setdefault(name, []).extend(types)

    def foxglove_node_present() -> bool:
        return "foxglove_bridge" in node.get_node_names()

    try:
        node.wait_for(foxglove_node_present, 15.0, "/foxglove_bridge node in the graph")
    except CheckFailed:
        discovered = sorted(node.get_node_names_and_namespaces())
        raise CheckFailed(
            f"/foxglove_bridge node is not in the ROS graph; discovered: {discovered}"
        )
    print("PASS  /foxglove_bridge node is running")

    missing = []
    skipped = []
    for topic in sorted(topics):
        if topic not in graph:
            if topic in PERCEPTION_ONLY_TOPICS:
                skipped.append(topic)
            else:
                missing.append(topic)
    if missing:
        raise CheckFailed(f"layout topics missing from the ROS graph: {missing}")
    print(
        f"PASS  {len(topics) - len(skipped)} layout topics exist in the ROS graph "
        f"(skipped {len(skipped)} perception-only topics)"
    )

    checked = 0
    for topic, paths in sorted(field_paths.items()):
        if topic not in graph:
            continue
        message_type = graph[topic][0]
        for path in paths:
            resolve_field_path(message_type, path)
            checked += 1
    print(f"PASS  {checked} layout field paths resolve against live message types")


def check_manual_gateway(node: BridgeCheck) -> None:
    def latest_command() -> VehicleCommand | None:
        return node.commands[-1] if node.commands else None

    node.wait_for(
        lambda: latest_command() is not None,
        15.0,
        "manual gateway to publish /loader/command",
    )

    drive = Twist()
    drive.linear.x = 0.5
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        node.drive_pub.publish(drive)
        node.spin_for(0.05)
    node.wait_for(
        lambda: latest_command() is not None
        and abs(latest_command().traction_torque_nm - 5000.0) < 1.0
        and latest_command().gear == VehicleCommand.GEAR_FORWARD
        and latest_command().brake_command == 0.0,
        5.0,
        "drive input to become a 5000 N*m forward command",
    )
    print("PASS  Teleop drive input maps to a forward traction command")

    node.spin_for(0.8)
    node.wait_for(
        lambda: latest_command() is not None
        and abs(latest_command().traction_torque_nm) < 1e-6
        and latest_command().brake_command == 1.0
        and latest_command().gear == VehicleCommand.GEAR_NEUTRAL,
        5.0,
        "stale input to fall back to neutral braking",
    )
    print("PASS  stale Teleop input falls back to neutral braking within the timeout")

    node.estop_pub.publish(Bool(data=True))
    node.wait_for(
        lambda: latest_command() is not None and latest_command().emergency_stop,
        5.0,
        "emergency stop to latch into /loader/command",
    )
    node.estop_pub.publish(Bool(data=False))
    node.wait_for(
        lambda: latest_command() is not None and not latest_command().emergency_stop,
        5.0,
        "emergency stop release",
    )
    print("PASS  emergency stop latches and releases through the gateway")

    node.wait_for(lambda: len(node.statuses) > 0, 5.0, "/loader/manual/status JSON")
    status = json.loads(node.statuses[-1])
    required_keys = {
        "enabled",
        "emergency_stop",
        "drive_input_fresh",
        "hydraulic_input_fresh",
        "gear",
        "traction_torque_nm",
        "target_articulation_angle_rad",
        "lift_valve_command",
        "tilt_valve_command",
    }
    missing_keys = required_keys - status.keys()
    if missing_keys:
        raise CheckFailed(f"/loader/manual/status is missing keys: {missing_keys}")
    print("PASS  /loader/manual/status publishes the documented JSON summary")


def main() -> int:
    try:
        check_bridge_socket()
        rclpy.init()
        node = BridgeCheck()
        try:
            check_layout_topics(node)
            check_manual_gateway(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except CheckFailed as error:
        print(f"FAIL  {error}", file=sys.stderr)
        return 1
    print("PASS  Foxglove bridge, layout and manual gateway verified end to end.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
