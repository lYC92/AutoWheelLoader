"""Exercise the same Foxglove v1/CDR transport used by the visualization client."""
import json
import math
import struct
import time

from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from websockets.sync.client import connect


def check_wire(perception=False):
    required = {
        "/clock", "/loader/state", "/loader/command", "/joint_states",
        "/loader/bucket_interaction", "/loader/terrain_state",
        "/robot_description", "/tf", "/tf_static", "/loader/manual/status",
    }
    if perception:
        required.update({"/loader/sensors/lidar/scan/points",
                         "/loader/sensors/lidar/scan/points_effect",
                         "/loader/sensors/imu", "/loader_soil/observer/scan/points"})
    channels, samples, stamps, transforms = {}, {}, {}, {}
    reorder_ns = {}
    with connect("ws://127.0.0.1:8765", subprotocols=["foxglove.sdk.v1", "foxglove.websocket.v1"],
                 max_size=16*1024*1024, proxy=None, compression=None) as ws:
        assert ws.subprotocol in {"foxglove.sdk.v1", "foxglove.websocket.v1"}, "wrong WebSocket subprotocol"
        deadline = time.monotonic() + 30
        info = None
        while time.monotonic() < deadline and (not required <= channels.keys() or info is None):
            frame = ws.recv(timeout=max(0.01, deadline-time.monotonic()))
            if not isinstance(frame, str):
                continue
            obj = json.loads(frame)
            if obj["op"] == "serverInfo":
                info = obj
            if obj["op"] == "advertise":
                channels.update({c["topic"]: c for c in obj["channels"]})
        assert required <= channels.keys(), f"wire channels missing: {required-channels.keys()}"
        assert "clientPublish" in info["capabilities"], "bridge cannot accept Teleop"
        subscriptions = {}
        for i, topic in enumerate(sorted(required), 1):
            channel = channels[topic]
            assert channel["encoding"] == "cdr" and channel["schema"], f"invalid schema: {topic}"
            subscriptions[i] = topic
        ws.send(json.dumps({"op": "subscribe", "subscriptions": [
            {"id": i, "channelId": channels[t]["id"]} for i, t in subscriptions.items()]}))

        def receive(timeout):
            frame = ws.recv(timeout=timeout)
            if isinstance(frame, str):
                obj = json.loads(frame)
                assert not (obj["op"] == "status" and obj.get("level") == 2), obj
                return None, None
            if not frame or frame[0] != 1:
                return None, None
            sub_id, timestamp = struct.unpack_from("<IQ", frame, 1)
            topic = subscriptions[sub_id]
            message = deserialize_message(frame[13:], get_message(channels[topic]["schemaName"]))
            previous = stamps.setdefault(topic, [])
            # SDK log time follows callback arrival order, including clock setup.
            # Validate the source simulation timestamp used by the Plot panels.
            source_stamp = message.clock if topic == "/clock" else getattr(getattr(message, "header", None), "stamp", None)
            source_time = timestamp if source_stamp is None else source_stamp.sec*1_000_000_000+source_stamp.nanosec
            if source_stamp is not None:
                # Multithreaded bridge callbacks may deliver adjacent samples out
                # of order. Bound reordering, and require the source time to advance.
                rewind = max(0, max(previous, default=source_time)-source_time)
                reorder_ns[topic] = max(reorder_ns.get(topic, 0), rewind)
                assert rewind <= 100_000_000, f"source time rewound >100ms: {topic}"
            previous.append(source_time)
            samples[topic] = message
            if topic in ("/tf", "/tf_static"):
                for tf in message.transforms:
                    transforms[tf.child_frame_id] = tf.header.frame_id
            return topic, message

        deadline = time.monotonic() + 25
        latched = {"/tf_static", "/robot_description"}
        while time.monotonic() < deadline:
            receive(max(0.01, deadline-time.monotonic()))
            if all(len(stamps.get(t, [])) >= (1 if t in latched else 3) for t in required):
                break
        assert required <= samples.keys(), f"wire messages missing: {required-samples.keys()}"
        assert all(len(stamps[t]) >= (1 if t in latched else 3) for t in required), "telemetry stopped"
        assert all(max(stamps[t]) > min(stamps[t]) for t in required-latched), "source timestamps stopped"
        assert math.isfinite(samples["/loader/state"].longitudinal_speed_mps)
        assert len(samples["/joint_states"].position) >= 8, "incomplete joint states"
        assert "<robot" in samples["/robot_description"].data, "URDF missing on wire"
        if perception:
            for topic in ("/loader/sensors/lidar/scan/points",
                          "/loader/sensors/lidar/scan/points_effect", "/loader/sensors/imu"):
                frame_id = samples[topic].header.frame_id
                seen = set()
                while frame_id != "base_link" and frame_id in transforms and frame_id not in seen:
                    seen.add(frame_id)
                    frame_id = transforms[frame_id]
                assert frame_id == "base_link", f"{topic} cannot transform into base_link"
            cloud = samples["/loader/sensors/lidar/scan/points_effect"]
            assert cloud.width == 1024 and cloud.height == 32
            assert len(cloud.data) == cloud.height * cloud.row_step
        print(f"PASS  WebSocket handshake, CDR schemas and live samples for {len(required)} topics")
        print(f"INFO  maximum source timestamp reordering: {max(reorder_ns.values(), default=0)/1e6:.3f} ms")
        if perception:
            print("PASS  raw/effect point clouds and IMU connect to base_link through wire TF")

        # This input travels through the bridge, unlike the separate ROS gateway checks.
        ws.send(json.dumps({"op": "advertise", "channels": [{
            "id": 101, "topic": "/loader/manual/hydraulics", "encoding": "cdr",
            "schemaName": "geometry_msgs/msg/Twist"}]}))
        command = Twist()
        command.linear.z, command.angular.y = 0.25, -0.25
        success = False
        deadline = time.monotonic()+8
        try:
            while time.monotonic() < deadline:
                ws.send(struct.pack("<BI", 1, 101)+serialize_message(command))
                until = min(time.monotonic()+0.08, deadline)
                while time.monotonic() < until:
                    try:
                        topic, message = receive(max(0.001, until-time.monotonic()))
                    except TimeoutError:
                        break
                    if topic == "/loader/command" and message.lift_valve_command == 0.25 and message.tilt_valve_command == -0.25:
                        success = True
                if success:
                    break
        finally:
            ws.send(struct.pack("<BI", 1, 101)+serialize_message(Twist()))
            ws.send(json.dumps({"op": "unadvertise", "channelIds": [101]}))
        assert success, "WebSocket Teleop did not reach VehicleCommand"
        print("PASS  WebSocket Teleop -> bridge -> gateway -> VehicleCommand CDR round trip")
