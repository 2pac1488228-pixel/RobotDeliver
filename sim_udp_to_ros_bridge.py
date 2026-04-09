#!/usr/bin/env python3
"""
Bridge RobotDeliver UDP packets to ROS2 topics.

Expected UDP packet format (from RosConnectionManager):
{
  "topic": "/scan",
  "msg_type": "sensor_msgs/LaserScan",
  "timestamp": 12.34,
  "payload": "{\"angle_min\":..., ...}"
}

Usage:
  python tools/sim_udp_to_ros_bridge.py --listen-ip 0.0.0.0 --listen-port 9000
"""

import argparse
import json
import socket
import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Any, Dict, Optional


@dataclass
class Packet:
    topic: str
    msg_type: str
    timestamp: float
    payload: Dict[str, Any]


def parse_packet(raw: bytes) -> Optional[Packet]:
    try:
        envelope = json.loads(raw.decode("utf-8"))
        payload_raw = envelope.get("payload", "{}")
        payload = json.loads(payload_raw) if isinstance(payload_raw, str) else payload_raw
        return Packet(
            topic=envelope.get("topic", "/unknown"),
            msg_type=envelope.get("msg_type", "std_msgs/String"),
            timestamp=float(envelope.get("timestamp", time.time())),
            payload=payload if isinstance(payload, dict) else {},
        )
    except Exception:
        return None


class UdpReceiver(threading.Thread):
    def __init__(self, listen_ip: str, listen_port: int, queue: Queue, verbose: bool = False):
        super().__init__(daemon=True)
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.queue = queue
        self.verbose = verbose
        self._stop = threading.Event()

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.listen_ip, self.listen_port))
        sock.settimeout(0.5)
        if self.verbose:
            print(f"[UDP] Listening on {self.listen_ip}:{self.listen_port}")

        while not self._stop.is_set():
            try:
                data, addr = sock.recvfrom(65535)
                pkt = parse_packet(data)
                if pkt is None:
                    if self.verbose:
                        print(f"[UDP] Failed to parse packet from {addr}")
                    continue
                self.queue.put(pkt)
            except socket.timeout:
                continue
            except Exception as exc:
                print(f"[UDP] Receiver error: {exc}")

        sock.close()

    def stop(self) -> None:
        self._stop.set()


class Ros2Bridge:
    def __init__(self, queue: Queue, node_name: str, verbose: bool = False):
        self.queue = queue
        self.verbose = verbose

        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from sensor_msgs.msg import LaserScan, NavSatFix, Image
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Quaternion

        self.rclpy = rclpy
        self.String = String
        self.LaserScan = LaserScan
        self.NavSatFix = NavSatFix
        self.Image = Image
        self.Odometry = Odometry
        self.Quaternion = Quaternion

        class BridgeNode(Node):
            pass

        self.rclpy.init()
        self.node = BridgeNode(node_name)
        self.publishers: Dict[str, Any] = {}
        self.node.create_timer(0.01, self._drain_queue)

        self.node.get_logger().info("ROS2 bridge started.")

    def _stamp(self, sec: float):
        from builtin_interfaces.msg import Time

        whole = int(sec)
        frac = sec - whole
        t = Time()
        t.sec = whole
        t.nanosec = int(frac * 1e9)
        return t

    def _get_pub(self, topic: str, msg_type: str):
        key = f"{topic}|{msg_type}"
        if key in self.publishers:
            return self.publishers[key]

        if msg_type == "sensor_msgs/LaserScan":
            pub = self.node.create_publisher(self.LaserScan, topic, 10)
        elif msg_type == "sensor_msgs/NavSatFix":
            pub = self.node.create_publisher(self.NavSatFix, topic, 10)
        elif msg_type == "nav_msgs/Odometry":
            pub = self.node.create_publisher(self.Odometry, topic, 10)
        elif msg_type == "sensor_msgs/Image":
            pub = self.node.create_publisher(self.Image, topic, 10)
        else:
            pub = self.node.create_publisher(self.String, topic, 10)
        self.publishers[key] = pub
        return pub

    def _yaw_to_quat(self, yaw_rad: float):
        import math

        q = self.Quaternion()
        q.w = math.cos(yaw_rad * 0.5)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_rad * 0.5)
        return q

    def _build_message(self, pkt: Packet):
        import math

        p = pkt.payload
        if pkt.msg_type == "sensor_msgs/LaserScan":
            msg = self.LaserScan()
            msg.header.stamp = self._stamp(float(p.get("stamp", pkt.timestamp)))
            msg.header.frame_id = "laser"
            msg.angle_min = float(p.get("angle_min", 0.0))
            msg.angle_max = float(p.get("angle_max", 2.0 * math.pi))
            msg.angle_increment = float(p.get("angle_increment", 0.0))
            msg.range_min = float(p.get("range_min", 0.0))
            msg.range_max = float(p.get("range_max", 0.0))
            ranges_csv = p.get("ranges_csv", "")
            msg.ranges = [float(x) for x in ranges_csv.split(",") if x]
            return msg

        if pkt.msg_type == "sensor_msgs/NavSatFix":
            msg = self.NavSatFix()
            msg.header.stamp = self._stamp(float(p.get("stamp", pkt.timestamp)))
            msg.header.frame_id = "gps"
            msg.latitude = float(p.get("latitude", 0.0))
            msg.longitude = float(p.get("longitude", 0.0))
            msg.altitude = float(p.get("altitude", 0.0))
            return msg

        if pkt.msg_type == "nav_msgs/Odometry":
            msg = self.Odometry()
            msg.header.stamp = self._stamp(float(p.get("stamp", pkt.timestamp)))
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x = float(p.get("x", 0.0))
            msg.pose.pose.position.y = float(p.get("y", 0.0))
            msg.pose.pose.position.z = float(p.get("z", 0.0))
            yaw_deg = float(p.get("yaw_deg", 0.0))
            msg.pose.pose.orientation = self._yaw_to_quat(math.radians(yaw_deg))
            msg.twist.twist.linear.x = float(p.get("linear_mps", 0.0))
            msg.twist.twist.angular.z = math.radians(float(p.get("angular_degps", 0.0)))
            return msg

        if pkt.msg_type == "sensor_msgs/Image":
            msg = self.Image()
            msg.header.stamp = self._stamp(float(p.get("stamp", pkt.timestamp)))
            msg.header.frame_id = "camera"
            msg.width = int(p.get("width", 0))
            msg.height = int(p.get("height", 0))
            msg.encoding = "mono8"
            msg.is_bigendian = 0
            msg.step = msg.width
            msg.data = bytes(msg.width * msg.height)
            return msg

        msg = self.String()
        msg.data = json.dumps({"msg_type": pkt.msg_type, "payload": pkt.payload}, ensure_ascii=False)
        return msg

    def _drain_queue(self):
        while True:
            try:
                pkt = self.queue.get_nowait()
            except Empty:
                break
            try:
                pub = self._get_pub(pkt.topic, pkt.msg_type)
                msg = self._build_message(pkt)
                pub.publish(msg)
                if self.verbose:
                    self.node.get_logger().info(f"Published {pkt.msg_type} -> {pkt.topic}")
            except Exception as exc:
                self.node.get_logger().error(f"Publish error: {exc}")

    def spin(self):
        self.rclpy.spin(self.node)

    def shutdown(self):
        try:
            self.node.destroy_node()
        finally:
            self.rclpy.shutdown()


def run_print_mode(queue: Queue, verbose: bool = False):
    print("[MOCK] ROS not enabled. Printing incoming packets...")
    while True:
        pkt = queue.get()
        if verbose:
            print(f"[MOCK] {pkt.msg_type} {pkt.topic} {pkt.payload}")
        else:
            print(f"[MOCK] {pkt.topic} ({pkt.msg_type})")


def main():
    parser = argparse.ArgumentParser(description="RobotDeliver UDP -> ROS2 bridge")
    parser.add_argument("--listen-ip", default="0.0.0.0", help="UDP listen IP")
    parser.add_argument("--listen-port", type=int, default=9000, help="UDP listen port")
    parser.add_argument("--node-name", default="robotdeliver_udp_bridge", help="ROS2 node name")
    parser.add_argument("--mock-only", action="store_true", help="Do not use ROS; just print packets")
    parser.add_argument("--verbose", action="store_true", help="Verbose logs")
    args = parser.parse_args()

    queue: Queue = Queue()
    receiver = UdpReceiver(args.listen_ip, args.listen_port, queue, verbose=args.verbose)
    receiver.start()

    bridge = None
    try:
        if args.mock_only:
            run_print_mode(queue, verbose=args.verbose)
            return

        try:
            bridge = Ros2Bridge(queue=queue, node_name=args.node_name, verbose=args.verbose)
            bridge.spin()
        except ModuleNotFoundError as exc:
            print(f"[WARN] ROS2 modules not found ({exc}). Switching to --mock-only mode.")
            run_print_mode(queue, verbose=args.verbose)
    except KeyboardInterrupt:
        print("\nStopping bridge...")
    finally:
        receiver.stop()
        if bridge is not None:
            bridge.shutdown()


if __name__ == "__main__":
    main()

