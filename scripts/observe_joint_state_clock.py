#!/usr/bin/env python3
"""Short, no-motion observation of the simulated joint-state publisher chain."""
import argparse
import csv
import json
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from dual_arm_a1_msgs.msg import GripperContactFeedback


TOPICS = (
    "/joint_state_broadcaster/joint_states",
    "/mj_left_gripper_sim_node/joint_states",
    "/mj_right_gripper_sim_node/joint_states",
    "/joint_states",
)


class Observer(Node):
    def __init__(self):
        super().__init__("joint_state_clock_observer")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.clock_seconds = None
        self.rows = []
        self.private_name_errors = []
        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        for topic in TOPICS:
            self.create_subscription(
                JointState, topic,
                lambda msg, source=topic: self.on_joint_state(source, msg), qos)

    def on_clock(self, msg):
        self.clock_seconds = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_joint_state(self, topic, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic in ("/mj_left_gripper_sim_node/joint_states",
                     "/mj_right_gripper_sim_node/joint_states"):
            side = "left" if "left" in topic else "right"
            expected_fingers = {f"mj_{side}_finger_joint1", f"mj_{side}_finger_joint2"}
            if not expected_fingers.issubset(set(msg.name)):
                self.private_name_errors.append({"topic": topic, "names": list(msg.name)})
        self.rows.append({
            "recv_steady_sec": time.monotonic(), "topic": topic,
            "stamp_sim_sec": stamp,
            "clock_sim_sec_at_receive": self.clock_seconds,
            "stamp_minus_clock_sec": (stamp - self.clock_seconds
                                      if self.clock_seconds is not None else None),
            "names": list(msg.name), "positions": list(msg.position),
        })


class FingerTraceObserver(Node):
    """Record typed hardware, private gripper, and aggregate finger samples."""
    def __init__(self, ready_file=None):
        super().__init__("finger_feedback_trace_observer")
        self.rows = []
        self.seen = set()
        self.ready_file = ready_file
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        for topic in ("/mj_left_gripper_sim_node/joint_states",
                      "/mj_right_gripper_sim_node/joint_states", "/joint_states"):
            self.create_subscription(
                JointState, topic,
                lambda msg, source=topic: self.on_joint_state(source, msg), qos)
        self.create_subscription(GripperContactFeedback, "/gripper_contact_feedback",
                                 self.on_typed, qos)

    def add_row(self, source, topic, arm, stamp, seq, name, position):
        self.rows.append((time.monotonic(), source, topic, arm, stamp, seq, name, position))

    def on_joint_state(self, topic, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        arm = "mj_left" if "left" in topic else "mj_right" if "right" in topic else ""
        self.seen.add(topic)
        for name, position in zip(msg.name, msg.position):
            if "finger_joint" in name:
                self.add_row("joint_state", topic, arm, stamp, "", name, position)
        self.write_ready_if_complete()

    def on_typed(self, msg):
        self.seen.add("typed:" + msg.arm)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for suffix, position in (("1", msg.finger_q1), ("2", msg.finger_q2)):
            self.add_row("typed_hardware", "/gripper_contact_feedback", msg.arm, stamp,
                         msg.update_seq, msg.arm + "_finger_joint" + suffix, position)
        self.write_ready_if_complete()

    def write_ready_if_complete(self):
        required = {"/mj_left_gripper_sim_node/joint_states",
                    "/mj_right_gripper_sim_node/joint_states", "/joint_states",
                    "typed:mj_left", "typed:mj_right"}
        if self.ready_file and required.issubset(self.seen):
            with open(self.ready_file, "w", encoding="utf-8") as stream:
                stream.write("ready\n")


def run_finger_trace(args):
    rclpy.init()
    node = FingerTraceObserver(args.ready_file)
    stopping = {"requested": False}
    signal.signal(signal.SIGINT, lambda _signum, _frame: stopping.__setitem__("requested", True))
    signal.signal(signal.SIGTERM, lambda _signum, _frame: stopping.__setitem__("requested", True))
    while rclpy.ok() and not stopping["requested"]:
        rclpy.spin_once(node, timeout_sec=0.05)
    with open(args.finger_trace_csv, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["recv_steady_sec", "source", "topic", "arm", "header_stamp_sim",
                         "update_seq", "joint_name", "position_m"])
        writer.writerows(node.rows)
    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=2.5)
    parser.add_argument("--out")
    parser.add_argument("--continuous-finger-trace", action="store_true")
    parser.add_argument("--finger-trace-csv")
    parser.add_argument("--ready-file")
    args = parser.parse_args()
    if args.continuous_finger_trace:
        if not args.finger_trace_csv:
            parser.error("--continuous-finger-trace requires --finger-trace-csv")
        run_finger_trace(args)
        return 0
    if not args.out:
        parser.error("--out is required unless --continuous-finger-trace is enabled")
    rclpy.init()
    node = Observer()
    discovery_started = time.monotonic()
    discovery_deadline = discovery_started + 8.0
    required_publishers = set(TOPICS) | {"/clock"}
    while rclpy.ok() and time.monotonic() < discovery_deadline:
        discovered = {
            topic for topic in required_publishers
            if node.get_publishers_info_by_topic(topic)
        }
        if discovered == required_publishers:
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    publisher_discovery_wall_sec = time.monotonic() - discovery_started
    deadline = time.monotonic() + args.seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    endpoints = {}
    for topic in ("/joint_states",) + TOPICS[:-1] + ("/clock",):
        endpoints[topic] = [
            {"node_name": info.node_name, "node_namespace": info.node_namespace,
             "topic_type": info.topic_type, "endpoint_gid": list(info.endpoint_gid)}
            for info in node.get_publishers_info_by_topic(topic)
        ]
    expected = {f"mj_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8)}
    expected |= {f"mj_{side}_finger_joint{j}" for side in ("left", "right") for j in ("1", "2")}
    global_samples = [row for row in node.rows if row["topic"] == "/joint_states"]
    latest_names = set(global_samples[-1]["names"]) if global_samples else set()
    skew = [abs(row["stamp_minus_clock_sec"]) for row in node.rows
            if row["stamp_minus_clock_sec"] is not None]
    report = {
        "duration_wall_sec": args.seconds,
        "publisher_discovery_wall_sec": publisher_discovery_wall_sec,
        "rows": node.rows,
        "publishers": endpoints,
        "global_joint_state_sample_count": len(global_samples),
        "global_missing_expected_joints": sorted(expected - latest_names),
        "private_finger_name_errors": node.private_name_errors,
        "max_abs_stamp_clock_delta_sec": max(skew) if skew else None,
    }
    with open(args.out, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    node.destroy_node()
    rclpy.shutdown()
    if (not global_samples or expected - latest_names or node.private_name_errors or
            not skew or max(skew) > 0.25):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
