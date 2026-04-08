#!/usr/bin/env python3
import argparse
import csv
import datetime
import math
import os
import signal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


class ExperimentDataRecorder(Node):
    def __init__(self, mode: str, output_data_dir: str, sample_hz: float) -> None:
        super().__init__("experiment_data_recorder")
        self.mode = mode
        self.output_data_dir = output_data_dir
        self.sample_hz = max(sample_hz, 1.0)

        os.makedirs(self.output_data_dir, exist_ok=True)

        self.left_wrench = WrenchStamped()
        self.right_wrench = WrenchStamped()

        self.current_k_scale = 100.0
        self.current_d_scale = 100.0
        self.current_accel_norm = 0.0

        self.last_joint_vel = None
        self.last_joint_time = None

        self.history_time = []
        self.history_ly = []
        self.history_ry = []
        self.history_stress = []
        self.history_accel = []
        self.history_k_scale = []
        self.history_d_scale = []

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.should_exit = False
        self.stop_reason = ""
        self.saved = False

        self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster_left/wrench",
            self.left_cb,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster_right/wrench",
            self.right_cb,
            10,
        )
        self.create_subscription(JointState, "/joint_states", self.joint_cb, 10)
        self.create_subscription(
            Float64MultiArray,
            "/adaptive_impedance_params",
            self.adaptive_cb,
            10,
        )
        self.create_subscription(String, "/task_status", self.status_cb, 10)

        self.create_timer(1.0 / self.sample_hz, self.sample_once)

    def left_cb(self, msg: WrenchStamped) -> None:
        self.left_wrench = msg

    def right_cb(self, msg: WrenchStamped) -> None:
        self.right_wrench = msg

    def adaptive_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 6:
            self.current_k_scale = float(msg.data[4])
            self.current_d_scale = float(msg.data[5])
        elif len(msg.data) >= 2:
            self.current_k_scale = float(msg.data[0])
            self.current_d_scale = float(msg.data[1])

    def joint_cb(self, msg: JointState) -> None:
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not msg.velocity:
            return

        vel = list(msg.velocity)
        if self.last_joint_vel is not None and self.last_joint_time is not None:
            if len(vel) == len(self.last_joint_vel):
                dt = current_time - self.last_joint_time
                if dt > 0.0:
                    sq_sum = 0.0
                    for v_now, v_prev in zip(vel, self.last_joint_vel):
                        acc = (v_now - v_prev) / dt
                        sq_sum += acc * acc
                    self.current_accel_norm = math.sqrt(sq_sum)

        self.last_joint_vel = vel
        self.last_joint_time = current_time

    def status_cb(self, msg: String) -> None:
        if msg.data == "DONE":
            self.stop_reason = "task_status_DONE"
            self.should_exit = True
        elif msg.data == "ERROR":
            self.stop_reason = "task_status_ERROR"
            self.should_exit = True

    def sample_once(self) -> None:
        current_t = self.get_clock().now().nanoseconds / 1e9 - self.start_time

        lf = self.left_wrench.wrench.force
        rf = self.right_wrench.wrench.force
        internal_stress = abs(lf.y + rf.y) / 2.0

        self.history_time.append(current_t)
        self.history_ly.append(float(lf.y))
        self.history_ry.append(float(rf.y))
        self.history_stress.append(float(internal_stress))
        self.history_accel.append(float(self.current_accel_norm))
        self.history_k_scale.append(float(self.current_k_scale))
        self.history_d_scale.append(float(self.current_d_scale))

    def save_csv(self, reason: str) -> str:
        if self.saved:
            return ""

        self.saved = True
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_data_dir, f"experiment_{self.mode}_{timestamp}.csv")

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "Time",
                    "Left_Y_Force",
                    "Right_Y_Force",
                    "Internal_Stress",
                    "Joint_Accel_Norm",
                    "K_Scale",
                    "D_Scale",
                ]
            )
            for row in zip(
                self.history_time,
                self.history_ly,
                self.history_ry,
                self.history_stress,
                self.history_accel,
                self.history_k_scale,
                self.history_d_scale,
            ):
                writer.writerow(row)

        print(
            f"[experiment_data_recorder] Saved CSV: {filename} "
            f"(rows={len(self.history_time)}, reason={reason})"
        )
        return filename


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record experiment time-series data into CSV")
    parser.add_argument("mode", nargs="?", default="compliant_chomp", help="control mode label")
    parser.add_argument(
        "--output-data-dir",
        default=os.getenv("EXPERIMENT_DATA_DIR", "experiment_data"),
        help="directory to write experiment CSV files",
    )
    parser.add_argument("--sample-hz", type=float, default=20.0, help="sampling rate in Hz")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    rclpy.init()
    node = ExperimentDataRecorder(args.mode, args.output_data_dir, args.sample_hz)

    def _stop(signum, _frame):
        if not node.stop_reason:
            node.stop_reason = f"signal_{signum}"
        node.should_exit = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        reason = node.stop_reason if node.stop_reason else "shutdown"
        node.save_csv(reason)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
