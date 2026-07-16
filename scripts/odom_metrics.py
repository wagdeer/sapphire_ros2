#!/usr/bin/env python3
"""Record Sapphire odometry as TUM trajectory and emit a JSON summary."""

import argparse
import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class OdomMetrics(Node):
    def __init__(self, trajectory_path: Path, summary_path: Path) -> None:
        super().__init__("sapphire_odom_metrics")
        self._trajectory_path = trajectory_path
        self._summary_path = summary_path
        self._stream = trajectory_path.open("w", encoding="utf-8")
        self._subscription = self.create_subscription(
            Odometry, "/sapphire/odometry", self._on_odometry, 100
        )
        self._count = 0
        self._first_stamp = None
        self._last_stamp = None
        self._last_position = None
        self._path_length = 0.0

    def _on_odometry(self, message: Odometry) -> None:
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        xyz = (position.x, position.y, position.z)
        if not all(math.isfinite(value) for value in xyz):
            return
        if self._last_position is not None:
            self._path_length += math.dist(self._last_position, xyz)
        self._last_position = xyz
        self._first_stamp = stamp if self._first_stamp is None else self._first_stamp
        self._last_stamp = stamp
        self._count += 1
        self._stream.write(
            f"{stamp:.9f} {position.x:.9f} {position.y:.9f} {position.z:.9f} "
            f"{orientation.x:.9f} {orientation.y:.9f} "
            f"{orientation.z:.9f} {orientation.w:.9f}\n"
        )

    def close(self) -> None:
        self._stream.close()
        duration = (
            self._last_stamp - self._first_stamp
            if self._first_stamp is not None and self._last_stamp is not None
            else 0.0
        )
        summary = {
            "messages": self._count,
            "duration_sec": duration,
            "path_length_m": self._path_length,
            "final_position_m": self._last_position,
        }
        self._summary_path.write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(summary), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--summary", required=True, type=Path)
    args = parser.parse_args()
    args.trajectory.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = OdomMetrics(args.trajectory, args.summary)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
