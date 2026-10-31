#!/usr/bin/env python3
"""Record Sapphire odometry as TUM trajectory and emit a JSON summary.

The file follows /sapphire/trajectory once that path is published. Each path
message is the full per-keyframe replay, so a loop closure replaces history
already written. Until the first path message, /sapphire/odometry is appended
as before.
"""

import argparse
import json
import math
import os
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _tum_line(stamp: float, position, orientation) -> str:
    return (
        f"{stamp:.9f} {position[0]:.9f} {position[1]:.9f} {position[2]:.9f} "
        f"{orientation[0]:.9f} {orientation[1]:.9f} "
        f"{orientation[2]:.9f} {orientation[3]:.9f}\n"
    )


class OdomMetrics(Node):
    def __init__(self, trajectory_path: Path, summary_path: Path) -> None:
        super().__init__("sapphire_odom_metrics")
        self._trajectory_path = trajectory_path
        self._summary_path = summary_path
        self._stream = trajectory_path.open("w", encoding="utf-8")
        self._odom_poses: list[tuple[float, tuple, tuple]] = []
        self._path_poses: list[tuple[float, tuple, tuple]] | None = None
        self._subscription = self.create_subscription(
            Odometry, "/sapphire/odometry", self._on_odometry, 100
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_subscription = self.create_subscription(
            PathMsg, "/sapphire/trajectory", self._on_path, path_qos
        )

    def _on_odometry(self, message: Odometry) -> None:
        if self._path_poses is not None:
            return
        stamp = _stamp_seconds(message.header.stamp)
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        xyz = (position.x, position.y, position.z)
        quat = (orientation.x, orientation.y, orientation.z, orientation.w)
        if not all(math.isfinite(value) for value in (*xyz, *quat, stamp)):
            return
        self._odom_poses.append((stamp, xyz, quat))
        self._stream.write(_tum_line(stamp, xyz, quat))

    def _on_path(self, message: PathMsg) -> None:
        poses: list[tuple[float, tuple, tuple]] = []
        for pose_stamped in message.poses:
            stamp = _stamp_seconds(pose_stamped.header.stamp)
            position = pose_stamped.pose.position
            orientation = pose_stamped.pose.orientation
            xyz = (position.x, position.y, position.z)
            quat = (orientation.x, orientation.y, orientation.z, orientation.w)
            if not all(math.isfinite(value) for value in (*xyz, *quat, stamp)):
                continue
            poses.append((stamp, xyz, quat))
        if not poses:
            return
        self._path_poses = poses
        self._rewrite(poses)

    def _rewrite(self, poses: list[tuple[float, tuple, tuple]]) -> None:
        if not self._stream.closed:
            self._stream.close()
        temporary = self._trajectory_path.with_name(
            self._trajectory_path.name + ".tmp"
        )
        with temporary.open("w", encoding="utf-8", newline="\n") as stream:
            for stamp, xyz, quat in poses:
                stream.write(_tum_line(stamp, xyz, quat))
        os.replace(temporary, self._trajectory_path)

    def _summary_poses(self) -> list[tuple[float, tuple, tuple]]:
        if self._path_poses is not None:
            return self._path_poses
        return self._odom_poses

    def close(self) -> None:
        poses = self._summary_poses()
        if self._path_poses is not None:
            self._rewrite(poses)
        elif not self._stream.closed:
            self._stream.close()
        path_length = 0.0
        for previous, current in zip(poses, poses[1:]):
            path_length += math.dist(previous[1], current[1])
        duration = poses[-1][0] - poses[0][0] if len(poses) >= 2 else 0.0
        summary = {
            "messages": len(poses),
            "duration_sec": duration,
            "path_length_m": path_length,
            "final_position_m": poses[-1][1] if poses else None,
            "source": "trajectory" if self._path_poses is not None else "odometry",
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
