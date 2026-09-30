#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
import time

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Int32
from std_srvs.srv import Trigger, TriggerResponse

from c12_ros1.math_utils import (
    matrix_to_quaternion,
    pose_to_matrix,
    quaternion_to_matrix,
    transform_to_matrix,
)


class TagLocalizationNode:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.tf_lookup_timeout_sec = max(0.01, float(rospy.get_param("~tf_lookup_timeout_sec", 0.20)))
        self.max_tf_time_offset_sec = max(0.0, float(rospy.get_param("~max_tf_time_offset_sec", 0.30)))
        self.window_size = max(3, int(rospy.get_param("~position_window_size", 20)))
        self.min_samples = max(3, int(rospy.get_param("~min_lock_samples", 12)))
        self.std_limit = float(rospy.get_param("~lock_position_std_m", 0.10))
        self.outlier_reject_distance_m = max(0.0, float(rospy.get_param("~outlier_reject_distance_m", 0.50)))
        self.lock_once = bool(rospy.get_param("~lock_once", True))
        self.history = deque(maxlen=self.window_size)
        self.locked = False
        self.current_id = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose_world_pub = rospy.Publisher("/c12/tag/pose_world", PoseStamped, queue_size=10)
        self.filtered_pub = rospy.Publisher("/c12/tag/pose_world_filtered", PoseStamped, queue_size=10)
        self.locked_pub = rospy.Publisher("/c12/tag/locked_pose_world", PoseStamped, queue_size=1, latch=True)
        self.lock_pub = rospy.Publisher("/c12/tag/locked", Bool, queue_size=1, latch=True)
        self.std_pub = rospy.Publisher("/c12/tag/position_std_m", Float32, queue_size=10)
        rospy.Subscriber(rospy.get_param("~input_pose_topic", "/c12/tag/pose_camera"), PoseStamped, self.on_pose, queue_size=10)
        rospy.Subscriber("/c12/tag/id", Int32, self.on_id, queue_size=10)
        rospy.Service("/c12/tag/reset_lock", Trigger, self.on_reset)
        self._last_tf_warn = 0.0
        self.lock_pub.publish(Bool(data=False))

    def on_id(self, msg):
        tag_id = int(msg.data)
        if self.current_id is not None and tag_id != self.current_id and not self.locked:
            self.history.clear()
        self.current_id = tag_id

    def on_reset(self, _request):
        self.history.clear()
        self.locked = False
        self.lock_pub.publish(Bool(data=False))
        return TriggerResponse(success=True, message="tag lock cleared")

    def matrix_to_pose(self, matrix, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.pose.position.x = float(matrix[0, 3])
        msg.pose.position.y = float(matrix[1, 3])
        msg.pose.position.z = float(matrix[2, 3])
        q = matrix_to_quaternion(matrix[:3, :3])
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])
        return msg

    def on_pose(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame,
                msg.header.frame_id,
                stamp,
                rospy.Duration(self.tf_lookup_timeout_sec),
            )
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_tf_warn > 2.0:
                rospy.logwarn("TF %s <- %s unavailable: %s", self.world_frame, msg.header.frame_id, exc)
                self._last_tf_warn = now
            return

        if self.max_tf_time_offset_sec > 0.0 and tf_msg.header.stamp != rospy.Time(0):
            offset = abs((tf_msg.header.stamp - stamp).to_sec())
            if offset > self.max_tf_time_offset_sec:
                now = time.monotonic()
                if now - self._last_tf_warn > 2.0:
                    rospy.logwarn(
                        "TF %s <- %s time offset %.3fs exceeds %.3fs",
                        self.world_frame,
                        msg.header.frame_id,
                        offset,
                        self.max_tf_time_offset_sec,
                    )
                    self._last_tf_warn = now
                return

        world_tag = transform_to_matrix(tf_msg.transform) @ pose_to_matrix(msg.pose)
        world_msg = self.matrix_to_pose(world_tag, msg.header.stamp)
        self.pose_world_pub.publish(world_msg)
        if self.locked and self.lock_once:
            return

        if self.outlier_reject_distance_m > 0.0 and len(self.history) >= 3:
            old_positions = np.array([item[:3, 3] for item in self.history])
            median_old = np.median(old_positions, axis=0)
            distance = float(np.linalg.norm(world_tag[:3, 3] - median_old))
            if distance > self.outlier_reject_distance_m:
                rospy.logwarn_throttle(
                    1.0,
                    "Reject tag pose outlier: %.3f m from window median",
                    distance,
                )
                return

        self.history.append(world_tag)
        positions = np.array([item[:3, 3] for item in self.history])
        median = np.median(positions, axis=0)
        std = float(np.max(np.std(positions, axis=0))) if len(positions) > 1 else float("inf")

        rotations = []
        reference = None
        for item in self.history:
            q = matrix_to_quaternion(item[:3, :3])
            if reference is None:
                reference = q
            if np.dot(q, reference) < 0:
                q = -q
            rotations.append(q)
        qmean = np.mean(rotations, axis=0)
        qmean /= max(np.linalg.norm(qmean), 1e-12)
        filtered = np.eye(4)
        filtered[:3, 3] = median
        filtered[:3, :3] = quaternion_to_matrix(*qmean)
        filtered_msg = self.matrix_to_pose(filtered, msg.header.stamp)
        self.filtered_pub.publish(filtered_msg)
        self.std_pub.publish(Float32(data=float(std if np.isfinite(std) else 999.0)))

        if len(self.history) >= self.min_samples and std <= self.std_limit:
            self.locked = True
            self.locked_pub.publish(filtered_msg)
            self.lock_pub.publish(Bool(data=True))
            rospy.loginfo(
                "Tag locked at (%.3f, %.3f, %.3f), max std=%.3f m",
                median[0],
                median[1],
                median[2],
                std,
            )


def main():
    rospy.init_node("c12_tag_localization")
    TagLocalizationNode()
    rospy.spin()


if __name__ == "__main__":
    main()
