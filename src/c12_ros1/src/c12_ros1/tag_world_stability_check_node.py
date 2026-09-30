#!/usr/bin/env python3
from __future__ import annotations

from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import String


class TagWorldStabilityCheckNode:
    def __init__(self):
        self.window_size = max(5, int(rospy.get_param("~window_size", 80)))
        self.warn_drift_m = float(rospy.get_param("~warn_drift_m", 0.20))
        self.min_camera_motion_m = float(rospy.get_param("~min_camera_motion_m", 0.05))
        self.samples = deque(maxlen=self.window_size)
        self.camera_samples = deque(maxlen=self.window_size)
        self.gimbal_samples = deque(maxlen=self.window_size)
        self.report_pub = rospy.Publisher("/c12/tag/world_stability_report", String, queue_size=1, latch=True)
        rospy.Subscriber("/c12/tag/pose_world_filtered", PoseStamped, self.on_world, queue_size=20)
        rospy.Subscriber("/c12/tag/pose_camera", PoseStamped, self.on_camera, queue_size=20)
        rospy.Subscriber("/c12/gimbal/angles_deg", Vector3Stamped, self.on_gimbal, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.report)

    @staticmethod
    def point_array(pose):
        p = pose.pose.position
        return np.array([p.x, p.y, p.z], dtype=float)

    def on_world(self, msg):
        self.samples.append(self.point_array(msg))

    def on_camera(self, msg):
        self.camera_samples.append(self.point_array(msg))

    def on_gimbal(self, msg):
        v = msg.vector
        self.gimbal_samples.append(np.array([v.x, v.y, v.z], dtype=float))

    @staticmethod
    def drift(values):
        if len(values) < 2:
            return 0.0, 0.0
        arr = np.asarray(values, dtype=float)
        span = float(np.max(np.linalg.norm(arr - np.median(arr, axis=0), axis=1)))
        std = float(np.max(np.std(arr, axis=0)))
        return span, std

    def report(self, _event):
        world_span, world_std = self.drift(self.samples)
        camera_span, _ = self.drift(self.camera_samples)
        gimbal_motion, _ = self.drift(self.gimbal_samples)
        status = "OK"
        if len(self.samples) >= 5 and world_span > self.warn_drift_m:
            status = "WARN_WORLD_DRIFT"
        if len(self.camera_samples) >= 5 and camera_span < self.min_camera_motion_m and len(self.gimbal_samples) >= 5 and gimbal_motion > 5.0:
            status = "WARN_CAMERA_POSE_NOT_CHANGING"
        text = (
            "status=%s samples=%d world_span=%.3fm world_std=%.3fm "
            "camera_span=%.3fm gimbal_span=%.2fdeg"
            % (status, len(self.samples), world_span, world_std, camera_span, gimbal_motion)
        )
        self.report_pub.publish(String(data=text))
        if status != "OK":
            rospy.logwarn_throttle(2.0, text)
        else:
            rospy.loginfo_throttle(5.0, text)


def main():
    rospy.init_node("c12_tag_world_stability_check")
    TagWorldStabilityCheckNode()
    rospy.spin()


if __name__ == "__main__":
    main()
