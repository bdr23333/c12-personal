#!/usr/bin/env python3
from __future__ import annotations

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main():
    rospy.init_node("c12_static_tf_from_params")
    parent = rospy.get_param("~parent_frame", "base_link")
    child = rospy.get_param("~child_frame", "c12_mount")
    xyz = rospy.get_param("~xyz", [0.0, 0.0, 0.0])
    rpy = rospy.get_param("~rpy", [0.0, 0.0, 0.0])
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError("~xyz and ~rpy must both contain 3 numeric values")

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(xyz[0])
    t.transform.translation.y = float(xyz[1])
    t.transform.translation.z = float(xyz[2])
    qx, qy, qz, qw = quaternion_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(t)
    rospy.loginfo(
        "Static TF %s -> %s xyz=%s rpy=%s",
        parent,
        child,
        [float(v) for v in xyz],
        [float(v) for v in rpy],
    )
    rospy.spin()


if __name__ == "__main__":
    main()
