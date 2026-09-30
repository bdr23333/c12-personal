#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomTfBridge:
    def __init__(self):
        self.world_frame_override = rospy.get_param("~world_frame_override", "")
        self.base_frame_override = rospy.get_param("~base_frame_override", "")
        self.br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(rospy.get_param("~odom_topic", "/lio/robo/odom"), Odometry, self.cb, queue_size=10)

    def cb(self, msg):
        t = TransformStamped()
        t.header = msg.header
        if self.world_frame_override:
            t.header.frame_id = self.world_frame_override
        t.child_frame_id = self.base_frame_override or msg.child_frame_id or "base_link"
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q
        self.br.sendTransform(t)


def main():
    rospy.init_node("c12_odom_tf_bridge")
    OdomTfBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
