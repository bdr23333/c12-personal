#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


class TagMissionNode:
    STATES = (
        "IDLE",
        "WAIT_HOVER",
        "SEARCH",
        "TRACK",
        "ACQUIRE",
        "LOCALIZE",
        "LOCK_TARGET",
        "APPROACH",
        "FINE_ALIGN",
        "ARRIVED",
        "TARGET_LOST",
        "ABORTED",
    )

    def __init__(self):
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.odom_topic = rospy.get_param("~odom_topic", "/lio/robo/odom")
        self.planner_goal_topic = rospy.get_param("~planner_goal_topic", "/remote/goal")
        self.hover_height_m = float(rospy.get_param("~hover_height_m", 2.0))
        self.min_takeoff_height_m = float(rospy.get_param("~min_takeoff_height_m", 0.5))
        self.hover_stable_sec = float(rospy.get_param("~hover_stable_sec", 2.0))
        self.hover_max_linear_speed_mps = float(rospy.get_param("~hover_max_linear_speed_mps", 0.15))
        self.hover_max_vertical_speed_mps = float(rospy.get_param("~hover_max_vertical_speed_mps", 0.10))
        self.hover_max_roll_pitch_deg = float(rospy.get_param("~hover_max_roll_pitch_deg", 8.0))
        self.target_lost_sec = float(rospy.get_param("~target_lost_sec", 2.0))
        self.lost_hold_after_lock = bool(rospy.get_param("~lost_hold_after_lock", False))
        self.fine_align_distance_m = float(rospy.get_param("~fine_align_distance_m", 0.5))
        self.horizontal_arrival_m = float(rospy.get_param("~horizontal_arrival_m", 0.2))
        self.vertical_arrival_m = float(rospy.get_param("~vertical_arrival_m", 0.15))
        self.gimbal_home_yaw_deg = float(rospy.get_param("~gimbal_home_yaw_deg", 0.0))
        self.gimbal_home_pitch_deg = float(rospy.get_param("~gimbal_home_pitch_deg", -90.0))
        self.gimbal_arrival_deg = float(rospy.get_param("~gimbal_arrival_deg", 4.0))
        self.use_gimbal_arrival = bool(rospy.get_param("~use_gimbal_arrival", True))
        self.ignore_yaw_near_nadir = bool(rospy.get_param("~ignore_yaw_near_nadir", True))
        self.nadir_pitch_tolerance_deg = float(rospy.get_param("~nadir_pitch_tolerance_deg", 8.0))
        self.image_width = int(rospy.get_param("~image_width", 1280))
        self.image_height = int(rospy.get_param("~image_height", 720))
        self.pixel_arrival_px = float(rospy.get_param("~pixel_arrival_px", 45.0))
        self.use_pixel_arrival = bool(rospy.get_param("~use_pixel_arrival", True))
        self.arrival_stable_sec = float(rospy.get_param("~arrival_stable_sec", 1.0))
        self.goal_publish_hz = float(rospy.get_param("~goal_publish_hz", 2.0))

        self.state = "WAIT_HOVER" if self.auto_start else "IDLE"
        self.visible = False
        self.last_seen = 0.0
        self.filtered_pose = None
        self.last_filtered = 0.0
        self.locked_pose = None
        self.odom = None
        self.gimbal = None
        self.center = None
        self.last_center = 0.0
        self.goal = None
        self.arrival_since = None
        self.hover_since = None
        self.last_goal_publish = 0.0

        self.state_pub = rospy.Publisher("/tag_mission/state", String, queue_size=1, latch=True)
        self.arrived_pub = rospy.Publisher("/tag_mission/arrived", Bool, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher(self.planner_goal_topic, PoseStamped, queue_size=10)
        rospy.Subscriber("/c12/tag/visible", Bool, self.on_visible, queue_size=10)
        rospy.Subscriber("/c12/tag/pose_world_filtered", PoseStamped, self.on_filtered, queue_size=10)
        rospy.Subscriber("/c12/tag/locked_pose_world", PoseStamped, self.on_locked, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=10)
        rospy.Subscriber("/c12/gimbal/angles_deg", Vector3Stamped, self.on_gimbal, queue_size=10)
        rospy.Subscriber("/c12/tag/center_pixel", PointStamped, self.on_center, queue_size=10)
        rospy.Service("/tag_mission/start", Trigger, self.on_start)
        rospy.Service("/tag_mission/reset", Trigger, self.on_reset)
        rospy.Service("/tag_mission/abort", Trigger, self.on_abort)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))
        self.arrived_pub.publish(Bool(data=self.state == "ARRIVED"))

    def set_state(self, state):
        if state not in self.STATES:
            rospy.logerr("Invalid mission state: %s", state)
            return
        if state != self.state:
            rospy.loginfo("Mission state: %s -> %s", self.state, state)
            self.state = state
            self.arrival_since = None
            if state != "WAIT_HOVER":
                self.hover_since = None
            self.publish_state()

    def on_visible(self, msg):
        self.visible = bool(msg.data)
        if self.visible:
            self.last_seen = time.monotonic()

    def on_filtered(self, msg):
        self.filtered_pose = msg
        self.last_filtered = time.monotonic()

    def on_locked(self, msg):
        self.locked_pose = msg
        self.goal = self.make_goal(msg)

    def on_odom(self, msg):
        self.odom = msg

    def on_gimbal(self, msg):
        self.gimbal = msg

    def on_center(self, msg):
        self.center = msg
        self.last_center = time.monotonic()

    def on_start(self, _req):
        self.set_state("WAIT_HOVER")
        return TriggerResponse(success=True, message="mission waiting for stable hover")

    def on_reset(self, _req):
        self.filtered_pose = None
        self.locked_pose = None
        self.goal = None
        self.arrival_since = None
        self.hover_since = None
        self.set_state("IDLE")
        return TriggerResponse(success=True, message="mission reset; call /c12/tag/reset_lock before a new target")

    def on_abort(self, _req):
        self.publish_hold()
        self.set_state("ABORTED")
        return TriggerResponse(success=True, message="mission aborted")

    def make_goal(self, tag):
        goal = PoseStamped()
        goal.header.frame_id = self.world_frame
        goal.pose.position.x = tag.pose.position.x
        goal.pose.position.y = tag.pose.position.y
        goal.pose.position.z = tag.pose.position.z + self.hover_height_m
        if self.odom is not None:
            goal.pose.orientation = self.odom.pose.pose.orientation
        else:
            goal.pose.orientation.w = 1.0
        return goal

    def publish_goal(self, goal):
        goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(goal)
        self.last_goal_publish = time.monotonic()

    def publish_hold(self):
        if self.odom is None:
            return
        hold = PoseStamped()
        hold.header.frame_id = self.world_frame
        hold.pose = self.odom.pose.pose
        self.publish_goal(hold)

    @staticmethod
    def quaternion_to_roll_pitch(q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        return roll, pitch

    def hover_ok(self):
        if self.odom is None:
            return False
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist
        horizontal_speed = math.hypot(twist.linear.x, twist.linear.y)
        vertical_speed = abs(twist.linear.z)
        roll, pitch = self.quaternion_to_roll_pitch(pose.orientation)
        attitude_deg = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
        return (
            pose.position.z >= self.min_takeoff_height_m
            and horizontal_speed <= self.hover_max_linear_speed_mps
            and vertical_speed <= self.hover_max_vertical_speed_mps
            and attitude_deg <= self.hover_max_roll_pitch_deg
        )

    def errors(self):
        if self.odom is None or self.goal is None:
            return None
        p = self.odom.pose.pose.position
        g = self.goal.pose.position
        return math.hypot(p.x - g.x, p.y - g.y), abs(p.z - g.z)

    def arrival_ok(self):
        errors = self.errors()
        if errors is None:
            return False
        horizontal, vertical = errors
        ok = horizontal < self.horizontal_arrival_m and vertical < self.vertical_arrival_m
        if self.use_gimbal_arrival:
            if self.gimbal is None:
                return False
            yaw_error = abs(self.gimbal.vector.x - self.gimbal_home_yaw_deg)
            pitch_error = abs(self.gimbal.vector.y - self.gimbal_home_pitch_deg)
            pitch_ok = pitch_error < self.gimbal_arrival_deg
            near_nadir = self.ignore_yaw_near_nadir and pitch_error < self.nadir_pitch_tolerance_deg
            yaw_ok = near_nadir or yaw_error < self.gimbal_arrival_deg
            ok = ok and pitch_ok and yaw_ok
        if self.use_pixel_arrival:
            if self.center is None or time.monotonic() - self.last_center > 0.5:
                return False
            ex = self.center.point.x - self.image_width / 2.0
            ey = self.center.point.y - self.image_height / 2.0
            ok = ok and math.hypot(ex, ey) < self.pixel_arrival_px
        return ok

    def publish_goal_periodic(self, interval):
        if self.goal is not None and time.monotonic() - self.last_goal_publish > interval:
            self.publish_goal(self.goal)

    def target_lost(self, now):
        return now - self.last_seen > self.target_lost_sec

    def tick(self, _event):
        now = time.monotonic()
        if self.state in ("IDLE", "ABORTED"):
            return

        if self.state == "WAIT_HOVER":
            if self.hover_ok():
                if self.hover_since is None:
                    self.hover_since = now
                elif now - self.hover_since >= self.hover_stable_sec:
                    self.set_state("SEARCH")
            else:
                self.hover_since = None
            return

        if self.state == "SEARCH":
            if self.visible:
                self.set_state("TRACK")
            return

        if self.state == "TRACK":
            if self.target_lost(now):
                self.set_state("SEARCH")
            elif self.filtered_pose is not None and now - self.last_filtered < 1.0:
                self.set_state("ACQUIRE")
            return

        if self.state == "ACQUIRE":
            if self.locked_pose is not None:
                self.set_state("LOCK_TARGET")
            elif self.filtered_pose is not None and now - self.last_filtered < 1.0:
                self.set_state("LOCALIZE")
            elif self.target_lost(now):
                self.set_state("SEARCH")
            return

        if self.state == "LOCALIZE":
            if self.locked_pose is not None:
                self.set_state("LOCK_TARGET")
            elif self.target_lost(now):
                self.set_state("SEARCH")
            return

        if self.state == "LOCK_TARGET":
            if self.locked_pose is None:
                self.set_state("SEARCH")
                return
            self.goal = self.make_goal(self.locked_pose)
            self.publish_goal(self.goal)
            self.set_state("APPROACH")
            return

        if self.state in ("APPROACH", "FINE_ALIGN"):
            if self.target_lost(now) and (self.locked_pose is None or self.lost_hold_after_lock):
                self.publish_hold()
                self.set_state("TARGET_LOST")
                return
            self.publish_goal_periodic(1.0 / max(self.goal_publish_hz, 0.1))
            errors = self.errors()
            if errors and errors[0] < self.fine_align_distance_m and self.state == "APPROACH":
                self.set_state("FINE_ALIGN")
            if self.arrival_ok():
                if self.arrival_since is None:
                    self.arrival_since = now
                elif now - self.arrival_since >= self.arrival_stable_sec:
                    self.publish_hold()
                    self.set_state("ARRIVED")
            else:
                self.arrival_since = None
            return

        if self.state == "TARGET_LOST":
            self.publish_hold()
            if self.visible:
                self.set_state("TRACK")
        elif self.state == "ARRIVED":
            if now - self.last_goal_publish > 0.5:
                self.publish_hold()


def main():
    rospy.init_node("c12_tag_mission")
    TagMissionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
