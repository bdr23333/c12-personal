#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


MAV_CMD_DO_SET_ACTUATOR = 187
SUPPORTED_FMU_CHANNELS = (7, 8)
FMU_CHANNEL_TO_ACTUATOR_SET = {7: 1, 8: 2}


class GripperReleaseNode:
    """Release a Pixhawk-connected gripper once the tag mission has arrived.

    The physical PWM setup is intentionally kept identical to the user's
    validated Pixhawk 6C CH7/CH8 deployment:

      MAV_CMD_DO_SET_ACTUATOR param1 -> FMU_CH7
      MAV_CMD_DO_SET_ACTUATOR param2 -> FMU_CH8

    This ROS wrapper sends the MAVLink command through MAVROS instead of opening
    the Pixhawk serial port directly, so it does not compete with MAVROS for the
    same FCU connection.
    """

    def __init__(self):
        self.enabled = bool(rospy.get_param("~enabled", False))
        self.auto_open_on_arrival = bool(rospy.get_param("~auto_open_on_arrival", True))
        self.require_fresh_arrival_edge = bool(rospy.get_param("~require_fresh_arrival_edge", True))
        self.require_armed_for_auto = bool(rospy.get_param("~require_armed_for_auto", True))

        self.arrival_topic = rospy.get_param("~arrival_topic", "/tag_mission/arrived")
        self.mission_state_topic = rospy.get_param("~mission_state_topic", "/tag_mission/state")
        self.odom_topic = rospy.get_param("~odom_topic", "/lio/robo/odom")
        self.mavros_state_topic = rospy.get_param("~mavros_state_topic", "/mavros/state")
        self.command_service = rospy.get_param("~command_service", "/mavros/cmd/command")

        self.fmu_channel = int(rospy.get_param("~fmu_channel", 7))
        self.output_min_pwm = int(rospy.get_param("~output_min_pwm", 1000))
        self.output_max_pwm = int(rospy.get_param("~output_max_pwm", 2000))
        self.safe_min_pwm = int(rospy.get_param("~safe_min_pwm", 1100))
        self.safe_max_pwm = int(rospy.get_param("~safe_max_pwm", 1900))
        self.neutral_pwm = int(rospy.get_param("~neutral_pwm", 1500))
        self.open_pwm = int(rospy.get_param("~open_pwm", 1200))
        self.close_pwm = int(rospy.get_param("~close_pwm", 1800))

        self.max_horizontal_speed_mps = float(rospy.get_param("~max_horizontal_speed_mps", 0.15))
        self.max_vertical_speed_mps = float(rospy.get_param("~max_vertical_speed_mps", 0.10))
        self.min_release_height_m = float(rospy.get_param("~min_release_height_m", 0.5))
        self.stable_sec = float(rospy.get_param("~stable_sec", 1.0))
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.5))

        self.command_repeats = int(rospy.get_param("~command_repeats", 3))
        self.command_repeat_delay_sec = float(rospy.get_param("~command_repeat_delay_sec", 0.15))
        self.service_wait_timeout_sec = float(rospy.get_param("~service_wait_timeout_sec", 2.0))
        self.auto_max_attempts = int(rospy.get_param("~auto_max_attempts", 3))
        self.auto_retry_sec = float(rospy.get_param("~auto_retry_sec", 1.0))

        self._validate_params()

        self.arrived = False
        self.mission_state = "UNKNOWN"
        self.odom = None
        self.last_odom_monotonic = 0.0
        self.mavros_state = None

        self.saw_not_arrived = False
        self.stable_since = None
        self.released_this_cycle = False
        self.auto_attempts = 0
        self.next_auto_attempt = 0.0
        self.last_error = ""

        self.state_pub = rospy.Publisher("/gripper/state", String, queue_size=1, latch=True)
        self.released_pub = rospy.Publisher("/gripper/released", Bool, queue_size=1, latch=True)

        rospy.Subscriber(self.arrival_topic, Bool, self.on_arrived, queue_size=10)
        rospy.Subscriber(self.mission_state_topic, String, self.on_mission_state, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=20)
        rospy.Subscriber(self.mavros_state_topic, State, self.on_mavros_state, queue_size=10)

        rospy.Service("/gripper/open", Trigger, self.on_open)
        rospy.Service("/gripper/close", Trigger, self.on_close)
        rospy.Service("/gripper/neutral", Trigger, self.on_neutral)
        rospy.Service("/gripper/reset_auto", Trigger, self.on_reset_auto)

        self.command_client = rospy.ServiceProxy(self.command_service, CommandLong, persistent=False)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)
        self.publish_status("DISABLED" if not self.enabled else "WAITING_FOR_MISSION")

        rospy.loginfo(
            "Gripper release node ready: enabled=%s, %s, open=%dus, close=%dus",
            self.enabled,
            self.output_label,
            self.open_pwm,
            self.close_pwm,
        )

    @property
    def output_label(self):
        return "FMU_CH{}".format(self.fmu_channel)

    @property
    def actuator_set(self):
        return FMU_CHANNEL_TO_ACTUATOR_SET[self.fmu_channel]

    def _validate_params(self):
        if self.fmu_channel not in SUPPORTED_FMU_CHANNELS:
            raise rospy.ROSInitException("~fmu_channel must be 7 or 8")
        if not self.output_min_pwm < self.neutral_pwm < self.output_max_pwm:
            raise rospy.ROSInitException("neutral_pwm must lie inside output_min_pwm..output_max_pwm")
        if not self.output_min_pwm <= self.safe_min_pwm < self.safe_max_pwm <= self.output_max_pwm:
            raise rospy.ROSInitException("safe PWM range must lie inside the physical output range")
        for name, value in (("open_pwm", self.open_pwm), ("close_pwm", self.close_pwm), ("neutral_pwm", self.neutral_pwm)):
            if not self.safe_min_pwm <= value <= self.safe_max_pwm:
                raise rospy.ROSInitException(
                    "{}={} is outside safe PWM range {}..{}".format(
                        name, value, self.safe_min_pwm, self.safe_max_pwm
                    )
                )
        if self.command_repeats < 1:
            raise rospy.ROSInitException("command_repeats must be >= 1")
        if self.command_repeat_delay_sec < 0.0:
            raise rospy.ROSInitException("command_repeat_delay_sec must be >= 0")
        if self.stable_sec < 0.0:
            raise rospy.ROSInitException("stable_sec must be >= 0")
        if self.auto_max_attempts < 1:
            raise rospy.ROSInitException("auto_max_attempts must be >= 1")

    def publish_status(self, state):
        self.state_pub.publish(String(data=state))
        self.released_pub.publish(Bool(data=self.released_this_cycle))

    def on_arrived(self, msg):
        new_value = bool(msg.data)
        if not new_value:
            self.saw_not_arrived = True
            self.stable_since = None
            self.released_this_cycle = False
            self.auto_attempts = 0
            self.next_auto_attempt = 0.0
            self.last_error = ""
        elif not self.arrived and new_value:
            self.stable_since = None
            self.auto_attempts = 0
            self.next_auto_attempt = 0.0
        self.arrived = new_value

    def on_mission_state(self, msg):
        self.mission_state = str(msg.data)

    def on_odom(self, msg):
        self.odom = msg
        self.last_odom_monotonic = time.monotonic()

    def on_mavros_state(self, msg):
        self.mavros_state = msg

    def pwm_to_normalized(self, pwm):
        pwm = float(pwm)
        center = float(self.neutral_pwm)
        low = float(self.output_min_pwm)
        high = float(self.output_max_pwm)
        if pwm <= center:
            return (pwm - center) / (center - low)
        return (pwm - center) / (high - center)

    def send_pwm(self, pwm):
        if not self.safe_min_pwm <= pwm <= self.safe_max_pwm:
            return False, "PWM {} outside safe range {}..{}".format(
                pwm, self.safe_min_pwm, self.safe_max_pwm
            )

        value = self.pwm_to_normalized(pwm)
        if not -1.0 <= value <= 1.0:
            return False, "normalized actuator value {:.3f} outside -1..1".format(value)

        params = [float("nan")] * 6 + [0.0]
        params[self.actuator_set - 1] = value

        try:
            rospy.wait_for_service(self.command_service, timeout=self.service_wait_timeout_sec)
        except rospy.ROSException as exc:
            return False, "MAVROS command service unavailable: {}".format(exc)

        for index in range(self.command_repeats):
            try:
                response = self.command_client(
                    broadcast=False,
                    command=MAV_CMD_DO_SET_ACTUATOR,
                    confirmation=0,
                    param1=params[0],
                    param2=params[1],
                    param3=params[2],
                    param4=params[3],
                    param5=params[4],
                    param6=params[5],
                    param7=params[6],
                )
            except rospy.ServiceException as exc:
                return False, "MAVROS CommandLong call failed: {}".format(exc)

            if not response.success:
                return False, "PX4 rejected MAV_CMD_DO_SET_ACTUATOR, result={}".format(response.result)

            if index + 1 < self.command_repeats and self.command_repeat_delay_sec > 0.0:
                rospy.sleep(self.command_repeat_delay_sec)

        return True, "{} PWM={}us actuator={:.3f}".format(self.output_label, pwm, value)

    def manual_command(self, label, pwm, mark_released=False):
        ok, detail = self.send_pwm(pwm)
        if ok:
            if mark_released:
                self.released_this_cycle = True
            self.publish_status("MANUAL_{}".format(label.upper()))
            rospy.logwarn("Gripper manual %s: %s", label, detail)
            return TriggerResponse(success=True, message=detail)
        self.last_error = detail
        self.publish_status("ERROR")
        rospy.logerr("Gripper manual %s failed: %s", label, detail)
        return TriggerResponse(success=False, message=detail)

    def on_open(self, _req):
        return self.manual_command("open", self.open_pwm, mark_released=True)

    def on_close(self, _req):
        return self.manual_command("close", self.close_pwm, mark_released=False)

    def on_neutral(self, _req):
        return self.manual_command("neutral", self.neutral_pwm, mark_released=False)

    def on_reset_auto(self, _req):
        self.stable_since = None
        self.released_this_cycle = False
        self.auto_attempts = 0
        self.next_auto_attempt = 0.0
        self.last_error = ""
        if not self.arrived:
            self.saw_not_arrived = True
        self.publish_status("WAITING_FOR_MISSION" if self.enabled else "DISABLED")
        return TriggerResponse(success=True, message="automatic release latch reset")

    def odom_is_fresh(self, now):
        return self.odom is not None and now - self.last_odom_monotonic <= self.odom_timeout_sec

    def motion_is_stable(self, now):
        if not self.odom_is_fresh(now):
            return False, "ODOM_STALE"

        twist = self.odom.twist.twist.linear
        horizontal = math.hypot(twist.x, twist.y)
        vertical = abs(twist.z)
        height = self.odom.pose.pose.position.z

        if horizontal > self.max_horizontal_speed_mps:
            return False, "MOVING_HORIZONTAL"
        if vertical > self.max_vertical_speed_mps:
            return False, "MOVING_VERTICAL"
        if height < self.min_release_height_m:
            return False, "BELOW_RELEASE_HEIGHT"
        return True, "STABLE"

    def auto_gate_ok(self):
        if not self.enabled or not self.auto_open_on_arrival:
            return False, "DISABLED"
        if not self.arrived or self.mission_state != "ARRIVED":
            return False, "WAITING_FOR_ARRIVAL"
        if self.require_fresh_arrival_edge and not self.saw_not_arrived:
            return False, "WAITING_FOR_FRESH_ARRIVAL_EDGE"
        if self.require_armed_for_auto:
            if self.mavros_state is None or not self.mavros_state.connected:
                return False, "MAVROS_NOT_CONNECTED"
            if not self.mavros_state.armed:
                return False, "VEHICLE_NOT_ARMED"
        return True, "READY"

    def tick(self, _event):
        now = time.monotonic()

        if self.released_this_cycle:
            self.publish_status("RELEASED")
            return

        gate_ok, gate_state = self.auto_gate_ok()
        if not gate_ok:
            self.stable_since = None
            self.publish_status(gate_state)
            return

        stable, stable_state = self.motion_is_stable(now)
        if not stable:
            self.stable_since = None
            self.publish_status(stable_state)
            return

        if self.stable_since is None:
            self.stable_since = now
            self.publish_status("STABILIZING")
            return

        if now - self.stable_since < self.stable_sec:
            self.publish_status("STABILIZING")
            return

        if self.auto_attempts >= self.auto_max_attempts:
            self.publish_status("AUTO_FAILED")
            return
        if now < self.next_auto_attempt:
            self.publish_status("AUTO_RETRY_WAIT")
            return

        self.auto_attempts += 1
        self.publish_status("RELEASING")
        rospy.logwarn(
            "Automatic gripper release attempt %d/%d after mission ARRIVED and %.2fs stable hover",
            self.auto_attempts,
            self.auto_max_attempts,
            self.stable_sec,
        )
        ok, detail = self.send_pwm(self.open_pwm)
        if ok:
            self.released_this_cycle = True
            self.last_error = ""
            self.publish_status("RELEASED")
            rospy.logwarn("Automatic gripper release succeeded: %s", detail)
            return

        self.last_error = detail
        self.next_auto_attempt = now + self.auto_retry_sec
        rospy.logerr("Automatic gripper release failed: %s", detail)
        if self.auto_attempts >= self.auto_max_attempts:
            self.publish_status("AUTO_FAILED")
        else:
            self.publish_status("AUTO_RETRY_WAIT")


def main():
    rospy.init_node("c12_gripper_release")
    GripperReleaseNode()
    rospy.spin()


if __name__ == "__main__":
    main()
