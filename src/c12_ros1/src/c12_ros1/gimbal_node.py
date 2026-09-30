#!/usr/bin/env python3
from __future__ import annotations

import math
import socket
import threading
import time

import rospy
from geometry_msgs.msg import Vector3, Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

from c12_ros1.protocol import (
    add_checksum,
    encode_s16_angle_deg,
    encode_s8_speed_deg_s,
    parse_full_gac,
)


class C12GimbalNode:
    def __init__(self) -> None:
        self.camera_ip = rospy.get_param("~camera_ip", "192.168.1.91")
        self.camera_port = int(rospy.get_param("~camera_port", 5000))
        self.local_ip = rospy.get_param("~local_ip", "0.0.0.0")
        self.local_port = int(rospy.get_param("~local_port", 5000))
        self.attitude_rate_hz = max(1, min(100, int(rospy.get_param("~attitude_rate_hz", 10))))
        self.ptz_speed_deg_s = max(0.1, min(12.7, abs(float(rospy.get_param("~ptz_speed_deg_s", 3.0)))))
        self.speed_refresh_hz = max(1.0, float(rospy.get_param("~speed_refresh_hz", 20.0)))
        self.motion_timeout_sec = max(0.0, float(rospy.get_param("~motion_timeout_sec", 0.0)))
        self.frame_id = rospy.get_param("~frame_id", "c12_gimbal_base")
        self.move_to_startup_pose = bool(rospy.get_param("~move_to_startup_pose", True))
        self.startup_yaw_deg = float(rospy.get_param("~startup_yaw_deg", 90.0))
        self.startup_pitch_deg = float(rospy.get_param("~startup_pitch_deg", -45.0))
        self.startup_speed_deg_s = float(rospy.get_param("~startup_speed_deg_s", 3.0))
        self.startup_delay_sec = max(0.0, float(rospy.get_param("~startup_delay_sec", 2.0)))

        self.angle_pub = rospy.Publisher("/c12/gimbal/angles_deg", Vector3Stamped, queue_size=10)
        self.joint_pub = rospy.Publisher("/c12/gimbal/joint_states", JointState, queue_size=10)
        self.raw_rx_pub = rospy.Publisher("/c12/gimbal/raw_rx", String, queue_size=10)

        rospy.Subscriber("/c12/gimbal/cmd_angle_deg", Vector3, self.on_angle, queue_size=10)
        rospy.Subscriber("/c12/gimbal/cmd_speed_deg_s", Vector3, self.on_speed, queue_size=10)
        rospy.Subscriber("/c12/gimbal/cmd_ptz", String, self.on_direction, queue_size=10)
        rospy.Subscriber("/c12/gimbal/cmd_center", Empty, self.on_center, queue_size=10)

        self._socket_lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.local_ip, self.local_port))
        self.sock.settimeout(0.2)

        self._stop_event = threading.Event()
        self._active_yaw_speed = 0.0
        self._active_pitch_speed = 0.0
        self._speed_active = False
        self._last_motion_command_monotonic = time.monotonic()

        self._rx_thread = threading.Thread(target=self.receive_loop)
        self._rx_thread.daemon = True
        self._rx_thread.start()
        self._speed_timer = rospy.Timer(rospy.Duration(1.0 / self.speed_refresh_hz), self.refresh_speed)

        self.send_body("#TPUG2wGAA%02X" % self.attitude_rate_hz)
        rospy.loginfo(
            "C12 UDP %s:%d; listen %s:%d; speed refresh %.1f Hz",
            self.camera_ip,
            self.camera_port,
            self.local_ip,
            self.local_port,
            self.speed_refresh_hz,
        )
        self._startup_timer = None
        if self.move_to_startup_pose:
            self._startup_timer = rospy.Timer(
                rospy.Duration(max(0.01, self.startup_delay_sec)),
                self.move_to_startup_pose_once,
                oneshot=True,
            )

    def send_body(self, body: str) -> str:
        packet = add_checksum(body)
        with self._socket_lock:
            self.sock.sendto(packet.encode("ascii"), (self.camera_ip, self.camera_port))
        return packet

    def send_speed(self, yaw_speed: float, pitch_speed: float) -> str:
        return self.send_body(
            "#TPUG4wGSM"
            + encode_s8_speed_deg_s(yaw_speed)
            + encode_s8_speed_deg_s(pitch_speed)
        )

    def set_speed(self, yaw_speed: float, pitch_speed: float) -> str:
        self._active_yaw_speed = max(-12.7, min(12.7, float(yaw_speed)))
        self._active_pitch_speed = max(-12.7, min(12.7, float(pitch_speed)))
        self._speed_active = abs(self._active_yaw_speed) > 1e-6 or abs(self._active_pitch_speed) > 1e-6
        self._last_motion_command_monotonic = time.monotonic()
        return self.send_speed(self._active_yaw_speed, self._active_pitch_speed)

    def stop_speed(self) -> str:
        self._active_yaw_speed = 0.0
        self._active_pitch_speed = 0.0
        self._speed_active = False
        return self.send_speed(0.0, 0.0)

    def refresh_speed(self, _event) -> None:
        if not self._speed_active or self._stop_event.is_set():
            return
        if self.motion_timeout_sec > 0.0 and time.monotonic() - self._last_motion_command_monotonic > self.motion_timeout_sec:
            rospy.logwarn("Motion timeout: sent stop %s", self.stop_speed())
            return
        self.send_speed(self._active_yaw_speed, self._active_pitch_speed)

    def send_center(self) -> str:
        self.stop_speed()
        speed_hex = "%02X" % int(round(min(9.9, self.ptz_speed_deg_s) * 10.0))
        return self.send_body(
            "#TPUGCwGAM"
            + encode_s16_angle_deg(0.0)
            + speed_hex
            + encode_s16_angle_deg(0.0)
            + speed_hex
        )

    def send_angle(self, yaw_deg: float, pitch_deg: float, speed_deg_s: float) -> str:
        self.stop_speed()
        yaw = max(-90.0, min(90.0, float(yaw_deg)))
        pitch = max(-90.0, min(10.0, float(pitch_deg)))
        speed = max(0.1, min(9.9, abs(float(speed_deg_s))))
        speed_hex = "%02X" % int(round(speed * 10.0))
        payload = encode_s16_angle_deg(yaw) + speed_hex + encode_s16_angle_deg(pitch) + speed_hex
        return self.send_body("#TPUGCwGAM" + payload)

    def move_to_startup_pose_once(self, _event) -> None:
        self._startup_timer = None
        packet = self.send_angle(self.startup_yaw_deg, self.startup_pitch_deg, self.startup_speed_deg_s)
        rospy.loginfo(
            "startup pose yaw=%.2f, pitch=%.2f, speed=%.1f: %s",
            self.startup_yaw_deg,
            self.startup_pitch_deg,
            self.startup_speed_deg_s,
            packet,
        )

    def cancel_startup_pose(self) -> None:
        if self._startup_timer is not None:
            self._startup_timer.shutdown()
            self._startup_timer = None
            rospy.loginfo("startup pose canceled by active gimbal command")

    def on_center(self, _msg: Empty) -> None:
        self.cancel_startup_pose()
        rospy.loginfo("Center: %s", self.send_center())

    def on_direction(self, msg: String) -> None:
        self.cancel_startup_pose()
        command = msg.data.strip().lower()
        speed = self.ptz_speed_deg_s
        if command == "left":
            packet = self.set_speed(-speed, 0.0)
        elif command == "right":
            packet = self.set_speed(speed, 0.0)
        elif command == "up":
            packet = self.set_speed(0.0, speed)
        elif command == "down":
            packet = self.set_speed(0.0, -speed)
        elif command == "stop":
            packet = self.stop_speed()
        elif command in ("center", "home"):
            packet = self.send_center()
        else:
            rospy.logerr("Use left/right/up/down/stop/center")
            return
        rospy.loginfo("%s: %s", command, packet)

    def on_speed(self, msg: Vector3) -> None:
        if abs(msg.x) >= 1e-6 or abs(msg.y) >= 1e-6:
            self.cancel_startup_pose()
        packet = self.stop_speed() if abs(msg.x) < 1e-6 and abs(msg.y) < 1e-6 else self.set_speed(msg.x, msg.y)
        rospy.loginfo("speed yaw=%.2f, pitch=%.2f: %s", msg.x, msg.y, packet)

    def on_angle(self, msg: Vector3) -> None:
        self.cancel_startup_pose()
        yaw = max(-90.0, min(90.0, float(msg.x)))
        pitch = max(-90.0, min(10.0, float(msg.y)))
        speed = max(0.1, min(9.9, abs(float(msg.z))))
        packet = self.send_angle(yaw, pitch, speed)
        rospy.loginfo("angle yaw=%.2f, pitch=%.2f, speed=%.1f: %s", yaw, pitch, speed, packet)

    def receive_loop(self) -> None:
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            try:
                data, address = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            text = data.decode("ascii", errors="replace").strip("\x00\r\n ")
            if not text:
                continue
            self.raw_rx_pub.publish(String(data="%s:%d %s" % (address[0], address[1], text)))
            angles = parse_full_gac(text)
            if angles is None:
                continue

            stamp = rospy.Time.now()
            angle_msg = Vector3Stamped()
            angle_msg.header.stamp = stamp
            angle_msg.header.frame_id = self.frame_id
            angle_msg.vector.x = angles.yaw_deg
            angle_msg.vector.y = angles.pitch_deg
            angle_msg.vector.z = angles.roll_deg
            self.angle_pub.publish(angle_msg)

            joint_msg = JointState()
            joint_msg.header.stamp = stamp
            joint_msg.name = ["c12_yaw_joint", "c12_pitch_joint", "c12_roll_joint"]
            joint_msg.position = [math.radians(angles.yaw_deg), math.radians(angles.pitch_deg), math.radians(angles.roll_deg)]
            self.joint_pub.publish(joint_msg)

    def shutdown(self) -> None:
        if self._stop_event.is_set():
            return
        try:
            self.stop_speed()
            self.send_body("#TPUG2wGAA00")
        except OSError:
            pass
        self._stop_event.set()
        try:
            self.sock.close()
        except OSError:
            pass
        if self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)


def main() -> None:
    rospy.init_node("c12_gimbal")
    node = C12GimbalNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
