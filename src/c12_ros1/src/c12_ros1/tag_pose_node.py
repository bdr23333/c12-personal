#!/usr/bin/env python3
from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3, Vector3Stamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import SetBool, SetBoolResponse

from c12_ros1.ros_compat import RospyLogger
from c12_ros1.rtsp_reader import LatestFrameReader


def matrix_to_quaternion(matrix):
    m = matrix
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    q /= max(np.linalg.norm(q), 1e-12)
    return tuple(float(v) for v in q)


def polygon_area(corners):
    x = corners[:, 0]
    y = corners[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))))


class C12TagPoseNode:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "c12_visible_optical_frame")
        self.detector_type = rospy.get_param("~detector_type", "apriltag").lower()
        self.tag_size = float(rospy.get_param("~tag_size_m", 0.20))
        self.target_id = int(rospy.get_param("~target_id", -1))
        self.target_code = rospy.get_param("~target_code", "")
        self.use_undistort = bool(rospy.get_param("~use_undistort", True))
        self.publish_debug = bool(rospy.get_param("~publish_debug_image", True))
        self.jpeg_quality = int(rospy.get_param("~debug_jpeg_quality", 75))
        self.enable_tracking = bool(rospy.get_param("~enable_gimbal_tracking", True))
        self.safe_yaw = float(rospy.get_param("~safe_yaw_deg", 90.0))
        self.safe_pitch = float(rospy.get_param("~safe_pitch_deg", -45.0))
        self.safe_pose_speed = abs(float(rospy.get_param("~safe_pose_speed_deg_s", 3.0)))
        self.safe_pose_repeat_sec = max(0.2, float(rospy.get_param("~safe_pose_repeat_sec", 2.0)))
        self.deadband_px = float(rospy.get_param("~deadband_px", 35.0))
        self.yaw_kp = float(rospy.get_param("~yaw_kp", 0.006))
        self.pitch_kp = float(rospy.get_param("~pitch_kp", 0.006))
        self.yaw_sign = float(rospy.get_param("~yaw_sign", 1.0))
        self.pitch_sign = float(rospy.get_param("~pitch_sign", -1.0))
        self.max_speed = abs(float(rospy.get_param("~max_gimbal_speed_deg_s", 3.0)))
        self.control_publish_hz = max(1.0, float(rospy.get_param("~control_publish_hz", 20.0)))
        self.stale_timeout = max(0.2, float(rospy.get_param("~vision_stale_timeout_sec", 1.0)))
        self.enable_search = bool(rospy.get_param("~enable_gimbal_search", False))
        self.search_startup_yaw = float(rospy.get_param("~search_startup_yaw_deg", 90.0))
        self.search_startup_pitch = float(rospy.get_param("~search_startup_pitch_deg", -45.0))
        self.search_up_pitch = float(rospy.get_param("~search_up_pitch_deg", 0.0))
        self.search_down_pitch = float(rospy.get_param("~search_down_pitch_deg", -90.0))
        self.search_sweep_pitch = float(rospy.get_param("~search_sweep_pitch_deg", -45.0))
        self.search_left_yaw = float(rospy.get_param("~search_left_yaw_deg", -90.0))
        self.search_right_yaw = float(rospy.get_param("~search_right_yaw_deg", 90.0))
        self.search_speed = abs(float(rospy.get_param("~search_speed_deg_s", 1.0)))
        self.search_sweep_step = max(1.0, abs(float(rospy.get_param("~search_sweep_step_deg", 30.0))))
        self.search_sweep_pause = max(0.0, float(rospy.get_param("~search_sweep_pause_sec", 1.0)))
        self.search_pitch_step = max(1.0, abs(float(rospy.get_param("~search_pitch_step_deg", 20.0))))
        self.search_pitch_pause = max(0.0, float(rospy.get_param("~search_pitch_pause_sec", 1.0)))
        self.search_stable_sec = max(0.1, float(rospy.get_param("~search_stable_sec", 1.5)))
        self.search_angle_tolerance = max(0.1, float(rospy.get_param("~search_angle_tolerance_deg", 3.0)))
        self.search_arrival_timeout = max(1.0, float(rospy.get_param("~search_arrival_timeout_sec", 45.0)))

        self.camera_matrix, self.dist_coeffs, self.image_width, self.image_height = self._load_camera_info()
        self.fx = float(self.camera_matrix[0, 0])
        self.fy = float(self.camera_matrix[1, 1])
        self.cx = float(self.camera_matrix[0, 2])
        self.cy = float(self.camera_matrix[1, 2])

        self.apriltag_detector = None
        self.aruco_detector = None
        self.qr_detector = None
        self._init_detector()

        self.visible_pub = rospy.Publisher("/c12/tag/visible", Bool, queue_size=10)
        self.id_pub = rospy.Publisher("/c12/tag/id", Int32, queue_size=10)
        self.code_pub = rospy.Publisher("/c12/tag/code", String, queue_size=10)
        self.center_pub = rospy.Publisher("/c12/tag/center_pixel", PointStamped, queue_size=10)
        self.pose_pub = rospy.Publisher("/c12/tag/pose_camera", PoseStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/c12/tag/pose_error", Float32, queue_size=10)
        self.speed_pub = rospy.Publisher("/c12/gimbal/cmd_speed_deg_s", Vector3, queue_size=10)
        self.angle_cmd_pub = rospy.Publisher("/c12/gimbal/cmd_angle_deg", Vector3, queue_size=10)
        self.control_state_pub = rospy.Publisher("/c12/gimbal/auto_control_state", String, queue_size=1, latch=True)
        self.debug_pub = rospy.Publisher("/c12/tag/debug/compressed", CompressedImage, queue_size=1) if self.publish_debug else None
        rospy.Subscriber("/c12/gimbal/angles_deg", Vector3Stamped, self._on_gimbal_angles, queue_size=10)
        rospy.Subscriber("/tag_mission/state", String, self._on_mission_state, queue_size=1)
        rospy.Subscriber("/c12/gimbal/manual_override", Bool, self._on_manual_override, queue_size=1)
        rospy.Service("/c12/gimbal/set_manual_override", SetBool, self._on_set_manual_override)

        self._control_lock = threading.Lock()
        self._desired_yaw = 0.0
        self._desired_pitch = 0.0
        self._last_vision = 0.0
        self._tracking_active = False
        self._stale_stop_sent = True
        self._current_gimbal = None
        self._mission_state = "IDLE"
        self._manual_override = False
        self._safe_pose_sent = False
        self._last_safe_pose = 0.0
        self._stop_sent_for_state = False
        self._search_state = "wait_initial"
        self._search_goal = None
        self._search_goal_since = 0.0
        self._search_arrived_since = None
        self._tag_visible = False
        self._control_stop = threading.Event()
        self._control_thread = threading.Thread(target=self._control_loop)
        self._control_thread.daemon = True
        self._control_thread.start()
        self._search_timer = rospy.Timer(rospy.Duration(0.1), self._search_loop)

        url = rospy.get_param("~rtsp_url", "rtsp://192.168.1.91:554/stream=1")
        backend = rospy.get_param("~backend", "ffmpeg_cli")
        transport = rospy.get_param("~transport", "tcp")
        rospy.loginfo(
            "Opening tag RTSP: %s; backend=%s; transport=%s; detector=%s; size=%.3f m",
            url,
            backend,
            transport,
            self.detector_type,
            self.tag_size,
        )
        self.reader = LatestFrameReader(
            url=url,
            backend=backend,
            transport=transport,
            latency_ms=int(rospy.get_param("~latency_ms", 0)),
            logger=RospyLogger(),
            time_source_ns=lambda: rospy.Time.now().to_nsec(),
        )
        self.reader.start()
        self.last_sequence = -1
        hz = max(0.5, float(rospy.get_param("~max_detection_hz", 15.0)))
        self.timer = rospy.Timer(rospy.Duration(1.0 / hz), self.process_latest)

    def _publish_control_state(self, mode):
        suffix = "MANUAL_OVERRIDE" if self._manual_override else mode
        self.control_state_pub.publish(String(data=suffix))

    def _load_camera_info(self):
        path = rospy.get_param("~camera_info_file", "")
        if not path:
            raise RuntimeError("camera_info_file is required")
        data = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8"))
        k = np.asarray(data["camera_matrix"]["data"], dtype=float).reshape(3, 3)
        d = np.asarray(data.get("distortion_coefficients", {}).get("data", [0, 0, 0, 0, 0]), dtype=float)
        width = int(data.get("image_width", round(k[0, 2] * 2)))
        height = int(data.get("image_height", round(k[1, 2] * 2)))
        if data.get("calibration_is_placeholder", False):
            rospy.logwarn("Camera calibration is marked as placeholder; metric pose will be approximate")
        return k, d, width, height

    def _init_detector(self):
        if self.detector_type == "apriltag":
            try:
                from pupil_apriltags import Detector
            except ImportError as exc:
                raise RuntimeError("pupil-apriltags is missing. Install it in the ROS1 Python environment") from exc
            self.apriltag_detector = Detector(
                families=rospy.get_param("~tag_family", "tag36h11"),
                nthreads=2,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0,
            )
        elif self.detector_type == "aruco":
            if not hasattr(cv2, "aruco"):
                raise RuntimeError("OpenCV ArUco module is unavailable. Install opencv-contrib-python.")
            name = rospy.get_param("~aruco_dictionary", "DICT_ARUCO_ORIGINAL")
            if not hasattr(cv2.aruco, name):
                raise ValueError("Unknown ArUco dictionary: %s" % name)
            dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
            parameters = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(dictionary, parameters) if hasattr(cv2.aruco, "ArucoDetector") else (dictionary, parameters)
        elif self.detector_type == "qrcode":
            self.qr_detector = cv2.QRCodeDetector()
        else:
            raise ValueError("detector_type must be apriltag, aruco, or qrcode")

    def _set_desired(self, yaw, pitch):
        with self._control_lock:
            self._desired_yaw = float(yaw)
            self._desired_pitch = float(pitch)
            self._last_vision = time.monotonic()
            self._tracking_active = True
            self._stale_stop_sent = False

    def _stop_tracking(self):
        with self._control_lock:
            self._desired_yaw = 0.0
            self._desired_pitch = 0.0
            self._tracking_active = False
            self._stale_stop_sent = True
        self.speed_pub.publish(Vector3())

    def _on_mission_state(self, msg):
        state = msg.data.strip().upper()
        if state == self._mission_state:
            return
        self._mission_state = state
        self._safe_pose_sent = False
        self._stop_sent_for_state = False
        self._reset_search_cycle()
        if state not in ("SEARCH", "TRACK", "ACQUIRE", "LOCALIZE", "LOCK_TARGET", "APPROACH", "FINE_ALIGN"):
            self._stop_tracking()
        self._publish_control_state(state)

    def _on_manual_override(self, msg):
        self._manual_override = bool(msg.data)
        if self._manual_override:
            self._stop_tracking()
            self._reset_search_cycle()
        self._publish_control_state(self._mission_state)

    def _on_set_manual_override(self, req):
        self._manual_override = bool(req.data)
        if self._manual_override:
            self._stop_tracking()
            self._reset_search_cycle()
        self._publish_control_state(self._mission_state)
        return SetBoolResponse(success=True, message="manual_override=%s" % self._manual_override)

    def _auto_allowed(self):
        return not self._manual_override

    def _tracking_allowed(self):
        return self._auto_allowed() and self.enable_tracking and self._mission_state in (
            "TRACK",
            "ACQUIRE",
            "LOCALIZE",
            "LOCK_TARGET",
            "APPROACH",
            "FINE_ALIGN",
        )

    def _search_allowed(self):
        return self._auto_allowed() and self.enable_search and self._mission_state == "SEARCH"

    def _safe_pose_allowed(self):
        return self._auto_allowed() and self._mission_state == "WAIT_HOVER"

    def _control_loop(self):
        period = 1.0 / self.control_publish_hz
        while not rospy.is_shutdown() and not self._control_stop.wait(period):
            should_publish = True
            with self._control_lock:
                age = time.monotonic() - self._last_vision
                yaw = self._desired_yaw
                pitch = self._desired_pitch
                if not self._tracking_allowed() or not self._tracking_active:
                    should_publish = False
                elif age > self.stale_timeout:
                    if self._stale_stop_sent:
                        should_publish = False
                    else:
                        yaw = pitch = 0.0
                        self._stale_stop_sent = True
                        self._tracking_active = False
            if not should_publish:
                continue
            if age > self.stale_timeout:
                yaw = pitch = 0.0
            self.speed_pub.publish(Vector3(x=yaw, y=pitch, z=0.0))

    def _on_gimbal_angles(self, msg):
        self._current_gimbal = msg

    def _reset_search_cycle(self):
        self._search_state = "wait_initial"
        self._search_goal = None
        self._search_goal_since = 0.0
        self._search_arrived_since = None

    def _send_search_goal(self, yaw, pitch):
        self._search_goal = (float(yaw), float(pitch))
        self._search_goal_since = time.monotonic()
        self._search_arrived_since = None
        self.angle_cmd_pub.publish(Vector3(x=float(yaw), y=float(pitch), z=self.search_speed))
        rospy.loginfo("search goal yaw=%.1f, pitch=%.1f, speed=%.1f", yaw, pitch, self.search_speed)

    def _search_goal_stable(self, stable_sec):
        if self._search_goal is None:
            return False
        now = time.monotonic()
        if self._current_gimbal is None:
            return now - self._search_goal_since >= self.search_arrival_timeout
        yaw_goal, pitch_goal = self._search_goal
        angles = self._current_gimbal.vector
        arrived = abs(float(angles.x) - yaw_goal) <= self.search_angle_tolerance and abs(float(angles.y) - pitch_goal) <= self.search_angle_tolerance
        if not arrived:
            self._search_arrived_since = None
            if now - self._search_goal_since > self.search_arrival_timeout:
                rospy.logwarn("search goal arrival timeout; continuing scan")
                return True
            return False
        if self._search_arrived_since is None:
            self._search_arrived_since = now
        return now - self._search_arrived_since >= stable_sec

    def _set_search_state(self, state, yaw, pitch):
        rospy.loginfo("search state -> %s", state)
        self._search_state = state
        self._send_search_goal(yaw, pitch)

    def _current_search_yaw(self):
        if self._current_gimbal is not None:
            return float(self._current_gimbal.vector.x)
        if self._search_goal is not None:
            return float(self._search_goal[0])
        return self.search_startup_yaw

    def _current_search_pitch(self):
        if self._current_gimbal is not None:
            return float(self._current_gimbal.vector.y)
        if self._search_goal is not None:
            return float(self._search_goal[1])
        return self.search_startup_pitch

    def _set_pitch_segment(self, state, target_pitch):
        pitch = self._current_search_pitch()
        next_pitch = max(target_pitch, pitch - self.search_pitch_step) if target_pitch < pitch else min(target_pitch, pitch + self.search_pitch_step)
        self._set_search_state(state, self.search_startup_yaw, next_pitch)

    def _set_sweep_segment(self, state, direction):
        yaw = self._current_search_yaw()
        next_yaw = max(self.search_left_yaw, yaw - self.search_sweep_step) if direction < 0.0 else min(self.search_right_yaw, yaw + self.search_sweep_step)
        self._set_search_state(state, next_yaw, self.search_sweep_pitch)

    def _search_loop(self, _event):
        if self._safe_pose_allowed() and (
            not self._safe_pose_sent
            or time.monotonic() - self._last_safe_pose >= self.safe_pose_repeat_sec
        ):
            self.angle_cmd_pub.publish(Vector3(x=self.safe_yaw, y=self.safe_pitch, z=self.safe_pose_speed))
            self._safe_pose_sent = True
            self._last_safe_pose = time.monotonic()
            self._publish_control_state("WAIT_HOVER_SAFE")
            return
        if self._mission_state in ("ARRIVED", "ABORTED", "TARGET_LOST", "IDLE") and not self._stop_sent_for_state:
            self._stop_tracking()
            self._stop_sent_for_state = True
            self._publish_control_state(self._mission_state)
            return
        if not self._search_allowed():
            return
        if self._tag_visible:
            self._stop_tracking()
            self._reset_search_cycle()
            return
        if self._search_goal is None:
            self._send_search_goal(self.search_startup_yaw, self.search_startup_pitch)
            return
        if self._search_state == "wait_initial" and self._search_goal_stable(self.search_stable_sec):
            self._set_pitch_segment("move_up", self.search_up_pitch)
        elif self._search_state == "move_up" and self._search_goal_stable(self.search_pitch_pause):
            self._set_pitch_segment("move_down" if self._search_goal[1] >= self.search_up_pitch else "move_up", self.search_down_pitch if self._search_goal[1] >= self.search_up_pitch else self.search_up_pitch)
        elif self._search_state == "move_down" and self._search_goal_stable(self.search_pitch_pause):
            if self._search_goal[1] <= self.search_down_pitch:
                self._set_sweep_segment("sweep_left", -1.0)
            else:
                self._set_pitch_segment("move_down", self.search_down_pitch)
        elif self._search_state == "sweep_left" and self._search_goal_stable(self.search_sweep_pause):
            self._set_sweep_segment("sweep_right" if self._search_goal[0] <= self.search_left_yaw else "sweep_left", 1.0 if self._search_goal[0] <= self.search_left_yaw else -1.0)
        elif self._search_state == "sweep_right" and self._search_goal_stable(self.search_sweep_pause):
            if self._search_goal[0] >= self.search_right_yaw:
                self._set_search_state("return_initial", self.search_startup_yaw, self.search_startup_pitch)
            else:
                self._set_sweep_segment("sweep_right", 1.0)
        elif self._search_state == "return_initial" and self._search_goal_stable(0.2):
            rospy.loginfo("search cycle finished; restarting")
            self._reset_search_cycle()

    def _tracking_command(self, center, width, height):
        yaw = pitch = 0.0
        if center is not None:
            ex = float(center[0]) - width / 2.0
            ey = float(center[1]) - height / 2.0
            if abs(ex) > self.deadband_px:
                yaw = np.clip(self.yaw_sign * self.yaw_kp * ex, -self.max_speed, self.max_speed)
            if abs(ey) > self.deadband_px:
                pitch = np.clip(self.pitch_sign * self.pitch_kp * ey, -self.max_speed, self.max_speed)
        self._set_desired(float(yaw), float(pitch))

    def _detect_apriltag(self, gray):
        detections = self.apriltag_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(self.fx, self.fy, self.cx, self.cy),
            tag_size=self.tag_size,
        )
        candidates = [d for d in detections if self.target_id < 0 or int(d.tag_id) == self.target_id]
        if not candidates:
            return None
        det = max(candidates, key=lambda d: polygon_area(np.asarray(d.corners)))
        return {
            "id": int(det.tag_id),
            "code": str(det.tag_id),
            "center": np.asarray(det.center, dtype=float),
            "corners": np.asarray(det.corners, dtype=float),
            "rotation": np.asarray(det.pose_R, dtype=float),
            "translation": np.asarray(det.pose_t, dtype=float).reshape(3),
            "error": float(getattr(det, "pose_err", 0.0)),
            "score": float(getattr(det, "decision_margin", 0.0)),
        }

    def _estimate_square_pose(self, corners):
        corners = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        half = self.tag_size / 2.0
        object_points = np.array([[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float32)
        result = cv2.solvePnPGeneric(object_points, corners, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not result[0]:
            return None
        choices = []
        errors = result[3] if len(result) > 3 else [0.0] * len(result[1])
        for rvec, tvec, generic_error in zip(result[1], result[2], errors):
            translation = np.asarray(tvec, dtype=float).reshape(3)
            if translation[2] <= 0.0:
                continue
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            reprojection_error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - corners) ** 2, axis=1))))
            choices.append((reprojection_error, float(np.asarray(generic_error).reshape(-1)[0]), rvec, translation))
        if not choices:
            return None
        error, _, rvec, translation = min(choices, key=lambda item: (item[0], item[1]))
        rotation, _ = cv2.Rodrigues(rvec)
        return rotation, translation, error

    def _detect_aruco(self, gray):
        if hasattr(self.aruco_detector, "detectMarkers"):
            corners_list, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            dictionary, parameters = self.aruco_detector
            corners_list, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        if ids is None or len(corners_list) == 0:
            return None
        candidates = []
        for corners, marker_id in zip(corners_list, ids.reshape(-1)):
            marker_id = int(marker_id)
            if self.target_id >= 0 and marker_id != self.target_id:
                continue
            corners_array = np.asarray(corners, dtype=np.float32).reshape(4, 2)
            candidates.append((polygon_area(corners_array), marker_id, corners_array))
        if not candidates:
            return None
        _area, marker_id, corners = max(candidates, key=lambda item: item[0])
        pose = self._estimate_square_pose(corners)
        if pose is None:
            return None
        rotation, translation, error = pose
        return {"id": marker_id, "code": str(marker_id), "center": corners.mean(axis=0), "corners": corners, "rotation": rotation, "translation": translation, "error": error, "score": float(_area)}

    def _detect_qrcode(self, gray):
        candidates = []
        try:
            ok, decoded, points, _ = self.qr_detector.detectAndDecodeMulti(gray)
        except Exception:
            ok, decoded, points = False, [], None
        if ok and points is not None:
            for code, corners in zip(decoded, points):
                if not self.target_code or code == self.target_code:
                    candidates.append((code, np.asarray(corners, dtype=np.float32).reshape(4, 2)))
        if not candidates:
            code, corners, _ = self.qr_detector.detectAndDecode(gray)
            if corners is not None and (not self.target_code or code == self.target_code):
                candidates.append((code, np.asarray(corners, dtype=np.float32).reshape(4, 2)))
        if not candidates:
            return None
        code, corners = max(candidates, key=lambda item: polygon_area(item[1]))
        pose = self._estimate_square_pose(corners)
        if pose is None:
            return None
        rotation, translation, error = pose
        return {"id": -1, "code": code, "center": corners.mean(axis=0), "corners": corners, "rotation": rotation, "translation": translation, "error": error, "score": 1.0}

    def process_latest(self, _event):
        snapshot = self.reader.get_latest(copy=True)
        if snapshot is None or snapshot.sequence == self.last_sequence:
            return
        self.last_sequence = snapshot.sequence
        frame = snapshot.frame
        height, width = frame.shape[:2]
        if self.use_undistort and np.any(np.abs(self.dist_coeffs) > 1e-12):
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detection = self._detect_apriltag(gray) if self.detector_type == "apriltag" else self._detect_aruco(gray) if self.detector_type == "aruco" else self._detect_qrcode(gray)

        self.visible_pub.publish(Bool(data=detection is not None))
        self._tag_visible = detection is not None
        center = None
        if detection is not None:
            now = rospy.Time.from_sec(snapshot.capture_time_ns * 1e-9) if snapshot.capture_time_ns > 0 else rospy.Time.now()
            center = detection["center"]
            center_msg = PointStamped()
            center_msg.header.stamp = now
            center_msg.header.frame_id = self.frame_id
            center_msg.point.x = float(center[0])
            center_msg.point.y = float(center[1])
            center_msg.point.z = float(detection["score"])
            self.center_pub.publish(center_msg)
            self.id_pub.publish(Int32(data=int(detection["id"])))
            self.code_pub.publish(String(data=str(detection["code"])))
            self.error_pub.publish(Float32(data=float(detection["error"])))

            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self.frame_id
            t = detection["translation"]
            pose.pose.position.x = float(t[0])
            pose.pose.position.y = float(t[1])
            pose.pose.position.z = float(t[2])
            qx, qy, qz, qw = matrix_to_quaternion(detection["rotation"])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pose_pub.publish(pose)

            corners = detection["corners"].astype(int)
            cv2.polylines(frame, [corners], True, (0, 255, 0), 3)
            cv2.circle(frame, tuple(center.astype(int)), 6, (0, 0, 255), -1)
            cv2.putText(frame, "%s %s z=%.2fm" % (self.detector_type, detection["code"], t[2]), tuple(corners[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if self._tracking_allowed() and center is not None:
            self._tracking_command(center, width, height)
        elif center is None and self._tracking_active and not self._stale_stop_sent:
            # The control loop will send one zero-speed command after stale_timeout.
            pass

        if self.debug_pub is not None:
            cv2.drawMarker(frame, (width // 2, height // 2), (255, 0, 0), cv2.MARKER_CROSS, 30, 2)
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id
                msg.format = "jpeg"
                msg.data = encoded.tobytes()
                self.debug_pub.publish(msg)

    def shutdown(self):
        self._set_desired(0.0, 0.0)
        self._control_stop.set()
        if self._control_thread.is_alive():
            self._control_thread.join(timeout=1.0)
        if self.enable_tracking:
            self.speed_pub.publish(Vector3())
        self.reader.stop()


def main():
    rospy.init_node("c12_tag_pose")
    node = C12TagPoseNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
