#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

import cv2
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from c12_ros1.ros_compat import RospyLogger
from c12_ros1.rtsp_reader import LatestFrameReader


def load_camera_info(path_text):
    if not path_text:
        return None
    path = Path(path_text).expanduser()
    if not path.exists():
        raise FileNotFoundError(path)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    msg = CameraInfo()
    msg.width = int(data["image_width"])
    msg.height = int(data["image_height"])
    msg.distortion_model = str(data.get("distortion_model", "plumb_bob"))
    msg.D = [float(x) for x in data["distortion_coefficients"]["data"]]
    msg.K = [float(x) for x in data["camera_matrix"]["data"]]
    msg.R = [float(x) for x in data["rectification_matrix"]["data"]]
    msg.P = [float(x) for x in data["projection_matrix"]["data"]]
    return msg


class C12ImageBridge:
    def __init__(self):
        self.url = rospy.get_param("~url", "rtsp://192.168.1.91:554/stream=1")
        self.publish_hz = max(1.0, float(rospy.get_param("~publish_hz", 30.0)))
        self.publish_raw = bool(rospy.get_param("~publish_raw", True))
        self.publish_compressed = bool(rospy.get_param("~publish_compressed", False))
        self.jpeg_quality = max(10, min(100, int(rospy.get_param("~jpeg_quality", 75))))
        self.resize_width = max(0, int(rospy.get_param("~resize_width", 0)))
        self.resize_height = max(0, int(rospy.get_param("~resize_height", 0)))
        self.frame_id = rospy.get_param("~frame_id", "c12_visible_optical_frame")

        if not self.publish_raw and not self.publish_compressed:
            raise ValueError("At least one of publish_raw/publish_compressed must be true")

        self.bridge = CvBridge()
        self.raw_pub = rospy.Publisher(rospy.get_param("~raw_topic", "/c12/visible/image_raw"), Image, queue_size=1) if self.publish_raw else None
        self.compressed_pub = rospy.Publisher(rospy.get_param("~compressed_topic", "/c12/visible/image_raw/compressed"), CompressedImage, queue_size=1) if self.publish_compressed else None
        self.camera_info = load_camera_info(rospy.get_param("~camera_info_file", ""))
        self.camera_info_pub = rospy.Publisher(rospy.get_param("~camera_info_topic", "/c12/visible/camera_info"), CameraInfo, queue_size=1) if self.camera_info is not None else None

        backend = rospy.get_param("~backend", "gstreamer")
        transport = rospy.get_param("~transport", "tcp")
        rospy.loginfo(
            "Starting image bridge: %s; backend=%s; transport=%s; raw=%s; compressed=%s",
            self.url,
            backend,
            transport,
            self.publish_raw,
            self.publish_compressed,
        )

        self.reader = LatestFrameReader(
            url=self.url,
            backend=backend,
            transport=transport,
            latency_ms=int(rospy.get_param("~latency_ms", 0)),
            logger=RospyLogger(),
            time_source_ns=lambda: rospy.Time.now().to_nsec(),
        )
        self.reader.start()
        self.last_sequence = -1
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self.publish_latest)

    def publish_latest(self, _event):
        snapshot = self.reader.get_latest(copy=True)
        if snapshot is None or snapshot.sequence == self.last_sequence:
            return
        self.last_sequence = snapshot.sequence
        frame = snapshot.frame

        if self.resize_width > 0 and self.resize_height > 0:
            frame = cv2.resize(frame, (self.resize_width, self.resize_height), interpolation=cv2.INTER_AREA)

        stamp = rospy.Time.now()
        if self.raw_pub is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.raw_pub.publish(msg)

        if self.compressed_pub is not None:
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.header.frame_id = self.frame_id
                msg.format = "jpeg"
                msg.data = encoded.tobytes()
                self.compressed_pub.publish(msg)

        if self.camera_info_pub is not None:
            self.camera_info.header.stamp = stamp
            self.camera_info.header.frame_id = self.frame_id
            self.camera_info_pub.publish(self.camera_info)

    def shutdown(self):
        self.reader.stop()


def main():
    rospy.init_node("c12_image_bridge")
    node = C12ImageBridge()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
