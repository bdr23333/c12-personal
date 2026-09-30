"""Latest-frame RTSP reader.

Backends:
- gstreamer: OpenCV + GStreamer
- ffmpeg: OpenCV + FFmpeg
- ffmpeg_cli: system ffmpeg process + raw BGR pipe

The ffmpeg_cli backend is useful when OpenCV reports that CAP_FFMPEG is
available but still cannot open an HEVC RTSP stream.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class FrameSnapshot:
    sequence: int
    capture_time_ns: int
    frame: np.ndarray


class LatestFrameReader:
    def __init__(
        self,
        url: str,
        backend: str = "gstreamer",
        transport: str = "tcp",
        latency_ms: int = 0,
        reconnect_sec: float = 1.0,
        logger=None,
        time_source_ns: Optional[Callable[[], int]] = None,
    ) -> None:
        self.url = url
        self.backend = backend.lower()
        self.transport = transport.lower()
        self.latency_ms = max(0, int(latency_ms))
        self.reconnect_sec = max(0.2, float(reconnect_sec))
        self.logger = logger
        self.time_source_ns = time_source_ns or time.time_ns

        if self.backend not in ("gstreamer", "ffmpeg", "ffmpeg_cli"):
            raise ValueError(
                "backend must be gstreamer, ffmpeg, or ffmpeg_cli"
            )
        if self.transport not in ("tcp", "udp"):
            raise ValueError("transport must be tcp or udp")

        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._sequence = 0
        self._capture_time_ns = 0
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._capture: Optional[cv2.VideoCapture] = None
        self._process: Optional[subprocess.Popen] = None
        self.active_backend = self.backend

    def _log_info(self, text: str) -> None:
        if self.logger is not None:
            self.logger.info(text)

    def _log_warning(self, text: str) -> None:
        if self.logger is not None:
            self.logger.warning(text)

    def _gstreamer_pipeline(self) -> str:
        return (
            f"rtspsrc location={self.url} protocols={self.transport} "
            f"latency={self.latency_ms} drop-on-latency=true ! "
            "rtph265depay ! h265parse ! avdec_h265 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "queue max-size-buffers=1 leaky=downstream ! "
            "appsink max-buffers=1 drop=true sync=false"
        )

    def _opencv_has_gstreamer(self) -> bool:
        for line in cv2.getBuildInformation().splitlines():
            if "GStreamer" in line:
                return "YES" in line.upper()
        return False

    def _open_gstreamer(self) -> Optional[cv2.VideoCapture]:
        if not self._opencv_has_gstreamer():
            self._log_warning(
                "This OpenCV build has no GStreamer support"
            )
            return None
        self._log_info("Opening RTSP with GStreamer")
        capture = cv2.VideoCapture(
            self._gstreamer_pipeline(), cv2.CAP_GSTREAMER
        )
        if not capture.isOpened():
            capture.release()
            return None
        return capture

    def _open_ffmpeg(self) -> Optional[cv2.VideoCapture]:
        self._log_info("Opening RTSP with OpenCV/FFmpeg")
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            f"rtsp_transport;{self.transport}"
            "|fflags;nobuffer"
            "|flags;low_delay"
            "|probesize;32"
            "|analyzeduration;0"
            "|max_delay;0"
        )
        capture = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            return None
        return capture

    def _open(self) -> Optional[cv2.VideoCapture]:
        if self.backend == "gstreamer":
            capture = self._open_gstreamer()
            if capture is not None:
                self.active_backend = "gstreamer"
                return capture
            self._log_warning(
                "GStreamer unavailable; falling back to FFmpeg"
            )
        self.active_backend = "ffmpeg"
        return self._open_ffmpeg()

    def _probe_resolution(self) -> Tuple[int, int]:
        ffprobe = shutil.which("ffprobe")
        if ffprobe is None:
            raise RuntimeError("ffprobe executable not found")

        command = [
            ffprobe,
            "-v", "error",
            "-rtsp_transport", self.transport,
            "-select_streams", "v:0",
            "-show_entries", "stream=width,height",
            "-of", "csv=s=x:p=0",
            self.url,
        ]
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=15,
            check=False,
        )
        value = result.stdout.strip().splitlines()
        if result.returncode != 0 or not value:
            detail = result.stderr.strip()
            raise RuntimeError(
                "ffprobe could not read RTSP resolution"
                + (f": {detail}" if detail else "")
            )

        width_text, height_text = value[-1].split("x", maxsplit=1)
        width = int(width_text)
        height = int(height_text)
        if width <= 0 or height <= 0:
            raise RuntimeError(
                f"invalid RTSP resolution: {width}x{height}"
            )
        return width, height

    def _start_ffmpeg_cli(
        self,
    ) -> Tuple[subprocess.Popen, int, int]:
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("ffmpeg executable not found")

        width, height = self._probe_resolution()
        command = [
            ffmpeg,
            "-hide_banner",
            "-loglevel", "error",
            "-rtsp_transport", self.transport,
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-probesize", "32",
            "-analyzeduration", "0",
            "-i", self.url,
            "-map", "0:v:0",
            "-an", "-sn", "-dn",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "pipe:1",
        ]

        self._log_info(
            f"Opening RTSP with system FFmpeg: "
            f"{width}x{height}; transport={self.transport}"
        )
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=width * height * 3 * 2,
        )
        return process, width, height

    @staticmethod
    def _read_exact(stream, size: int) -> Optional[bytes]:
        data = bytearray()
        while len(data) < size:
            chunk = stream.read(size - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    def _stop_ffmpeg_process(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)

    def _loop_ffmpeg_cli(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._process, width, height = self._start_ffmpeg_cli()
                self.active_backend = "ffmpeg_cli"
                frame_bytes = width * height * 3
                self._log_info(
                    f"RTSP opened with ffmpeg_cli: {self.url}"
                )

                assert self._process.stdout is not None
                while not self._stop_event.is_set():
                    raw = self._read_exact(
                        self._process.stdout,
                        frame_bytes,
                    )
                    if raw is None:
                        raise RuntimeError(
                            "FFmpeg raw-video pipe closed"
                        )
                    frame = np.frombuffer(
                        raw, dtype=np.uint8
                    ).reshape((height, width, 3))

                    with self._lock:
                        self._latest = frame.copy()
                        self._sequence += 1
                        self._capture_time_ns = self.time_source_ns()

            except Exception as exc:
                self._log_warning(
                    f"ffmpeg_cli RTSP error: "
                    f"{type(exc).__name__}: {exc}; retrying"
                )
            finally:
                self._stop_ffmpeg_process()

            self._stop_event.wait(self.reconnect_sec)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        if self.backend == "ffmpeg_cli":
            self._loop_ffmpeg_cli()
            return

        while not self._stop_event.is_set():
            try:
                if self._capture is None or not self._capture.isOpened():
                    self._capture = self._open()
                    if self._capture is None:
                        self._log_warning(
                            f"Cannot open RTSP: {self.url}; retrying"
                        )
                        self._stop_event.wait(self.reconnect_sec)
                        continue
                    self._log_info(
                        f"RTSP opened with {self.active_backend}: {self.url}"
                    )

                ok, frame = self._capture.read()
                if not ok or frame is None:
                    self._log_warning("RTSP read failed; reconnecting")
                    self._capture.release()
                    self._capture = None
                    self._stop_event.wait(self.reconnect_sec)
                    continue

                with self._lock:
                    self._latest = frame
                    self._sequence += 1
                    self._capture_time_ns = self.time_source_ns()
            except Exception as exc:
                self._log_warning(
                    f"RTSP worker exception: {type(exc).__name__}: {exc}"
                )
                if self._capture is not None:
                    self._capture.release()
                self._capture = None
                self._stop_event.wait(self.reconnect_sec)

    def get_latest(self, copy: bool = True) -> Optional[FrameSnapshot]:
        with self._lock:
            if self._latest is None:
                return None
            frame = self._latest.copy() if copy else self._latest
            return FrameSnapshot(
                sequence=self._sequence,
                capture_time_ns=self._capture_time_ns,
                frame=frame,
            )

    def stop(self) -> None:
        self._stop_event.set()
        if self._capture is not None:
            self._capture.release()
        self._stop_ffmpeg_process()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
