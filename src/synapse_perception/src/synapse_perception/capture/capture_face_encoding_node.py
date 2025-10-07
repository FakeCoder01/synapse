#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project Synapse — capture_face_encoding_node

Provides a ROS 2 service (CaptureFaceEncoding) that captures a single face
encoding from a live camera topic. Useful for UI/assistant-driven person
enrollment flows initiated by the Conversation Manager.

Behavior:
- Maintains (and lazily creates) subscribers to requested image topics.
- On service request:
  - Ensures a subscriber exists for the given image_topic.
  - Waits up to timeout_sec for a fresh frame.
  - Detects faces (hog/cnn, upsample) and filters by min size in pixels.
  - Selects target face by target_index or largest area if prefer_largest_face.
  - Computes the face encoding (e.g., 128-d for face_recognition).
  - Returns the encoding vector, chosen index, number of faces, and bbox.

Notes:
- To receive frames while the service waits, this node internally uses a
  ReentrantCallbackGroup. For best results, run the node with a MultiThreadedExecutor
  or launch this node alongside other nodes in a multi-threaded executor context.
- If you're using a single-threaded executor, configure a default subscriber to the
  expected image topic so frames are already buffered before the service call.
- Requires Python packages: face_recognition (dlib), OpenCV (cv2), cv_bridge.

Parameters:
- default_image_topic (string, default: "/camera/color/image_raw")
  Topic to subscribe for frames at startup. This helps in single-threaded
  executor environments to keep a recent frame buffered before service calls.
- auto_subscribe_on_request (bool, default: true)
  If true, the node will create a subscriber on-demand for the requested
  image_topic if it isn't already subscribed.

Service:
- synapse_interfaces/srv/CaptureFaceEncoding

Run:
  ros2 run synapse_perception capture_face_encoding_node

"""

from __future__ import annotations

import sys
import time
import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from synapse_interfaces.srv import CaptureFaceEncoding

# Optional deps: OpenCV and face_recognition
try:
    import cv2  # type: ignore
except Exception as _cv_exc:
    cv2 = None  # type: ignore

try:
    import face_recognition  # type: ignore

    _HAVE_FR = True
except Exception as _fr_exc:
    _HAVE_FR = False


@dataclass
class TopicBuffer:
    sub: rclpy.subscription.Subscription
    last_frame_bgr: Optional[np.ndarray]
    last_stamp: float


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class CaptureFaceEncodingNode(Node):
    def __init__(self) -> None:
        super().__init__("capture_face_encoding")

        if cv2 is None:
            self.get_logger().error(
                "OpenCV (cv2) not available. Please install opencv-python."
            )
        if not _HAVE_FR:
            self.get_logger().error(
                "face_recognition not available. Please install face-recognition (dlib)."
            )

        # Parameters
        self.declare_parameter("default_image_topic", "/camera/color/image_raw")
        self.declare_parameter("auto_subscribe_on_request", True)

        self.default_image_topic: str = (
            self.get_parameter("default_image_topic").get_parameter_value().string_value
        )
        self.auto_subscribe_on_request: bool = bool(
            self.get_parameter("auto_subscribe_on_request").value
        )

        self._cb_group = ReentrantCallbackGroup()
        self._bridge = CvBridge()

        # Topic buffers keyed by image topic
        self._buffers: Dict[str, TopicBuffer] = {}
        self._buffers_lock = threading.Lock()

        # Pre-subscribe to the default image topic to keep frames flowing
        if self.default_image_topic:
            try:
                self._ensure_subscription(self.default_image_topic)
                self.get_logger().info(
                    f"Pre-subscribed to default_image_topic='{self.default_image_topic}'"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to pre-subscribe to default topic '{self.default_image_topic}': {exc}"
                )

        # Service
        self._srv = self.create_service(
            CaptureFaceEncoding,
            "~capture",
            self._on_capture,
            callback_group=self._cb_group,
        )
        # Also provide a global name for convenience
        self._srv_global = self.create_service(
            CaptureFaceEncoding,
            "/perception/capture_face_encoding",
            self._on_capture,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            "CaptureFaceEncoding service ready: '~capture' and '/perception/capture_face_encoding'"
        )

    # ------------- Topic subscription and buffering -------------

    def _ensure_subscription(self, image_topic: str) -> None:
        with self._buffers_lock:
            if image_topic in self._buffers:
                return

            def _on_image(msg: Image, topic=image_topic):
                bgr = None
                try:
                    bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                except Exception as exc:
                    self.get_logger().warn(f"cv_bridge failed for {topic}: {exc}")
                    return
                now = time.monotonic()
                with self._buffers_lock:
                    tb = self._buffers.get(topic)
                    if tb is not None:
                        tb.last_frame_bgr = bgr
                        tb.last_stamp = now

            sub = self.create_subscription(
                Image, image_topic, _on_image, 10, callback_group=self._cb_group
            )
            self._buffers[image_topic] = TopicBuffer(
                sub=sub, last_frame_bgr=None, last_stamp=0.0
            )

    def _get_latest_frame(self, image_topic: str) -> Tuple[Optional[np.ndarray], float]:
        with self._buffers_lock:
            tb = self._buffers.get(image_topic)
            if tb is None:
                return None, 0.0
            return tb.last_frame_bgr, tb.last_stamp

    # ------------- Service handler -------------

    def _on_capture(
        self,
        request: CaptureFaceEncoding.Request,
        response: CaptureFaceEncoding.Response,
    ) -> CaptureFaceEncoding.Response:
        # Validate deps
        if cv2 is None or not _HAVE_FR:
            response.success = False
            response.face_encoding = []
            response.chosen_index = -1
            response.num_faces = 0
            response.bbox_x = 0
            response.bbox_y = 0
            response.bbox_width = 0
            response.bbox_height = 0
            response.message = (
                "Required dependencies not available (cv2/face_recognition)."
            )
            return response

        topic = request.image_topic or self.default_image_topic
        if not topic:
            response.success = False
            response.message = (
                "No image_topic provided and default_image_topic is empty."
            )
            return response

        # Ensure subscriber exists
        try:
            if self.auto_subscribe_on_request:
                self._ensure_subscription(topic)
        except Exception as exc:
            response.success = False
            response.message = f"Failed to subscribe to '{topic}': {exc}"
            return response

        # Wait for a fresh frame up to timeout
        timeout_sec = float(max(0.0, request.timeout_sec or 0.0))
        deadline = (
            time.monotonic() + timeout_sec if timeout_sec > 0 else time.monotonic()
        )

        frame_bgr: Optional[np.ndarray] = None
        last_stamp: float = 0.0

        # If timeout is zero, just use current buffer
        if timeout_sec <= 0.0:
            frame_bgr, last_stamp = self._get_latest_frame(topic)
        else:
            while time.monotonic() < deadline:
                frame_bgr, last_stamp = self._get_latest_frame(topic)
                if frame_bgr is not None and (time.monotonic() - last_stamp) < 2.0:
                    break
                time.sleep(
                    0.01
                )  # Allow other callbacks to run in multi-threaded executor

        if frame_bgr is None:
            response.success = False
            response.message = f"No frame available from '{topic}' within timeout."
            response.num_faces = 0
            response.chosen_index = -1
            return response

        # Detect faces
        detection_model = request.detection_model or "hog"
        upsample_times = int(
            request.upsample_times if request.upsample_times >= 0 else 0
        )
        encoding_model = request.encoding_model or "small"
        min_face_px = float(
            request.min_face_size_px if request.min_face_size_px > 0 else 40.0
        )

        rgb = frame_bgr[:, :, ::-1]
        try:
            face_locations = face_recognition.face_locations(
                rgb,
                number_of_times_upsampled=upsample_times,
                model=detection_model,
            )
        except Exception as exc:
            response.success = False
            response.message = f"face_locations failed: {exc}"
            response.num_faces = 0
            response.chosen_index = -1
            return response

        # Filter by min face size
        filtered_idx: List[int] = []
        for i, (top, right, bottom, left) in enumerate(face_locations):
            w = max(0, right - left)
            h = max(0, bottom - top)
            if min(w, h) >= min_face_px:
                filtered_idx.append(i)

        response.num_faces = len(filtered_idx)
        if not filtered_idx:
            response.success = False
            response.chosen_index = -1
            response.message = "No faces meeting minimum size detected."
            return response

        # Select target face
        chosen_index = -1
        if request.target_index >= 0 and request.target_index < len(face_locations):
            # Use provided index if that face also passed size filter
            if request.target_index in filtered_idx:
                chosen_index = int(request.target_index)
            else:
                response.success = False
                response.chosen_index = -1
                response.message = (
                    "Requested target_index did not meet minimum size filter."
                )
                return response
        else:
            if request.prefer_largest_face:
                # Choose the largest area among filtered
                best_area = -1
                for i in filtered_idx:
                    top, right, bottom, left = face_locations[i]
                    area = max(0, right - left) * max(0, bottom - top)
                    if area > best_area:
                        best_area = area
                        chosen_index = i
            else:
                # Choose first passing filter
                chosen_index = filtered_idx[0] if filtered_idx else -1

        if chosen_index < 0:
            response.success = False
            response.chosen_index = -1
            response.message = "No face selected."
            return response

        # Compute encoding for the selected face
        try:
            # Compute only for the selected face's location to reduce computation
            loc = [face_locations[chosen_index]]
            encs = face_recognition.face_encodings(
                rgb, known_face_locations=loc, model=encoding_model
            )
        except Exception as exc:
            response.success = False
            response.chosen_index = -1
            response.message = f"face_encodings failed: {exc}"
            return response

        if not encs:
            response.success = False
            response.chosen_index = -1
            response.message = "No encoding produced for the selected face."
            return response

        enc = encs[0]
        top, right, bottom, left = face_locations[chosen_index]

        # Populate response
        response.success = True
        response.face_encoding = [float(x) for x in enc.tolist()]
        response.chosen_index = int(chosen_index)
        response.num_faces = int(len(filtered_idx))
        response.bbox_x = int(max(0, left))
        response.bbox_y = int(max(0, top))
        response.bbox_width = int(max(0, right - left))
        response.bbox_height = int(max(0, bottom - top))
        response.message = "ok"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CaptureFaceEncodingNode()
    try:
        # Use a MultiThreadedExecutor for best concurrency (service + subscriptions)
        # If your launcher uses a single-threaded executor, frames should still work
        # if you pre-subscribe to the camera topic and keep frames buffered.
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
