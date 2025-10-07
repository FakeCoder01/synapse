#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project Synapse — Face Enrollment Utility

This script captures face images from a ROS 2 camera topic, computes a face encoding,
and enrolls a person profile in the Cognitive Core by calling the AddPerson service.

Features:
- Subscribes to an RGB image topic (default: /camera/color/image_raw).
- Detects face(s) and computes a 128-d encoding (face_recognition/dlib).
- Lets you accumulate several samples and averages them for a robust encoding.
- Prompts for a person name (or pass as a parameter).
- Calls /memory/add_person (synapse_interfaces/AddPerson) to persist the profile.

Controls (in the OpenCV window):
- n: prompt for name (if not provided via param)
- s: sample current frame's best face encoding (adds to memory buffer)
- c: clear accumulated samples
- e: enroll (uses the average of accumulated samples; if none, uses current frame)
- q or ESC: quit

ROS 2 Parameters:
- image_topic (string): camera topic (default: "/camera/color/image_raw")
- service_name (string): AddPerson service (default: "/memory/add_person")
- detection_model (string): "hog" (CPU) or "cnn" (GPU) for face detection (default: "hog")
- upsample_times (int): upsample for detection (default: 1)
- encoding_model (string): "small" or "large" (default: "small")
- person_name (string): if provided, used as default name (default: "")
- sample_count_target (int): target sample count for averaging before enroll (default: 5)

Requirements:
- rclpy, cv_bridge, sensor_msgs
- face_recognition (dlib), OpenCV (cv2)

Usage:
    ros2 run synapse_perception enroll_person.py
Or run directly:
    python3 enroll_person.py

Note:
- If face_recognition/dlib isn't available, this tool reports an error and exits.
"""

import sys
import time
import math
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time as RosTime

from cv_bridge import CvBridge

from synapse_interfaces.srv import AddPerson

# Try to import OpenCV and face_recognition
try:
    import cv2
except Exception as e:
    print(f"[enroll_person] OpenCV import failed: {e}", file=sys.stderr)
    sys.exit(1)

try:
    import face_recognition

    _HAVE_FR = True
except Exception as e:
    print(f"[enroll_person] face_recognition import failed: {e}", file=sys.stderr)
    _HAVE_FR = False


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def largest_bbox_index(face_locations: List[Tuple[int, int, int, int]]) -> int:
    if not face_locations:
        return -1
    areas = []
    for top, right, bottom, left in face_locations:
        w = max(0, right - left)
        h = max(0, bottom - top)
        areas.append(w * h)
    return int(np.argmax(areas))


class EnrollPersonNode(Node):
    def __init__(self):
        super().__init__("enroll_person")

        if not _HAVE_FR:
            self.get_logger().error(
                "face_recognition is not available. Install 'face-recognition' and dependencies."
            )
            raise RuntimeError("face_recognition missing")

        # Parameters
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("service_name", "/memory/add_person")
        self.declare_parameter("detection_model", "hog")  # "hog" | "cnn"
        self.declare_parameter("upsample_times", 1)
        self.declare_parameter("encoding_model", "small")  # "small" | "large"
        self.declare_parameter("person_name", "")
        self.declare_parameter("sample_count_target", 5)

        self.image_topic: str = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.service_name: str = (
            self.get_parameter("service_name").get_parameter_value().string_value
        )
        self.detection_model: str = (
            self.get_parameter("detection_model").get_parameter_value().string_value
        )
        self.upsample_times: int = int(self.get_parameter("upsample_times").value)
        self.encoding_model: str = (
            self.get_parameter("encoding_model").get_parameter_value().string_value
        )
        self.person_name: str = (
            self.get_parameter("person_name").get_parameter_value().string_value
        )
        self.sample_count_target: int = max(
            1, int(self.get_parameter("sample_count_target").value)
        )

        # Subscriptions / Service client
        self.bridge = CvBridge()
        self._last_frame_bgr: Optional[np.ndarray] = None
        self._last_locations: List[Tuple[int, int, int, int]] = []
        self._last_encodings: List[np.ndarray] = []

        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.cli_add_person = self.create_client(AddPerson, self.service_name)

        # Storage for multiple samples averaged for robust encoding
        self._samples: List[np.ndarray] = []

        # Create window
        self.win_name = "Enroll Person (n=name, s=sample, c=clear, e=enroll, q=quit)"
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)

        self.get_logger().info(
            f"EnrollPerson: image_topic={self.image_topic}, service={self.service_name}, "
            f"detection_model={self.detection_model}, upsample_times={self.upsample_times}, "
            f"encoding_model={self.encoding_model}, sample_target={self.sample_count_target}"
        )
        if self.person_name:
            self.get_logger().info(f"Initial person_name='{self.person_name}'")

    def _on_image(self, msg: Image) -> None:
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        self._last_frame_bgr = frame_bgr
        rgb = frame_bgr[:, :, ::-1]

        try:
            locations = face_recognition.face_locations(
                rgb,
                number_of_times_upsampled=self.upsample_times,
                model=self.detection_model,
            )
            encodings = []
            if locations:
                encodings = face_recognition.face_encodings(
                    rgb, known_face_locations=locations, model=self.encoding_model
                )
            self._last_locations = locations
            self._last_encodings = encodings
        except Exception as exc:
            self.get_logger().error(f"face_recognition processing failed: {exc}")
            self._last_locations = []
            self._last_encodings = []

    def _overlay(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Draw face boxes and status text."""
        out = frame_bgr.copy()
        h, w = out.shape[:2]
        # Draw faces
        for i, (top, right, bottom, left) in enumerate(self._last_locations):
            color = (
                (0, 255, 0)
                if i == largest_bbox_index(self._last_locations)
                else (0, 200, 255)
            )
            cv2.rectangle(out, (left, top), (right, bottom), color, 2)
            cv2.putText(
                out,
                f"face {i}",
                (left, max(0, top - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )

        # Status text
        name_txt = self.person_name if self.person_name else "<unset>"
        samples = len(self._samples)
        help_lines = [
            f"Name: {name_txt} | Samples: {samples}/{self.sample_count_target}",
            "Keys: n=name, s=sample, c=clear, e=enroll, q=quit",
        ]
        y0 = 24
        for line in help_lines:
            cv2.putText(
                out,
                line,
                (10, y0),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                out,
                line,
                (10, y0),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            y0 += 24
        return out

    def _best_current_encoding(self) -> Optional[np.ndarray]:
        if not self._last_encodings or not self._last_locations:
            return None
        idx = largest_bbox_index(self._last_locations)
        if idx < 0 or idx >= len(self._last_encodings):
            return None
        return np.array(self._last_encodings[idx], dtype=np.float32)

    def _average_encoding(self) -> Optional[np.ndarray]:
        if not self._samples:
            enc = self._best_current_encoding()
            return enc
        # Average across accumulated samples
        avg = np.mean(np.vstack(self._samples), axis=0)
        return avg.astype(np.float32)

    def _prompt_name(self) -> None:
        try:
            # Basic blocking prompt in console
            name = input("Enter person's name: ").strip()
            if name:
                self.person_name = name
                self.get_logger().info(f"Set name to '{self.person_name}'")
        except Exception as exc:
            self.get_logger().warn(f"Unable to read name from input: {exc}")

    def _sample_once(self) -> None:
        enc = self._best_current_encoding()
        if enc is None:
            self.get_logger().warn("No face detected for sampling.")
            return
        self._samples.append(enc)
        self.get_logger().info(f"Sampled frame. Total samples: {len(self._samples)}")

    def _clear_samples(self) -> None:
        self._samples = []
        self.get_logger().info("Cleared samples.")

    def _enroll(self) -> None:
        if not self.person_name:
            self.get_logger().warn("Name is not set. Press 'n' to set a name.")
            return
        # Compute encoding
        if len(self._samples) < max(1, self.sample_count_target // 2):
            self.get_logger().warn(
                f"Only {len(self._samples)} samples collected; consider collecting ~{self.sample_count_target} for better encoding."
            )
        enc = self._average_encoding()
        if enc is None:
            self.get_logger().warn("No face available to enroll.")
            return

        # Build request
        req = AddPerson.Request()
        req.name = self.person_name
        req.face_encoding = [float(x) for x in enc.tolist()]
        req.first_seen = self.get_clock().now().to_msg()
        req.notes = (
            f"Enrolled via enroll_person tool on {time.strftime('%Y-%m-%d %H:%M:%S')}."
        )

        # Call service
        if not self.cli_add_person.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service '{self.service_name}' not available.")
            return

        fut = self.cli_add_person.call_async(req)
        self.get_logger().info("Submitting AddPerson request...")
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done():
            self.get_logger().error("AddPerson call timed out.")
            return
        resp = fut.result()
        if not resp:
            self.get_logger().error("No response from AddPerson.")
            return
        if resp.success:
            self.get_logger().info(
                f"Enrolled person_id={resp.person_id} name='{self.person_name}'"
            )
            # Clear samples after a successful enrollment
            self._samples = []
        else:
            self.get_logger().error(f"Enrollment failed: {resp.message}")

    def render_and_handle_keys(self) -> bool:
        """Render display and process key inputs. Returns False to quit."""
        frame = self._last_frame_bgr
        if frame is None:
            # No frame yet; show a blank window
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank,
                "Waiting for image...",
                (20, 180),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow(self.win_name, blank)
        else:
            overlay = self._overlay(frame)
            cv2.imshow(self.win_name, overlay)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):  # 'q' or ESC
            return False
        elif key == ord("n"):
            self._prompt_name()
        elif key == ord("s"):
            self._sample_once()
        elif key == ord("c"):
            self._clear_samples()
        elif key == ord("e"):
            self._enroll()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EnrollPersonNode()
    except Exception as e:
        # already logged above; exit
        rclpy.shutdown()
        sys.exit(2)

    try:
        # Main loop: spin node and render UI
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if not node.render_and_handle_keys():
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
