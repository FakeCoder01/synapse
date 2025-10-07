#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project Synapse - Face Perception Node

Detects faces from a camera stream, computes 128-d encodings, queries the Cognitive
Core (memory server) to identify known persons, and publishes synapse_interfaces/DetectedPersons.

Features:
- Subscribes to color image; optionally depth + camera info for 3D localization.
- Uses face_recognition (dlib) for face detection and encodings.
- Queries GetPersonByFace service for identity resolution.
- Publishes DetectedPersons with a selected primary person-of-interest.

Parameters:
- image_topic (string): Color image topic. Default: "/camera/color/image_raw"
- depth_topic (string): Depth image topic (aligned to color). Empty to disable. Default: ""
- camera_info_topic (string): CameraInfo for color image. Empty to disable. Default: ""
- output_topic (string): Output topic for DetectedPersons. Default: "/detected_persons"

- detection_model (string): "hog" (CPU) or "cnn" (GPU) for face detection. Default: "hog"
- upsample_times (int): Number of times to upsample image for detection. Default: 1
- encoding_model (string): "small" or "large" for face encoding network. Default: "small"

- match_tolerance (double): Max distance / min similarity threshold for matching. Default: 0.48
- top_k (int): Number of candidate matches requested from memory. Default: 3
- encoding_model_id (string): Identifier sent to memory service. Default: "dlib_128"

- primary_strategy (string): "closest", "center", or "largest". Default: "closest"
- min_face_px (int): Skip detections smaller than this size in px. Default: 40
- max_faces (int): Maximum faces to process per frame. Default: 10

- service_name (string): Memory service for identification. Default: "/memory/get_person_by_face"
- service_timeout (double): Seconds to wait for service responses. Default: 1.0

Notes:
- If depth and camera_info are provided, 3D positions will be estimated.
- If only camera_info is provided (no depth), bearing is estimated from intrinsics.
- If neither depth nor camera_info is available, 3D position and bearing will be 0.

License: Apache-2.0
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from cv_bridge import CvBridge

from synapse_interfaces.msg import DetectedPerson, DetectedPersons
from synapse_interfaces.srv import GetPersonByFace

try:
    import cv2  # noqa: F401
    import face_recognition

    _HAVE_FR = True
except Exception as _exc:
    _HAVE_FR = False
    _FR_IMPORT_ERROR = _exc

# ROS 2 message_filters for approx sync (optional depth+info)
try:
    from message_filters import ApproximateTimeSynchronizer, Subscriber as MFSubscriber

    _HAVE_MF = True
except Exception:
    _HAVE_MF = False


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


class FacePerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("face_perception_node")

        if not _HAVE_FR:
            self.get_logger().error(
                f"face_recognition/dlib not available: {repr(_FR_IMPORT_ERROR)}. "
                "Install 'face-recognition' and its dependencies. Node will start but won't process images."
            )

        # Parameters
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("output_topic", "/detected_persons")

        self.declare_parameter("detection_model", "hog")  # "hog" or "cnn"
        self.declare_parameter("upsample_times", 1)
        self.declare_parameter("encoding_model", "small")  # "small" or "large"

        self.declare_parameter("match_tolerance", 0.48)
        self.declare_parameter("top_k", 3)
        self.declare_parameter("encoding_model_id", "dlib_128")

        self.declare_parameter(
            "primary_strategy", "closest"
        )  # "closest" | "center" | "largest"
        self.declare_parameter("min_face_px", 40)
        self.declare_parameter("max_faces", 10)

        self.declare_parameter("service_name", "/memory/get_person_by_face")
        self.declare_parameter("service_timeout", 1.0)

        self.image_topic: str = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.depth_topic: str = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        self.camera_info_topic: str = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.output_topic: str = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        self.detection_model: str = (
            self.get_parameter("detection_model").get_parameter_value().string_value
        )
        self.upsample_times: int = int(self.get_parameter("upsample_times").value)
        self.encoding_model: str = (
            self.get_parameter("encoding_model").get_parameter_value().string_value
        )

        self.match_tolerance: float = float(self.get_parameter("match_tolerance").value)
        self.top_k: int = int(self.get_parameter("top_k").value)
        self.encoding_model_id: str = (
            self.get_parameter("encoding_model_id").get_parameter_value().string_value
        )

        self.primary_strategy: str = (
            self.get_parameter("primary_strategy").get_parameter_value().string_value
        )
        self.min_face_px: int = int(self.get_parameter("min_face_px").value)
        self.max_faces: int = int(self.get_parameter("max_faces").value)

        self.service_name: str = (
            self.get_parameter("service_name").get_parameter_value().string_value
        )
        self.service_timeout: float = float(self.get_parameter("service_timeout").value)

        self.bridge = CvBridge()
        self._last_camera_info: Optional[CameraInfo] = None
        self._intrinsics: Optional[CameraIntrinsics] = None

        # Publisher
        self.pub = self.create_publisher(DetectedPersons, self.output_topic, 10)

        # Service client for identity resolution
        self.mem_client = self.create_client(GetPersonByFace, self.service_name)
        # Do not block forever; log if unavailable
        if not self.mem_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"Memory service '{self.service_name}' not available yet. "
                "Faces will be published as UNKNOWN until the service is ready."
            )

        # Subscriptions
        self._build_subscriptions()

        self.get_logger().info(
            "FacePerceptionNode ready:"
            f"\n  image_topic: {self.image_topic}"
            f"\n  depth_topic: {self.depth_topic or '(disabled)'}"
            f"\n  camera_info_topic: {self.camera_info_topic or '(disabled)'}"
            f"\n  output_topic: {self.output_topic}"
            f"\n  detection_model: {self.detection_model}, upsample_times: {self.upsample_times}, encoding_model: {self.encoding_model}"
            f"\n  match_tolerance: {self.match_tolerance:.3f}, top_k: {self.top_k}, encoding_model_id: {self.encoding_model_id}"
            f"\n  primary_strategy: {self.primary_strategy}, min_face_px: {self.min_face_px}, max_faces: {self.max_faces}"
            f"\n  service: {self.service_name}, timeout: {self.service_timeout:.2f}s"
        )

    def _build_subscriptions(self) -> None:
        # Keep a simple CameraInfo subscriber in any case to cache intrinsics
        if self.camera_info_topic:
            self.create_subscription(
                CameraInfo, self.camera_info_topic, self._on_camera_info, 10
            )

        if self.depth_topic and self.camera_info_topic and _HAVE_MF:
            # Approx sync of image + depth + info
            self.get_logger().info(
                "Using approximate time sync for color+depth+camera_info"
            )
            self._sub_color = MFSubscriber(
                self, Image, self.image_topic, qos_profile=10
            )
            self._sub_depth = MFSubscriber(
                self, Image, self.depth_topic, qos_profile=10
            )
            self._sub_info = MFSubscriber(
                self, CameraInfo, self.camera_info_topic, qos_profile=10
            )

            self._ats = ApproximateTimeSynchronizer(
                [self._sub_color, self._sub_depth, self._sub_info],
                queue_size=10,
                slop=0.05,
                allow_headerless=False,
            )
            self._ats.registerCallback(self._on_synced)
        else:
            # Color-only subscription
            self.get_logger().info("Subscribing to color image only (no synced depth).")
            self.create_subscription(Image, self.image_topic, self._on_color_only, 10)

            if self.depth_topic and not _HAVE_MF:
                self.get_logger().warn(
                    "Depth topic provided but message_filters not available. "
                    "Depth will be ignored; install message_filters to enable sync."
                )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._last_camera_info = msg
        if msg.k and len(msg.k) == 9:
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            self._intrinsics = CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy)

    def _on_color_only(self, img_msg: Image) -> None:
        if not _HAVE_FR:
            return
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        rgb = frame_bgr[:, :, ::-1]  # BGR -> RGB
        persons = self._detect_and_identify(
            rgb, img_msg.header, depth_img=None, cam_info=self._last_camera_info
        )
        if persons is None:
            return
        self.pub.publish(persons)

    def _on_synced(
        self, img_msg: Image, depth_msg: Image, info_msg: CameraInfo
    ) -> None:
        if not _HAVE_FR:
            return

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed (color): {exc}")
            return

        # Cache camera info/intrinsics
        self._on_camera_info(info_msg)

        # Depth image raw (no conversion to 8-bit!)
        try:
            depth_img = self.bridge.imgmsg_to_cv2(
                depth_msg
            )  # dtype depends on encoding
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed (depth): {exc}")
            depth_img = None

        rgb = frame_bgr[:, :, ::-1]
        persons = self._detect_and_identify(
            rgb, img_msg.header, depth_img=depth_img, cam_info=info_msg
        )
        if persons is None:
            return
        self.pub.publish(persons)

    def _detect_and_identify(
        self,
        rgb: np.ndarray,
        header: Header,
        depth_img: Optional[np.ndarray],
        cam_info: Optional[CameraInfo],
    ) -> Optional[DetectedPersons]:
        # Detect faces
        try:
            face_locations = face_recognition.face_locations(
                rgb,
                number_of_times_upsampled=self.upsample_times,
                model=self.detection_model,
            )
        except Exception as exc:
            self.get_logger().error(f"face_recognition.face_locations failed: {exc}")
            return None

        if not face_locations:
            # Publish empty message
            out = DetectedPersons()
            out.header = header
            out.persons = []
            out.primary_index = -1
            out.primary_reason = ""
            return out

        # Compute encodings (limit faces if too many)
        face_locations = face_locations[: self.max_faces]
        try:
            encodings = face_recognition.face_encodings(
                rgb, known_face_locations=face_locations, model=self.encoding_model
            )
        except Exception as exc:
            self.get_logger().error(f"face_recognition.face_encodings failed: {exc}")
            return None

        frame_h, frame_w = rgb.shape[:2]
        persons: List[DetectedPerson] = []
        for i, (loc, enc) in enumerate(zip(face_locations, encodings)):
            top, right, bottom, left = loc  # top, right, bottom, left
            w = max(0, right - left)
            h = max(0, bottom - top)
            if w < self.min_face_px or h < self.min_face_px:
                continue

            # Center pixel of the face bbox
            u = int(left + w / 2)
            v = int(top + h / 2)

            # Estimate 3D and bearing if possible
            position = Point()
            distance_m = 0.0
            bearing_rad = 0.0
            frame_id = header.frame_id

            if (
                depth_img is not None
                and cam_info is not None
                and self._intrinsics is not None
            ):
                z = self._depth_at(depth_img, u, v, cam_info)
                if z > 0.0 and np.isfinite(z):
                    x, y, z = self._project_to_3d(u, v, z, self._intrinsics)
                    position.x = float(x)
                    position.y = float(y)
                    position.z = float(z)
                    distance_m = float(math.sqrt(x * x + y * y + z * z))
                    # Approximate bearing around vertical axis: atan2(x, z)
                    bearing_rad = float(math.atan2(x, z))
                    frame_id = cam_info.header.frame_id or header.frame_id
                else:
                    # No valid depth at that pixel; fallback to bearing-only if intrinsics available
                    bearing_rad = self._bearing_from_pixel(u, self._intrinsics, frame_w)
                    frame_id = cam_info.header.frame_id or header.frame_id
            elif self._intrinsics is not None:
                bearing_rad = self._bearing_from_pixel(u, self._intrinsics, frame_w)
                frame_id = cam_info.header.frame_id if cam_info else header.frame_id

            # Query identity from memory
            is_known, person_id, name, rec_conf = self._query_identity(enc)

            # Build DetectedPerson
            dp = DetectedPerson()
            dp.header.stamp = header.stamp
            dp.header.frame_id = frame_id
            dp.track_id = i  # simple per-frame ID; replace with tracker if needed
            dp.is_known = is_known
            dp.person_id = person_id
            dp.name = name
            dp.recognition_confidence = float(rec_conf)
            dp.speaking_confidence = 0.0  # To be filled by audio pipeline if available
            dp.bbox_x = int(left)
            dp.bbox_y = int(top)
            dp.bbox_width = int(w)
            dp.bbox_height = int(h)
            dp.position = position
            dp.distance_m = float(distance_m)
            dp.bearing_rad = float(bearing_rad)
            dp.perceived_emotion = ""
            dp.perceived_emotion_confidence = 0.0

            persons.append(dp)

        # Construct output
        out = DetectedPersons()
        out.header = header
        out.header.frame_id = (
            persons[0].header.frame_id if persons else (header.frame_id or "")
        )
        out.persons = persons

        # Primary selection
        primary_index, reason = self._select_primary(persons, frame_w, frame_h)
        out.primary_index = int(primary_index)
        out.primary_reason = reason

        return out

    def _query_identity(self, encoding: np.ndarray) -> Tuple[bool, str, str, float]:
        """
        Query Cognitive Core with a face encoding to identify a known person.

        Returns: (is_known, person_id, name, match_confidence)
        """
        if not self.mem_client.service_is_ready():
            # Service unavailable; mark as unknown
            return (False, "", "", 0.0)

        req = GetPersonByFace.Request()
        req.face_encoding = [float(x) for x in encoding.tolist()]
        req.tolerance = float(self.match_tolerance)
        req.k = int(self.top_k)
        req.encoding_model = self.encoding_model_id

        try:
            future = self.mem_client.call_async(req)
            # Wait with timeout
            if not self._wait_future(future, timeout=self.service_timeout):
                return (False, "", "", 0.0)
            resp = future.result()
            if resp is None:
                return (False, "", "", 0.0)
            if resp.success and resp.person_id:
                return (True, resp.person_id, resp.name, float(resp.match_confidence))
            else:
                return (False, "", "", float(getattr(resp, "match_confidence", 0.0)))
        except Exception as exc:
            self.get_logger().warn(f"GetPersonByFace call failed: {exc}")
            return (False, "", "", 0.0)

    def _select_primary(
        self, persons: List[DetectedPerson], frame_w: int, frame_h: int
    ) -> Tuple[int, str]:
        """
        Select primary person-of-interest based on strategy:
        - "closest": smallest distance_m > 0, fallback to "center"
        - "center": smallest pixel distance to frame center
        - "largest": largest bbox area
        """
        if not persons:
            return (-1, "")

        strategy = self.primary_strategy.lower()
        if strategy not in ("closest", "center", "largest"):
            strategy = "closest"

        if strategy == "closest":
            # Consider only persons with a positive distance
            valid = [
                (i, p)
                for i, p in enumerate(persons)
                if p.distance_m > 0.0 and math.isfinite(p.distance_m)
            ]
            if valid:
                idx = min(valid, key=lambda ip: ip[1].distance_m)[0]
                return (idx, "closest")
            # Fallback to center if no valid distances
            strategy = "center"

        if strategy == "center":
            cx = frame_w * 0.5
            cy = frame_h * 0.5

            def center_dist(p: DetectedPerson) -> float:
                px = float(p.bbox_x + p.bbox_width / 2.0)
                py = float(p.bbox_y + p.bbox_height / 2.0)
                return (px - cx) ** 2 + (py - cy) ** 2

            idx = min(range(len(persons)), key=lambda i: center_dist(persons[i]))
            return (idx, "center")

        if strategy == "largest":

            def area(p: DetectedPerson) -> int:
                return int(p.bbox_width) * int(p.bbox_height)

            idx = max(range(len(persons)), key=lambda i: area(persons[i]))
            return (idx, "largest")

        return (-1, "")

    @staticmethod
    def _depth_to_meters(depth_value: float, encoding: str) -> float:
        """
        Convert depth to meters based on encoding:
        - "16UC1": millimeters (uint16), convert by /1000.0
        - "32FC1": meters (float32)
        """
        if encoding.upper() == "16UC1":
            return float(depth_value) / 1000.0
        elif encoding.upper() == "32FC1":
            return float(depth_value)
        else:
            # Unknown encoding; try reasonable assumption for typical RealSense depth (16UC1)
            return float(depth_value) / 1000.0

    def _depth_at(
        self, depth_img: np.ndarray, u: int, v: int, cam_info: CameraInfo
    ) -> float:
        """
        Sample depth value around (u, v) with a small window to reduce noise.
        Returns meters (float) or 0.0 if invalid.
        """
        h, w = depth_img.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return 0.0

        # Define a small neighborhood and take median of valid values
        half = 2
        u0 = max(0, u - half)
        v0 = max(0, v - half)
        u1 = min(w - 1, u + half)
        v1 = min(h - 1, v + half)

        patch = depth_img[v0 : v1 + 1, u0 : u1 + 1]
        flat = patch.reshape(-1)

        # Convert values depending on encoding
        encoding = (
            cam_info.depth_camera_info.encoding
            if hasattr(cam_info, "depth_camera_info")
            else cam_info.distortion_model
        )
        # Not all CameraInfo provide depth encoding; fallback to depth_img dtype
        enc_guess = ""
        if (
            hasattr(cam_info, "depth_camera_info")
            and cam_info.depth_camera_info.encoding
        ):
            enc_guess = cam_info.depth_camera_info.encoding
        else:
            # Guess based on dtype
            if flat.dtype == np.uint16:
                enc_guess = "16UC1"
            elif flat.dtype == np.float32 or flat.dtype == np.float64:
                enc_guess = "32FC1"
            else:
                enc_guess = "16UC1"

        # Convert each value and collect valid
        vals_m = []
        for val in flat:
            if np.isnan(val) or np.isinf(val):
                continue
            m = self._depth_to_meters(float(val), enc_guess)
            if m > 0.0:
                vals_m.append(m)

        if not vals_m:
            return 0.0
        return float(np.median(vals_m))

    @staticmethod
    def _project_to_3d(
        u: int, v: int, z: float, K: CameraIntrinsics
    ) -> Tuple[float, float, float]:
        """
        Back-project pixel (u, v) with depth z (meters) to 3D camera coordinates.
        """
        x = (u - K.cx) * z / K.fx
        y = (v - K.cy) * z / K.fy
        return float(x), float(y), float(z)

    @staticmethod
    def _bearing_from_pixel(u: int, K: CameraIntrinsics, frame_w: int) -> float:
        """
        Estimate horizontal bearing (rad) from pixel column using intrinsics:
        bearing ≈ atan2((u - cx), fx).
        """
        return float(math.atan2((u - K.cx), K.fx))

    def _wait_future(self, future, timeout: float) -> bool:
        """
        Wait for a future to complete with timeout.
        """
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            if future.done():
                return True
        return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FacePerceptionNode()
    try:
        rclpy.spin(node)
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
