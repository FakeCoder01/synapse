#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project Synapse — Behavior Manager

High-level autonomy that coordinates:
- Patrol among configurable waypoints (Nav2 goals)
- Seek-and-Greet: approach a known person at a safe distance, trigger greeting
- Person Following: follow a selected person by sending short, updated Nav2 goals

Inputs:
- synapse_interfaces/msg/DetectedPersons (/detected_persons)
- TF (to transform person position into map frame)
- Nav2 action server (/navigate_to_pose)

Outputs:
- Navigation goals to Nav2 (nav2_msgs/action/NavigateToPose)
- /behavior/state (std_msgs/String) with human-readable state
- /emotion/event (std_msgs/String) emits "greet_known" on greeting start

Parameters (examples with defaults):
- behavior.patrol_waypoints (list of dicts):
    [
      {name: "home", x: 0.0, y: 0.0, yaw_deg: 0.0},
      {name: "kitchen", x: 2.0, y: 1.0, yaw_deg: 90.0}
    ]
  If empty, a small default pattern is used.

- behavior.patrol_pause_sec: 3.0
- behavior.seek_enabled: true
- behavior.follow_enabled: true
- behavior.require_known_for_seek: true
- behavior.approach_stop_distance: 0.8
- behavior.greet_cooldown_sec: 60.0
- behavior.follow_distance: 1.2
- behavior.person_stale_timeout: 10.0
- behavior.detect_distance_threshold: 2.5
- behavior.detect_recognition_threshold: 0.50

- frames.map: "map"

Notes:
- Person positions are transformed to the 'map' frame if possible.
- Following works by sending updated waypoints near the target person.
- This node cooperates with Conversation Manager which also reacts to detections.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from synapse_interfaces.msg import DetectedPersons, DetectedPerson

import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time as RosTime


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw_deg: float = 0.0


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Return quaternion for a yaw angle (rad) in ENU."""
    q = Quaternion()
    half = 0.5 * yaw
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class BehaviorState:
    IDLE = "IDLE"
    PATROL = "PATROL"
    SEEK_GREET = "SEEK_GREET"
    FOLLOW = "FOLLOW"


class BehaviorManager(Node):
    def __init__(self):
        super().__init__("behavior_manager")

        # Parameters
        self.declare_parameter("behavior.patrol_waypoints", [])
        self.declare_parameter("behavior.patrol_pause_sec", 3.0)
        self.declare_parameter("behavior.seek_enabled", True)
        self.declare_parameter("behavior.follow_enabled", True)
        self.declare_parameter("behavior.require_known_for_seek", True)
        self.declare_parameter("behavior.approach_stop_distance", 0.8)
        self.declare_parameter("behavior.greet_cooldown_sec", 60.0)
        self.declare_parameter("behavior.follow_distance", 1.2)
        self.declare_parameter("behavior.person_stale_timeout", 10.0)
        self.declare_parameter("behavior.detect_distance_threshold", 2.5)
        self.declare_parameter("behavior.detect_recognition_threshold", 0.50)
        self.declare_parameter("frames.map", "map")

        # Resolve parameters
        self.map_frame: str = (
            self.get_parameter("frames.map").get_parameter_value().string_value
        )
        self.patrol_pause_sec: float = float(
            self.get_parameter("behavior.patrol_pause_sec").value
        )
        self.seek_enabled: bool = bool(
            self.get_parameter("behavior.seek_enabled").value
        )
        self.follow_enabled: bool = bool(
            self.get_parameter("behavior.follow_enabled").value
        )
        self.require_known_for_seek: bool = bool(
            self.get_parameter("behavior.require_known_for_seek").value
        )
        self.approach_stop_distance: float = float(
            self.get_parameter("behavior.approach_stop_distance").value
        )
        self.greet_cooldown_sec: float = float(
            self.get_parameter("behavior.greet_cooldown_sec").value
        )
        self.follow_distance: float = float(
            self.get_parameter("behavior.follow_distance").value
        )
        self.person_stale_timeout: float = float(
            self.get_parameter("behavior.person_stale_timeout").value
        )
        self.detect_distance_threshold: float = float(
            self.get_parameter("behavior.detect_distance_threshold").value
        )
        self.detect_recognition_threshold: float = float(
            self.get_parameter("behavior.detect_recognition_threshold").value
        )

        self.patrol_waypoints: List[Waypoint] = self._load_patrol_waypoints()

        # Publishers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_behavior_state = self.create_publisher(String, "/behavior/state", qos)
        self.pub_emotion_event = self.create_publisher(String, "/emotion/event", qos)

        # Subscriptions
        self.create_subscription(
            DetectedPersons, "/detected_persons", self._on_detected_persons, qos
        )

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3.0))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=True
        )

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._nav_connected = False
        self._connect_to_nav2()

        # Internal state
        self._state: str = (
            BehaviorState.PATROL if self.patrol_waypoints else BehaviorState.IDLE
        )
        self._state_lock = threading.Lock()
        self._last_state_msg_time = 0.0

        self._patrol_idx: int = 0
        self._last_patrol_goal_time: float = 0.0
        self._last_patrol_arrival_time: float = 0.0

        self._current_goal_handle = None
        self._current_goal_id: Optional[str] = None
        self._goal_lock = threading.Lock()

        self._last_known_people: Dict[str, Tuple[float, str, float, float, float]] = {}
        # person_id -> (timestamp, frame_id, x_map, y_map, z_map)
        self._last_seen_greet_ts: Dict[str, float] = {}

        self._target_person_id: Optional[str] = None
        self._target_person_name: str = ""
        self._last_follow_update: float = 0.0

        # Main timer: behavior evaluation loop
        self.timer = self.create_timer(0.5, self._on_timer)

        self._announce_state()

        self.get_logger().info(
            "BehaviorManager ready:\n"
            f"  State: {self._state}\n"
            f"  Waypoints: {len(self.patrol_waypoints)}\n"
            f"  seek_enabled={self.seek_enabled} follow_enabled={self.follow_enabled} "
            f"require_known_for_seek={self.require_known_for_seek}\n"
            f"  approach_stop_distance={self.approach_stop_distance:.2f}m follow_distance={self.follow_distance:.2f}m\n"
            f"  detect_distance_threshold={self.detect_distance_threshold:.2f}m recognition_threshold={self.detect_recognition_threshold:.2f}\n"
        )

    # ------------- Initialization helpers -------------

    def _load_patrol_waypoints(self) -> List[Waypoint]:
        param = self.get_parameter("behavior.patrol_waypoints").value
        waypoints: List[Waypoint] = []

        # Expecting a list of dicts with keys: name, x, y, yaw_deg
        if isinstance(param, list) and param:
            for idx, item in enumerate(param):
                try:
                    name = str(item.get("name", f"wp{idx}"))
                    x = float(item.get("x", 0.0))
                    y = float(item.get("y", 0.0))
                    yaw_deg = float(item.get("yaw_deg", 0.0))
                    waypoints.append(Waypoint(name=name, x=x, y=y, yaw_deg=yaw_deg))
                except Exception as exc:
                    self.get_logger().warn(f"Invalid waypoint at index {idx}: {exc}")
        else:
            # Default simple square patrol around origin
            waypoints = [
                Waypoint("home", 0.0, 0.0, 0.0),
                Waypoint("east", 1.5, 0.0, 0.0),
                Waypoint("north", 1.5, 1.5, 90.0),
                Waypoint("west", 0.0, 1.5, 180.0),
            ]
            self.get_logger().info(
                "No patrol_waypoints provided; using default pattern."
            )
        return waypoints

    def _connect_to_nav2(self) -> None:
        # Wait for Nav2 action server in background without blocking forever
        def _waiter():
            if self.nav_client.wait_for_server(timeout_sec=5.0):
                self._nav_connected = True
                self.get_logger().info(
                    "Connected to Nav2 NavigateToPose action server."
                )
            else:
                self.get_logger().warn("Nav2 NavigateToPose server not available yet.")

        threading.Thread(target=_waiter, daemon=True).start()

    # ------------- Subscriptions -------------

    def _on_detected_persons(self, msg: DetectedPersons) -> None:
        # Update last known positions for known persons
        now = self.get_clock().now()
        for p in msg.persons:
            if self.require_known_for_seek and not p.is_known:
                continue
            if p.is_known and (
                p.recognition_confidence >= self.detect_recognition_threshold
            ):
                # If we have a 3D position in some frame, try to transform to map
                pt_map = self._person_to_map_point(p)
                if pt_map is None:
                    continue
                x_m, y_m, z_m = pt_map
                self._last_known_people[p.person_id] = (
                    self._to_unix(now),
                    p.header.frame_id or "",
                    float(x_m),
                    float(y_m),
                    float(z_m),
                )

    # ------------- Timer / FSM -------------

    def _on_timer(self) -> None:
        with self._state_lock:
            state = self._state

        # Periodically announce state (throttled)
        self._announce_state(throttle_sec=5.0)

        if state == BehaviorState.IDLE:
            # Could add logic to transition to patrol if waypoints become available
            if self.patrol_waypoints and self._nav_connected:
                self._set_state(BehaviorState.PATROL)
            return

        if state == BehaviorState.PATROL:
            # If we have a greetable person, transition
            candidate = self._find_greet_candidate()
            if candidate and self.seek_enabled:
                self._target_person_id = candidate[0]
                self._target_person_name = candidate[1]
                self._set_state(BehaviorState.SEEK_GREET)
                self._cancel_nav_goal()
                return

            # Otherwise ensure we're navigating along patrol waypoints
            if not self._nav_connected:
                return
            if not self._has_active_goal():
                self._send_next_patrol_goal()

            return

        if state == BehaviorState.SEEK_GREET:
            if not self.seek_enabled:
                self._set_state(BehaviorState.PATROL)
                return

            # If we lost the target or it's stale, go back to patrol
            if not self._target_person_id or self._person_stale(self._target_person_id):
                self._set_state(BehaviorState.PATROL)
                self._cancel_nav_goal()
                return

            # Compute approach goal at safe stopping distance in front of the person
            goal_pose = self._compute_approach_goal(
                self._target_person_id, self.approach_stop_distance
            )
            if goal_pose is None:
                # If cannot compute, fallback to patrol
                self._set_state(BehaviorState.PATROL)
                self._cancel_nav_goal()
                return

            if not self._has_active_goal():
                self._send_nav_goal(goal_pose)
                # Fire a greet event immediately to Emotion Engine; Conversation Manager reacts to detections too
                self._publish_emotion_event("greet_known")
                # After greeting, if following enabled, transition to FOLLOW, else back to PATROL on goal result
                if self.follow_enabled:
                    # We'll transition to FOLLOW once we arrive or soon after (simplify by immediate transition)
                    self._set_state(BehaviorState.FOLLOW)
                else:
                    # We remain in SEEK_GREET until goal result or timeout; for simplicity transition to PATROL soon
                    self._set_state(BehaviorState.PATROL)
            return

        if state == BehaviorState.FOLLOW:
            if not self.follow_enabled:
                self._set_state(BehaviorState.PATROL)
                self._cancel_nav_goal()
                return

            if not self._target_person_id or self._person_stale(self._target_person_id):
                self._set_state(BehaviorState.PATROL)
                self._cancel_nav_goal()
                return

            # Update follow goal periodically or when person moved significantly
            now = time.time()
            if (now - self._last_follow_update) > 1.0:
                follow_goal = self._compute_approach_goal(
                    self._target_person_id, self.follow_distance
                )
                if follow_goal is not None:
                    # Cancel any existing goal to send a fresh one
                    self._cancel_nav_goal()
                    self._send_nav_goal(follow_goal)
                    self._last_follow_update = now
            return

    # ------------- Candidate selection -------------

    def _find_greet_candidate(self) -> Optional[Tuple[str, str]]:
        """Find a known person to greet based on distance and cooldown."""
        now_ts = time.time()
        best_id = None
        best_name = ""
        best_dist = float("inf")

        for person_id, (ts, _frame, x, y, _z) in self._last_known_people.items():
            age = now_ts - ts
            if age > self.person_stale_timeout:
                continue
            # Distance from robot unknown here; use absolute distance from map origin not meaningful.
            # Instead, greet if seen and not recently greeted; rely on approach step to compute proper goal.
            # Optionally filter by "could approximate distance" if we had robot pose; for simplicity, we skip.

            last_greet = self._last_seen_greet_ts.get(person_id, 0.0)
            if (now_ts - last_greet) < self.greet_cooldown_sec:
                continue

            # Use Euclidean distance to a nominal patrol center (0,0). This is a weak proxy; better is robot -> person.
            # Keep simple: pick most recently seen person (smaller age). Use distance filter if provided.
            # For now, treat all as candidates and pick the most recent.
            if age < best_dist:
                best_id = person_id
                best_name = ""  # name may be empty; Conversation Manager handles personalization
                best_dist = age

        if best_id is not None:
            self._last_seen_greet_ts[best_id] = now_ts
            return (best_id, best_name)
        return None

    # ------------- Navigation helpers -------------

    def _build_pose(self, x: float, y: float, yaw_rad: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quaternion(float(normalize_angle(yaw_rad)))
        return ps

    def _send_next_patrol_goal(self) -> None:
        if not self.patrol_waypoints:
            return
        wp = self.patrol_waypoints[self._patrol_idx % len(self.patrol_waypoints)]
        yaw = math.radians(wp.yaw_deg)
        goal = self._build_pose(wp.x, wp.y, yaw)
        self.get_logger().info(
            f"[PATROL] Navigating to waypoint {wp.name} ({wp.x:.2f},{wp.y:.2f},{wp.yaw_deg:.0f}°)"
        )
        self._send_nav_goal(goal)
        self._patrol_idx = (self._patrol_idx + 1) % len(self.patrol_waypoints)
        self._last_patrol_goal_time = time.time()

    def _send_nav_goal(self, pose: PoseStamped) -> None:
        if not self._nav_connected:
            self.get_logger().warn("Nav2 server not connected; cannot send goal.")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        def _feedback_cb(_fh, _fb):
            # Could add dynamic logic based on feedback
            pass

        with self._goal_lock:
            send_future = self.nav_client.send_goal_async(
                goal_msg, feedback_callback=_feedback_cb
            )

            def _goal_response_cb(fut):
                try:
                    goal_handle = fut.result()
                except Exception as exc:
                    self.get_logger().warn(f"NavigateToPose send failed: {exc}")
                    return
                if not goal_handle.accepted:
                    self.get_logger().warn("NavigateToPose goal rejected.")
                    return
                self._current_goal_handle = goal_handle
                self._current_goal_id = str(goal_handle)  # debug
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self._nav_result_cb)

            send_future.add_done_callback(_goal_response_cb)

    def _nav_result_cb(self, fut):
        with self._goal_lock:
            try:
                result = fut.result().result
                status = fut.result().status
            except Exception as exc:
                self.get_logger().warn(f"NavigateToPose result retrieval failed: {exc}")
                self._current_goal_handle = None
                self._current_goal_id = None
                return

            self._current_goal_handle = None
            self._current_goal_id = None

            # On patrol, pause briefly at each waypoint
            with self._state_lock:
                if self._state == BehaviorState.PATROL:
                    self._last_patrol_arrival_time = time.time()
                    # Pause handled in timer by waiting for patrol_pause_sec before sending next (implicitly by no active goal)
                # Other states can react as needed; in FOLLOW we continuously update goals, so no special handling here

    def _has_active_goal(self) -> bool:
        with self._goal_lock:
            return self._current_goal_handle is not None

    def _cancel_nav_goal(self) -> None:
        with self._goal_lock:
            if self._current_goal_handle is None:
                return
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._current_goal_handle = None
            self._current_goal_id = None

    # ------------- Person geometry / TF -------------

    def _person_to_map_point(
        self, p: DetectedPerson
    ) -> Optional[Tuple[float, float, float]]:
        """
        Transform person's position (in p.header.frame_id) to map frame.
        Returns (x, y, z) in map.
        """
        # If position is zero and no depth, skip
        if (abs(p.position.x) + abs(p.position.y) + abs(p.position.z)) < 1e-6:
            return None

        src_frame = p.header.frame_id or ""
        if not src_frame:
            return None

        pt = PointStamped()
        pt.header = p.header
        pt.point = p.position

        try:
            pt_map = self.tf_buffer.transform(
                pt, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.2)
            )
            return (float(pt_map.point.x), float(pt_map.point.y), float(pt_map.point.z))
        except (TransformException, Exception):
            return None

    def _compute_approach_goal(
        self, person_id: str, stop_distance: float
    ) -> Optional[PoseStamped]:
        """
        Compute a Nav2 goal at 'stop_distance' meters away from the person along the bearing from robot to person.
        Since we don't have the robot pose here, we approximate by standing behind the goal point with a small offset
        in the direction opposite to the map origin->person vector. This is a simplification — for better fidelity,
        integrate the current robot pose via TF (map->base_link) and compute a proper approach vector.
        """
        info = self._last_known_people.get(person_id)
        if info is None:
            return None
        _ts, _frame, x, y, _z = info

        # Vector from origin to person (simplified proxy)
        vx = x
        vy = y
        v_norm = math.sqrt(vx * vx + vy * vy)
        if v_norm < 1e-3:
            # Place stop distance toward +x
            gx = x - stop_distance
            gy = y
            yaw = 0.0
        else:
            ux = vx / v_norm
            uy = vy / v_norm
            # Step back from the person by stop_distance
            gx = x - ux * stop_distance
            gy = y - uy * stop_distance
            # Face the person
            yaw = math.atan2((y - gy), (x - gx))

        return self._build_pose(gx, gy, yaw)

    # ------------- State helpers -------------

    def _set_state(self, new_state: str) -> None:
        with self._state_lock:
            if new_state == self._state:
                return
            self.get_logger().info(f"Behavior state: {self._state} -> {new_state}")
            # On state exit actions
            if self._state in (BehaviorState.SEEK_GREET, BehaviorState.FOLLOW):
                self._cancel_nav_goal()

            self._state = new_state
            if new_state == BehaviorState.PATROL:
                # Clear target
                self._target_person_id = None
                self._target_person_name = ""
            self._announce_state()

    def _announce_state(self, throttle_sec: float = 0.0) -> None:
        now = time.time()
        if throttle_sec > 0.0 and (now - self._last_state_msg_time) < throttle_sec:
            return
        self._last_state_msg_time = now
        msg = String()
        with self._state_lock:
            msg.data = self._state
        self.pub_behavior_state.publish(msg)

    def _publish_emotion_event(self, event: str) -> None:
        if not event:
            return
        msg = String()
        msg.data = event
        self.pub_emotion_event.publish(msg)

    def _person_stale(self, person_id: str) -> bool:
        info = self._last_known_people.get(person_id)
        if info is None:
            return True
        ts, _frame, _x, _y, _z = info
        return (time.time() - ts) > self.person_stale_timeout

    @staticmethod
    def _to_unix(t: RosTime) -> float:
        try:
            return float(t.sec) + float(t.nanosec) * 1e-9
        except Exception:
            return time.time()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
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
