#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Project Synapse - Base Driver Node (Diff-Drive Skeleton)
#
# This ROS 2 node subscribes to /cmd_vel (geometry_msgs/msg/Twist) and translates these
# commands into low-level motor signals for a differential drive base. It also optionally
# publishes odometry (/odom) and TF (odom -> base_link), using either integrated commands
# (open-loop) or wheel encoders (closed-loop/feedback) when implemented.
#
# IMPORTANT:
# - This is a reference skeleton with a placeholder serial motor driver protocol.
# - Replace _send_motor_command(...) in MotorDriver with your controller's actual protocol.
# - If you have encoders, implement _read_encoders() to compute odometry from wheel ticks.
#
# Parameters:
#   cmd_vel_topic (string)       : Topic to subscribe for Twist commands (default: "/cmd_vel")
#   control_rate_hz (double)     : Control loop frequency in Hz (default: 50.0)
#   cmd_timeout (double)         : Seconds to stop if no cmd received (default: 0.5)
#
#   serial_port (string)         : Serial device path for motor controller (default: "/dev/ttyUSB0")
#   serial_baud (int)            : Serial baud rate (default: 115200)
#   invert_left_wheel (bool)     : Invert left wheel direction (default: false)
#   invert_right_wheel (bool)    : Invert right wheel direction (default: false)
#   debug_driver (bool)          : Log raw driver commands (default: false)
#
#   wheel_radius (double)        : Wheel radius in meters (default: 0.033)
#   wheel_separation (double)    : Distance between wheel centers (m) (default: 0.16)
#   max_linear_speed (double)    : Max linear speed of base (m/s) (default: 0.5)
#   max_angular_speed (double)   : Max angular speed of base (rad/s) (default: 2.0)
#   max_wheel_linear_speed (double): Max per-wheel surface speed (m/s) (default: 1.0)
#
#   publish_odom (bool)          : Publish nav_msgs/Odometry (default: true)
#   publish_tf (bool)            : Broadcast TF (odom->base_link) (default: true)
#   odom_source (string)         : "command" or "encoders" (default: "command")
#   odom_frame (string)          : Odometry frame id (default: "odom")
#   base_frame (string)          : Base frame id (default: "base_link")
#
#   has_encoders (bool)          : Whether encoders are available (default: false)
#   encoder_ticks_per_rev (int)  : Encoder ticks per wheel revolution (default: 2048)
#
# Topics:
#   Subscribes: /cmd_vel (geometry_msgs/msg/Twist)
#   Publishes : /odom (nav_msgs/msg/Odometry) [if publish_odom=true]
#   TF        : odom -> base_link [if publish_tf=true]
#
# License: Apache-2.0

import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert yaw (rad) to quaternion (x, y, z, w)."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class MotorDriver:
    """
    Placeholder motor driver using (optional) serial transport.

    Replace '_send_motor_command' with your actual motor controller protocol.
    For example:
      - Roboclaw
      - Cytron / Arduino-based custom firmware
      - ODrive
      - CAN-based controllers via a separate bridge

    If 'pyserial' isn't available or serial connection fails, the driver will operate
    in "mock" mode and simply log commands (if debug_driver is true).
    """

    def __init__(
        self,
        node: Node,
        port: str,
        baud: int,
        invert_left: bool = False,
        invert_right: bool = False,
        debug: bool = False,
    ):
        self._node = node
        self._port = port
        self._baud = baud
        self._invert_left = -1.0 if invert_left else 1.0
        self._invert_right = -1.0 if invert_right else 1.0
        self._debug = debug

        self._ser = None
        self._lock = threading.Lock()

        # Try to open serial (optional)
        try:
            import serial  # type: ignore

            self._ser = serial.Serial(
                port=self._port, baudrate=self._baud, timeout=0.02
            )
            self._node.get_logger().info(
                f"MotorDriver: Opened serial port {self._port} @ {self._baud} baud"
            )
        except Exception as exc:
            self._ser = None
            self._node.get_logger().warn(
                f"MotorDriver: Running in MOCK mode (serial unavailable): {exc}"
            )

    def set_wheel_rpm(self, left_rpm: float, right_rpm: float) -> None:
        """
        Set target RPMs for left and right wheels. Values can be negative.

        This function applies any invert flags and sends the resulting command to the driver.
        """
        l = left_rpm * self._invert_left
        r = right_rpm * self._invert_right
        with self._lock:
            self._send_motor_command(l, r)

    def stop(self) -> None:
        """Send a stop command (zero RPM) to both wheels."""
        with self._lock:
            self._send_motor_command(0.0, 0.0)

    def _send_motor_command(self, left_rpm: float, right_rpm: float) -> None:
        """
        Placeholder: Convert target RPMs to your controller's protocol and transmit.

        EXAMPLE PLACEHOLDER PROTOCOL:
          - ASCII line: "M <left_rpm> <right_rpm>\n"
          - Integers or floats depending on controller
          - Checksum or CRC if required by your device

        REPLACE THIS WITH YOUR ACTUAL DRIVER IMPLEMENTATION.
        """
        cmd = f"M {left_rpm:.2f} {right_rpm:.2f}\n"
        if self._ser:
            try:
                self._ser.write(cmd.encode("ascii"))
                # If your device requires flushing or CRLF, adjust here.
            except Exception as exc:
                self._node.get_logger().error(
                    f"MotorDriver: Serial write failed: {exc}"
                )
        if self._debug or not self._ser:
            self._node.get_logger().debug(f"[Driver TX] {cmd.strip()}")

    # Optional: implement encoder reading depending on your hardware
    def read_encoders(self) -> Optional[Tuple[int, int]]:
        """
        Placeholder: Read and return encoder tick counts since last read (delta ticks).

        Returns:
            (left_delta_ticks, right_delta_ticks) or None if not available.

        REPLACE THIS WITH YOUR ACTUAL DRIVER IMPLEMENTATION IF YOU HAVE ENCODERS.
        """
        # Example (mock): no encoders available
        return None


class BaseDriverNode(Node):
    def __init__(self):
        super().__init__("base_driver_node")

        # Parameters
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout", 0.5)

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_baud", 115200)
        self.declare_parameter("invert_left_wheel", False)
        self.declare_parameter("invert_right_wheel", False)
        self.declare_parameter("debug_driver", False)

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.16)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 2.0)
        self.declare_parameter("max_wheel_linear_speed", 1.0)

        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_source", "command")  # "command" or "encoders"
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("has_encoders", False)
        self.declare_parameter("encoder_ticks_per_rev", 2048)

        # Resolve parameters
        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        self.serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        self.serial_baud = int(self.get_parameter("serial_baud").value)
        self.invert_left_wheel = bool(self.get_parameter("invert_left_wheel").value)
        self.invert_right_wheel = bool(self.get_parameter("invert_right_wheel").value)
        self.debug_driver = bool(self.get_parameter("debug_driver").value)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_wheel_linear_speed = float(
            self.get_parameter("max_wheel_linear_speed").value
        )

        self.publish_odom = bool(self.get_parameter("publish_odom").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_source = (
            self.get_parameter("odom_source").get_parameter_value().string_value
        )
        self.odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self.base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )

        self.has_encoders = bool(self.get_parameter("has_encoders").value)
        self.encoder_ticks_per_rev = int(
            self.get_parameter("encoder_ticks_per_rev").value
        )

        if self.wheel_radius <= 0.0 or self.wheel_separation <= 0.0:
            self.get_logger().warn(
                "wheel_radius and wheel_separation must be > 0; clamping to minimums"
            )
            self.wheel_radius = max(1e-6, self.wheel_radius)
            self.wheel_separation = max(1e-6, self.wheel_separation)

        # Motor driver
        self.driver = MotorDriver(
            node=self,
            port=self.serial_port,
            baud=self.serial_baud,
            invert_left=self.invert_left_wheel,
            invert_right=self.invert_right_wheel,
            debug=self.debug_driver,
        )

        # State
        self._last_cmd_time = 0.0
        self._cmd_vx = 0.0
        self._cmd_wz = 0.0

        # Odometry state (meters, radians)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_odom_time = self.get_clock().now()

        # Publishers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, "odom", qos)
        else:
            self.odom_pub = None

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, qos)

        # Control loop timer
        period = 1.0 / self.control_rate_hz if self.control_rate_hz > 0.0 else 0.02
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"BaseDriverNode started:\n"
            f"  cmd_vel_topic: {self.cmd_vel_topic}\n"
            f"  control_rate_hz: {self.control_rate_hz:.1f}\n"
            f"  timeout: {self.cmd_timeout:.2f}s\n"
            f"  serial: {self.serial_port} @ {self.serial_baud}\n"
            f"  wheel_radius: {self.wheel_radius:.4f} m, separation: {self.wheel_separation:.4f} m\n"
            f"  max_linear: {self.max_linear_speed:.2f} m/s, max_angular: {self.max_angular_speed:.2f} rad/s\n"
            f"  max_wheel_linear: {self.max_wheel_linear_speed:.2f} m/s\n"
            f"  publish_odom: {self.publish_odom}, publish_tf: {self.publish_tf}, odom_source: {self.odom_source}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Clamp incoming commands to configured limits
        vx = clamp(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        wz = clamp(msg.angular.z, -self.max_angular_speed, self.max_angular_speed)

        self._cmd_vx = vx
        self._cmd_wz = wz
        self._last_cmd_time = time.time()

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_odom_time).nanoseconds * 1e-9
        dt = max(dt, 1e-9)  # avoid zero

        # Safety: stop if stale command
        if (time.time() - self._last_cmd_time) > self.cmd_timeout:
            vx, wz = 0.0, 0.0
        else:
            vx, wz = self._cmd_vx, self._cmd_wz

        # Convert base velocities to wheel linear velocities (m/s)
        v_left = vx - (wz * self.wheel_separation * 0.5)
        v_right = vx + (wz * self.wheel_separation * 0.5)

        # Scale if exceeds per-wheel limit
        max_abs = max(abs(v_left), abs(v_right), 1e-9)
        if max_abs > self.max_wheel_linear_speed:
            scale = self.max_wheel_linear_speed / max_abs
            v_left *= scale
            v_right *= scale

        # Convert wheel linear velocities to RPM
        # wheel_rpm = (v / (2*pi*r)) * 60
        left_rpm = (v_left / (2.0 * math.pi * self.wheel_radius)) * 60.0
        right_rpm = (v_right / (2.0 * math.pi * self.wheel_radius)) * 60.0

        # Send to motor driver
        self.driver.set_wheel_rpm(left_rpm, right_rpm)

        # Odometry update
        if self.publish_odom:
            if self.odom_source.lower() == "encoders" and self.has_encoders:
                self._update_odom_from_encoders(dt)
            else:
                # Open-loop: integrate commanded velocities
                self._update_odom_from_command(vx, wz, dt)

            # Publish odometry and TF
            self._publish_odom(now, vx, wz)
            if self.publish_tf:
                self._broadcast_tf(now)

        self._last_odom_time = now

    def _update_odom_from_command(self, vx: float, wz: float, dt: float) -> None:
        # Simple differential-drive kinematics integration in world frame
        if abs(wz) < 1e-6:
            dx = vx * dt
            dy = 0.0
            dth = 0.0
        else:
            dx = vx * math.cos(self._yaw) * dt
            dy = vx * math.sin(self._yaw) * dt
            dth = wz * dt

        self._x += dx
        self._y += dy
        self._yaw += dth
        # Normalize yaw to [-pi, pi]
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

    def _update_odom_from_encoders(self, dt: float) -> None:
        """
        Closed-loop odometry using wheel encoders (placeholder).

        Implement your protocol to obtain delta ticks for left and right wheels since last read.
        Convert ticks to traveled distance per wheel, then update pose.

        For example:
            delta_L, delta_R = self.driver.read_encoders()
            if delta_L is None: fall back to command integration
            d_left_m = 2*pi*R * (delta_L / ticks_per_rev)
            d_right_m = 2*pi*R * (delta_R / ticks_per_rev)
            d_center = (d_left_m + d_right_m) / 2
            d_theta = (d_right_m - d_left_m) / wheel_separation

            x += d_center * cos(theta)
            y += d_center * sin(theta)
            theta += d_theta
        """
        enc = self.driver.read_encoders()
        if enc is None:
            # Fallback to open-loop if encoders not available
            vx = (
                self._cmd_vx
                if (time.time() - self._last_cmd_time) <= self.cmd_timeout
                else 0.0
            )
            wz = (
                self._cmd_wz
                if (time.time() - self._last_cmd_time) <= self.cmd_timeout
                else 0.0
            )
            self._update_odom_from_command(vx, wz, dt)
            return

        delta_L, delta_R = enc
        ticks_rev = max(1, self.encoder_ticks_per_rev)
        c = 2.0 * math.pi * self.wheel_radius
        d_left = c * (float(delta_L) / float(ticks_rev))
        d_right = c * (float(delta_R) / float(ticks_rev))

        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.wheel_separation

        self._x += d_center * math.cos(self._yaw + 0.5 * d_theta)
        self._y += d_center * math.sin(self._yaw + 0.5 * d_theta)
        self._yaw += d_theta
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

    def _publish_odom(self, now, vx: float, wz: float) -> None:
        if self.odom_pub is None:
            return

        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(self._yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Covariances can be tuned per platform
        # For brevity, leave zeros; downstream filters (e.g., EKF) can set their own
        # odom.pose.covariance[...] = ...

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

    def _broadcast_tf(self, now) -> None:
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self._yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self) -> bool:
        try:
            self.get_logger().info("BaseDriverNode: Stopping motors...")
            self.driver.stop()
        except Exception as exc:
            self.get_logger().warn(
                f"BaseDriverNode: Failed to stop motors on shutdown: {exc}"
            )
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriverNode()
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
