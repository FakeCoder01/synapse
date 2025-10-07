#!/usr/bin/env python3
#
# Project Synapse - Keyboard Teleoperation Node
# Publishes geometry_msgs/Twist on /cmd_vel to manually drive the robot.
#
# Controls (default):
#   Movement:
#     w/W or Up Arrow     -> forward
#     s/S or Down Arrow   -> backward
#     a/A or Left Arrow   -> rotate left (CCW)
#     d/D or Right Arrow  -> rotate right (CW)
#     space or x          -> stop (zero velocities)
#
#   Speed scaling:
#     + or =              -> increase linear and angular scale by 10%
#     - or _              -> decrease linear and angular scale by 10%
#     ]                   -> increase angular scale by 10%
#     [                   -> decrease angular scale by 10%
#
#   Other:
#     h or ?              -> print help
#     q                   -> quit
#
# Parameters:
#   cmd_vel_topic (string): Topic to publish Twist on. Default: "/cmd_vel"
#   linear_scale (double):  Max linear speed (m/s). Default: 0.3
#   angular_scale (double): Max angular speed (rad/s). Default: 1.2
#   repeat_rate (double):   Publish rate (Hz). Default: 20.0
#   stop_on_key_release (bool): If true, stop when no key is pressed within deadman_timeout. Default: true
#   deadman_timeout (double): Seconds after last key press to stop if stop_on_key_release is true. Default: 0.5
#   enable_arrow_keys (bool): Enable arrow key handling. Default: true
#
# Cross-platform key capture (Linux/macOS via termios; Windows via msvcrt).
#
# License: Apache-2.0
#

import sys
import time
import threading
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Cross-platform key input helpers
_IS_WINDOWS = sys.platform.startswith('win')

if _IS_WINDOWS:
    try:
        import msvcrt  # type: ignore
    except Exception:
        msvcrt = None
else:
    import termios
    import tty
    import select

HELP_TEXT = """
Keyboard Teleop Controls
------------------------
Movement:
  w / Up Arrow     : forward
  s / Down Arrow   : backward
  a / Left Arrow   : rotate left (CCW)
  d / Right Arrow  : rotate right (CW)
  SPACE or x       : stop (zero velocities)

Speed scaling:
  + or =           : increase linear & angular by 10%
  - or _           : decrease linear & angular by 10%
  ]                : increase angular only by 10%
  [                : decrease angular only by 10%

Other:
  h or ?           : print this help
  q                : quit

Notes:
- Uppercase movement keys (W/A/S/D) apply a temporary boost multiplier.
- If stop_on_key_release is true, the robot stops when you stop pressing keys for deadman_timeout seconds.

"""


def clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


class _KeyReader:
    """
    Non-blocking key reader with timeout.
    - On POSIX: uses termios, tty, and select to read with timeout.
    - On Windows: uses msvcrt.kbhit() and msvcrt.getwch().
    """

    def __init__(self, enable_arrow_keys: bool = True):
        self.enable_arrow_keys = enable_arrow_keys
        self._posix_old_settings = None

    def __enter__(self):
        if not _IS_WINDOWS:
            self._posix_old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if not _IS_WINDOWS and self._posix_old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._posix_old_settings)
            self._posix_old_settings = None

    def get_key(self, timeout: float) -> Optional[str]:
        """
        Returns:
          - a single-character string if a key was read
          - 'UP', 'DOWN', 'LEFT', 'RIGHT' for arrow keys if enabled and detected
          - None if no key within timeout
        """
        if _IS_WINDOWS:
            if msvcrt is None:
                time.sleep(timeout)
                return None
            end_time = time.time() + timeout
            while time.time() < end_time:
                if msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    # Handle arrow keys: first char is 0 or 224, then a code
                    if self.enable_arrow_keys and (ch in ('\x00', '\xe0')):
                        code = msvcrt.getwch()
                        arrows = {
                            'H': 'UP',    # 72
                            'P': 'DOWN',  # 80
                            'K': 'LEFT',  # 75
                            'M': 'RIGHT', # 77
                        }
                        return arrows.get(code, None)
                    return ch
                time.sleep(0.01)
            return None
        else:
            # POSIX
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                ch = sys.stdin.read(1)
                if self.enable_arrow_keys and ch == '\x1b':
                    # Possible arrow sequence: ESC [ A/B/C/D
                    if self._wait_next(0.0005):
                        ch2 = sys.stdin.read(1)
                        if ch2 == '[' and self._wait_next(0.0005):
                            ch3 = sys.stdin.read(1)
                            if ch3 == 'A':
                                return 'UP'
                            if ch3 == 'B':
                                return 'DOWN'
                            if ch3 == 'C':
                                return 'RIGHT'
                            if ch3 == 'D':
                                return 'LEFT'
                    return None
                return ch
            return None

    def _wait_next(self, timeout: float) -> bool:
        if _IS_WINDOWS:
            return False
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        return bool(rlist)


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_scale', 0.30)   # m/s
        self.declare_parameter('angular_scale', 1.20)  # rad/s
        self.declare_parameter('repeat_rate', 20.0)    # Hz
        self.declare_parameter('stop_on_key_release', True)
        self.declare_parameter('deadman_timeout', 0.5)  # s
        self.declare_parameter('enable_arrow_keys', True)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.repeat_rate = float(self.get_parameter('repeat_rate').value)
        self.stop_on_key_release = bool(self.get_parameter('stop_on_key_release').value)
        self.deadman_timeout = float(self.get_parameter('deadman_timeout').value)
        self.enable_arrow_keys = bool(self.get_parameter('enable_arrow_keys').value)

        # Publisher
        self.pub_twist = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Internal state
        self._last_key_time = 0.0
        self._current_cmd = Twist()
        self._lock = threading.Lock()

        # Timers
        self.timer = self.create_timer(1.0 / self.repeat_rate, self._on_timer)

        # Start key reading thread
        self._stop_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._key_reader_thread, daemon=True)
        self._reader_thread.start()

        self._print_banner()

    def _print_banner(self):
        banner = (
            "Keyboard Teleop Started\n"
            f"  Publishing to: {self.cmd_vel_topic}\n"
            f"  linear_scale: {self.linear_scale:.2f} m/s, angular_scale: {self.angular_scale:.2f} rad/s\n"
            f"  repeat_rate: {self.repeat_rate:.1f} Hz, stop_on_key_release: {self.stop_on_key_release}\n"
            "Press 'h' or '?' for help, 'q' to quit.\n"
        )
        print(banner, flush=True)

    def _on_timer(self):
        # Deadman stop if configured and timeout exceeded
        with self._lock:
            cmd = Twist()
            if not self.stop_on_key_release or (time.time() - self._last_key_time) <= self.deadman_timeout:
                cmd = self._current_cmd
            # else default zero cmd
            self.pub_twist.publish(cmd)

    def _key_reader_thread(self):
        boost_multiplier = 1.8  # uppercase movement keys apply this multiplier temporarily
        help_last_print = 0.0

        with _KeyReader(enable_arrow_keys=self.enable_arrow_keys) as reader:
            while rclpy.ok() and not self._stop_event.is_set():
                # Smaller timeout than deadman to be responsive
                key = reader.get_key(timeout=0.05)
                now = time.time()

                if key is None:
                    continue

                # Normalize single-char keys to str
                if isinstance(key, str) and len(key) == 1:
                    ch = key
                else:
                    ch = key  # arrow keys come as 'UP', ...

                # Handle quit
                if ch == 'q':
                    print("Quitting keyboard teleop...", flush=True)
                    rclpy.shutdown()
                    break

                if ch in ('h', '?'):
                    print(HELP_TEXT, flush=True)
                    help_last_print = now
                    continue

                # Speed scaling keys
                if ch in ('+', '='):
                    self._adjust_scale(1.10, 1.10)
                    self._print_scales()
                    continue
                if ch in ('-', '_'):
                    self._adjust_scale(1/1.10, 1/1.10)
                    self._print_scales()
                    continue
                if ch == ']':
                    self._adjust_scale(1.0, 1.10)
                    self._print_scales()
                    continue
                if ch == '[':
                    self._adjust_scale(1.0, 1/1.10)
                    self._print_scales()
                    continue

                # Stop keys
                if ch in (' ', 'x'):
                    with self._lock:
                        self._current_cmd = Twist()
                        self._last_key_time = now
                    continue

                # Movement mapping
                lin = 0.0
                ang = 0.0
                temporary_boost = 1.0

                if ch in ('w', 'W', 'UP'):
                    lin = +self.linear_scale
                    temporary_boost = boost_multiplier if ch == 'W' else 1.0
                elif ch in ('s', 'S', 'DOWN'):
                    lin = -self.linear_scale
                    temporary_boost = boost_multiplier if ch == 'S' else 1.0
                elif ch in ('a', 'A', 'LEFT'):
                    ang = +self.angular_scale
                    temporary_boost = boost_multiplier if ch == 'A' else 1.0
                elif ch in ('d', 'D', 'RIGHT'):
                    ang = -self.angular_scale
                    temporary_boost = boost_multiplier if ch == 'D' else 1.0
                else:
                    # Unknown key: ignore but throttle help prints
                    if (now - help_last_print) > 2.0:
                        print("Unknown key. Press 'h' for help.", flush=True)
                        help_last_print = now
                    continue

                # Apply temporary boost to the command
                lin *= temporary_boost
                ang *= temporary_boost

                with self._lock:
                    cmd = Twist()
                    cmd.linear.x = lin
                    cmd.linear.y = 0.0
                    cmd.linear.z = 0.0
                    cmd.angular.x = 0.0
                    cmd.angular.y = 0.0
                    cmd.angular.z = ang
                    self._current_cmd = cmd
                    self._last_key_time = now

    def _adjust_scale(self, lin_factor: float, ang_factor: float):
        # Reasonable clamping to avoid runaway values
        self.linear_scale = clamp(self.linear_scale * lin_factor, 0.01, 5.0)
        self.angular_scale = clamp(self.angular_scale * ang_factor, 0.05, 10.0)

    def _print_scales(self):
        print(f"Scales -> linear: {self.linear_scale:.2f} m/s, angular: {self.angular_scale:.2f} rad/s", flush=True)

    def destroy_node(self):
        self._stop_event.set()
        try:
            if self._reader_thread.is_alive():
                self._reader_thread.join(timeout=0.2)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
