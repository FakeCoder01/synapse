#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project Synapse — Face Display Node (terminal/ASCII placeholder)

A backend-agnostic display node that renders simple ASCII "faces" in the terminal
based on the robot's current EmotionState. This is a lightweight placeholder that
you can run on any platform without graphics libraries while you integrate a real
display (OLED/LCD/HDMI).

Features:
- Subscribes to /emotion/state (synapse_interfaces/msg/EmotionState)
- Renders an ASCII face reflecting discrete state, intensity, valence, and arousal
- Optional colored output using ANSI escape codes
- Works even if stdout is not a TTY (prints compact status lines)

Parameters:
- state_topic (string): Topic to subscribe for emotion state (default: "/emotion/state")
- refresh_rate_hz (double): Refresh rate for rendering (default: 10.0)
- use_color (bool): Enable ANSI color output (default: true)
- fullscreen (bool): Clear screen and redraw on each frame (default: true)
- show_debug (bool): Show debug telemetry (default: false)

Controls:
- Press Ctrl+C to exit.

Note:
This node is intended as a simple development-time stand-in. For production,
replace with a driver that renders to your actual screen hardware.
"""

import os
import sys
import time
import shutil
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from synapse_interfaces.msg import EmotionState
from std_msgs.msg import String

ANSI_RESET = "\033[0m"
ANSI_BOLD = "\033[1m"

COLOR_MAP = {
    "neutral": "\033[37m",  # white/grey
    "happy": "\033[33m",  # yellow
    "curious": "\033[36m",  # cyan
    "sad": "\033[34m",  # blue
    "anxious": "\033[35m",  # magenta
    "annoyed": "\033[31m",  # red
    "surprised": "\033[93m",  # bright yellow
    "tired": "\033[90m",  # bright black (grey)
    "excited": "\033[95m",  # bright magenta
    "unknown": "\033[37m",
}


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def clear_screen():
    sys.stdout.write("\033[2J\033[H")  # clear + move cursor home


def move_cursor_home():
    sys.stdout.write("\033[H")


def center_block(lines: List[str], width: int) -> List[str]:
    out = []
    for line in lines:
        pad = max(0, (width - len(line)) // 2)
        out.append(" " * pad + line)
    return out


def face_glyphs_for_state(
    state_label: str, intensity: float, valence: float, arousal: float
) -> Tuple[str, str, str]:
    """
    Return glyphs (left_eye, mouth, right_eye) based on mood.
    """
    # Bound values
    intensity = clamp(intensity, 0.0, 1.0)
    valence = clamp(valence, -1.0, 1.0)
    arousal = clamp(arousal, 0.0, 1.0)

    # Eyes
    if state_label == "happy":
        left_eye, right_eye = "^", "^"
    elif state_label == "curious":
        left_eye, right_eye = "o", "•"
    elif state_label == "sad":
        left_eye, right_eye = "•", "•"
    elif state_label == "anxious":
        left_eye, right_eye = "o", "o"
    elif state_label == "annoyed":
        left_eye, right_eye = "¬", "¬"
    elif state_label == "surprised":
        left_eye, right_eye = "O", "O"
    elif state_label == "tired":
        left_eye, right_eye = "-", "-"
    elif state_label == "excited":
        left_eye, right_eye = "*", "*"
    else:  # neutral/unknown
        left_eye, right_eye = "•", "•"

    # Mouth curvature/shape selection by state (influenced by valence/arousal/intensity)
    mouth_width = 4 + int(6 * intensity)  # 4..10
    mouth_width = clamp(mouth_width, 3, 12)

    if state_label in ("happy", "excited"):
        # Upturned smile, bigger with intensity
        curve = "_" if intensity < 0.4 else "‿"
        mouth = curve * max(1, int(mouth_width * (0.6 if curve == "_" else 0.8)))
    elif state_label in ("curious",):
        # Side smirk influenced by valence
        if valence >= 0:
            mouth = "_" * (mouth_width - 1) + "."
        else:
            mouth = "." + "_" * (mouth_width - 1)
    elif state_label in ("sad",):
        # Downturned (simulate by parentheses)
        mouth = "(" + "_" * max(1, mouth_width - 2) + ")"
    elif state_label in ("anxious", "annoyed", "tired"):
        # Flat/straight mouth with slight variation by arousal
        mouth = "_" * mouth_width if arousal < 0.5 else "-" * mouth_width
    elif state_label in ("surprised",):
        # Open mouth
        radius = max(1, int(1 + 3 * intensity))
        mouth = "O" * radius
    else:
        # neutral/unknown
        mouth = "-" * mouth_width

    return left_eye, mouth, right_eye


def build_face_lines(
    state_label: str, intensity: float, valence: float, arousal: float
) -> List[str]:
    """
    Construct a 2D ASCII face as a list of lines.
    """
    left_eye, mouth, right_eye = face_glyphs_for_state(
        state_label, intensity, valence, arousal
    )

    # Eye spacing increases with intensity (more expressive)
    eye_gap = 5 + int(4 * intensity)  # 5..9
    eye_row = f"{left_eye}" + " " * eye_gap + f"{right_eye}"

    # Cheeks/face edges
    # Compose a simple face box with eyes, nose (optional), and mouth
    top = "╭" + "─" * (len(eye_row) + 4) + "╮"
    empty = "│ " + " " * (len(eye_row) + 2) + " │"
    eyes = "│  " + eye_row + "  │"

    # Nose minimal (optional)
    nose_char = "." if state_label in ("curious", "tired") else "•"
    nose_space_left = len(eye_row) // 2
    nose = (
        "│  "
        + " " * (nose_space_left)
        + nose_char
        + " " * (len(eye_row) - nose_space_left - 1)
        + "  │"
    )

    # Mouth row centered
    mouth_pad = max(0, (len(eye_row) - len(mouth)) // 2)
    mouth_row = (
        "│  "
        + " " * mouth_pad
        + mouth
        + " " * (len(eye_row) - len(mouth) - mouth_pad)
        + "  │"
    )

    bottom = "╰" + "─" * (len(eye_row) + 4) + "╯"

    return [top, empty, eyes, empty, nose, empty, mouth_row, empty, bottom]


def summarize_state_line(state: EmotionState) -> str:
    tags = ", ".join(state.tags) if state.tags else ""
    return (
        f"state={state.state_label} "
        f"intensity={state.intensity:.2f} valence={state.valence:.2f} arousal={state.arousal:.2f} "
        f"ttl~{state.ttl_seconds:.1f}s cause={state.cause or '-'} "
        f"tags=[{tags}]"
    )


class FaceDisplayNode(Node):
    def __init__(self):
        super().__init__("face_display_node")

        # Parameters
        self.declare_parameter("state_topic", "/emotion/state")
        self.declare_parameter("refresh_rate_hz", 10.0)
        self.declare_parameter("use_color", True)
        self.declare_parameter("fullscreen", True)
        self.declare_parameter("show_debug", False)

        self.state_topic = (
            self.get_parameter("state_topic").get_parameter_value().string_value
        )
        self.refresh_rate_hz = float(self.get_parameter("refresh_rate_hz").value)
        self.use_color = bool(self.get_parameter("use_color").value)
        self.fullscreen = bool(self.get_parameter("fullscreen").value)
        self.show_debug = bool(self.get_parameter("show_debug").value)

        # State cache
        self._last_state = EmotionState()
        self._have_state = False

        # Subscriptions
        self.create_subscription(
            EmotionState, self.state_topic, self._on_emotion_state, 10
        )

        # Timer for redraws
        period = 1.0 / max(1e-3, self.refresh_rate_hz)
        self.create_timer(period, self._on_timer)

        # Terminal detection
        self._is_tty = sys.stdout.isatty()

        self.get_logger().info(
            "face_display_node ready:"
            f"\n  state_topic: {self.state_topic}"
            f"\n  refresh_rate_hz: {self.refresh_rate_hz:.1f}"
            f"\n  use_color: {self.use_color}"
            f"\n  fullscreen: {self.fullscreen}"
            f"\n  tty: {self._is_tty}"
        )

    def _on_emotion_state(self, msg: EmotionState):
        self._last_state = msg
        self._have_state = True

    def _on_timer(self):
        if not self._have_state:
            # No state yet; draw placeholder
            self._render_placeholder()
            return

        s = self._last_state
        label = s.state_label or "neutral"
        intensity = clamp(s.intensity, 0.0, 1.0)
        valence = clamp(s.valence, -1.0, 1.0)
        arousal = clamp(s.arousal, 0.0, 1.0)

        if not self._is_tty:
            # Fallback: print compact line (no redraw control)
            sys.stdout.write(summarize_state_line(s) + "\n")
            sys.stdout.flush()
            return

        # Build face
        face_lines = build_face_lines(label, intensity, valence, arousal)

        # Terminal geometry
        try:
            cols = shutil.get_terminal_size((80, 24)).columns
        except Exception:
            cols = 80

        # Center and colorize
        face_lines = center_block(face_lines, cols)
        color = COLOR_MAP.get(label, COLOR_MAP["unknown"]) if self.use_color else ""
        reset = ANSI_RESET if self.use_color else ""

        # Compose info lines
        info_1 = summarize_state_line(s)
        info_2 = f"Press Ctrl+C to exit. Rendering at {self.refresh_rate_hz:.1f} Hz."

        info_1 = (color + info_1 + reset) if self.use_color else info_1
        info_2 = (color + info_2 + reset) if self.use_color else info_2

        info_block = center_block([info_1, info_2], cols)

        # Render
        if self.fullscreen:
            clear_screen()
        else:
            move_cursor_home()

        # Print face
        for line in face_lines:
            sys.stdout.write((color + line + reset) + "\n")
        # Spacer and info
        sys.stdout.write("\n")
        for line in info_block:
            sys.stdout.write(line + "\n")

        if self.show_debug:
            # Additional debug block aligned left
            sys.stdout.write("\n")
            sys.stdout.write("DEBUG:\n")
            sys.stdout.write(f"  label={label}\n")
            sys.stdout.write(f"  intensity={intensity:.3f}\n")
            sys.stdout.write(f"  valence={valence:.3f}\n")
            sys.stdout.write(f"  arousal={arousal:.3f}\n")

        sys.stdout.flush()

    def _render_placeholder(self):
        if not self._is_tty:
            sys.stdout.write("(face_display_node) waiting for /emotion/state...\n")
            sys.stdout.flush()
            return

        if self.fullscreen:
            clear_screen()
        else:
            move_cursor_home()

        try:
            cols = shutil.get_terminal_size((80, 24)).columns
        except Exception:
            cols = 80

        lines = [
            "╭─────────────────────────╮",
            "│   Project Synapse Face  │",
            "│     waiting for state   │",
            "╰─────────────────────────╯",
        ]
        for line in center_block(lines, cols):
            sys.stdout.write(line + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDisplayNode()
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
