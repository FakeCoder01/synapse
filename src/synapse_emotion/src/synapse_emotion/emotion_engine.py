#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project Synapse — Emotion Engine

A ROS 2 node that maintains and publishes the robot's global emotional state
as a simple finite-state machine with temporal decay. The state is published
as synapse_interfaces/msg/EmotionState on the /emotion/state topic.

Features:
- Discrete mood state (NEUTRAL, HAPPY, CURIOUS, SAD, ANXIOUS, ANNOYED, SURPRISED, TIRED, EXCITED)
- Continuous affect (valence [-1,1], arousal [0,1]) and intensity [0,1]
- Exponential decay of intensity/valence/arousal toward baseline
- Event-driven impulses via a simple string topic (/emotion/event)
- TTL hint derived from current intensity and decay parameters

Events (std_msgs/String on /emotion/event):
- "greet_known", "compliment"            -> HAPPY
- "user_seen", "novelty", "curious"      -> CURIOUS
- "low_battery"                          -> ANXIOUS
- "bump", "obstacle", "frustrated"       -> ANNOYED
- "idle", "sleepy"                        -> TIRED
- "surprised", "wow"                      -> SURPRISED
- "excited"                               -> EXCITED
- "sad"                                   -> SAD
- "neutral"                               -> NEUTRAL (reset bias)
- "sentiment:positive|neutral|negative"   -> affect-only nudge
- "set:STATE_LABEL" or "set:STATE_LABEL:intensity"
   STATE_LABEL ∈ {neutral,happy,curious,sad,anxious,annoyed,surprised,tired,excited}

Parameters:
- publish_rate_hz (double, default: 10.0)            : Publish frequency
- baseline_state (string, default: "neutral")        : Bias state used when decayed
- baseline_intensity (double, default: 0.20)         : Intensity floor for neutral engagement
- decay_half_life_s (double, default: 30.0)          : Intensity half-life (s) toward baseline_intensity
- affect_half_life_s (double, default: 20.0)         : Valence/arousal half-life toward 0.0
- event_topic (string, default: "/emotion/event")     : Event topic (std_msgs/String)
- state_topic (string, default: "/emotion/state")     : Output topic for EmotionState
- tags_retention_s (double, default: 10.0)            : How long to keep event tags before clearing

Notes:
- This is a minimal but functional engine. You can publish EmotionState as-is to a display node,
  and include state_label/intensity/valence/arousal in prompts for LLM conditioning.
- Extend by subscribing to additional signals (e.g., collision, battery, ASR sentiment)
  and translating them into event strings on /emotion/event.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from synapse_interfaces.msg import EmotionState


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def exp_half_life_decay(
    current: float, target: float, half_life_s: float, dt: float
) -> float:
    """
    Exponential decay toward a target using a half-life parameter.
    new = target + (current - target) * 0.5 ** (dt / half_life_s)
    """
    if half_life_s <= 1e-6:
        return target
    return target + (current - target) * (0.5 ** (dt / max(1e-6, half_life_s)))


def ttl_from_decay(
    current: float, target: float, half_life_s: float, threshold: float = 0.10
) -> float:
    """
    Estimate the remaining time (seconds) for current value to decay within a threshold
    of the target using half-life. If already <= threshold from target, return 0.
    """
    delta = abs(current - target)
    if delta <= threshold or half_life_s <= 1e-6:
        return 0.0
    # delta * (0.5 ** (t/half_life)) = threshold
    # 0.5 ** (t/half_life) = threshold / delta
    # (t/half_life) = log2(threshold / delta)
    # t = half_life * log2(threshold / delta)
    ratio = threshold / delta
    if ratio <= 0.0:
        return 0.0
    t = half_life_s * (math.log(ratio) / math.log(0.5))
    return max(0.0, t)


@dataclass
class MoodImpulse:
    new_state: (
        int | None
    )  # None to keep state unchanged; otherwise set to EmotionState enum
    d_intensity: float  # Additive nudge to intensity
    d_valence: float  # Additive nudge to valence
    d_arousal: float  # Additive nudge to arousal
    tags: List[str]
    cause: str


STATE_LABELS: Dict[int, str] = {
    EmotionState.NEUTRAL: "neutral",
    EmotionState.HAPPY: "happy",
    EmotionState.CURIOUS: "curious",
    EmotionState.SAD: "sad",
    EmotionState.ANXIOUS: "anxious",
    EmotionState.ANNOYED: "annoyed",
    EmotionState.SURPRISED: "surprised",
    EmotionState.TIRED: "tired",
    EmotionState.EXCITED: "excited",
}

LABEL_TO_STATE: Dict[str, int] = {v: k for k, v in STATE_LABELS.items()}


class EmotionEngine(Node):
    def __init__(self):
        super().__init__("emotion_engine")

        # Parameters
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("baseline_state", "neutral")
        self.declare_parameter("baseline_intensity", 0.20)
        self.declare_parameter("decay_half_life_s", 30.0)
        self.declare_parameter("affect_half_life_s", 20.0)
        self.declare_parameter("event_topic", "/emotion/event")
        self.declare_parameter("state_topic", "/emotion/state")
        self.declare_parameter("tags_retention_s", 10.0)

        self.publish_rate_hz: float = float(self.get_parameter("publish_rate_hz").value)
        baseline_label: str = (
            self.get_parameter("baseline_state").get_parameter_value().string_value
        )
        self.baseline_state: int = LABEL_TO_STATE.get(
            baseline_label.lower(), EmotionState.NEUTRAL
        )
        self.baseline_intensity: float = clamp(
            float(self.get_parameter("baseline_intensity").value), 0.0, 1.0
        )
        self.decay_half_life_s: float = float(
            self.get_parameter("decay_half_life_s").value
        )
        self.affect_half_life_s: float = float(
            self.get_parameter("affect_half_life_s").value
        )
        self.event_topic: str = (
            self.get_parameter("event_topic").get_parameter_value().string_value
        )
        self.state_topic: str = (
            self.get_parameter("state_topic").get_parameter_value().string_value
        )
        self.tags_retention_s: float = float(
            self.get_parameter("tags_retention_s").value
        )

        # Internal state
        self._state: int = self.baseline_state
        self._intensity: float = self.baseline_intensity
        self._valence: float = 0.0
        self._arousal: float = 0.2  # slight baseline alertness
        self._tags: List[Tuple[str, float]] = []  # (tag, timestamp)
        self._cause: str = "init"
        self._last_update_t: float = time.time()

        # Publishers and Subscribers
        self.pub_state = self.create_publisher(EmotionState, self.state_topic, 10)
        self.sub_event = self.create_subscription(
            String, self.event_topic, self._on_event, 10
        )

        # Timer
        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "EmotionEngine ready:"
            f"\n  publish_rate_hz: {self.publish_rate_hz:.2f}"
            f"\n  baseline_state: {STATE_LABELS[self.baseline_state]}"
            f"\n  baseline_intensity: {self.baseline_intensity:.2f}"
            f"\n  decay_half_life_s: {self.decay_half_life_s:.1f}"
            f"\n  affect_half_life_s: {self.affect_half_life_s:.1f}"
            f"\n  event_topic: {self.event_topic}"
            f"\n  state_topic: {self.state_topic}"
        )

    # -------------------------
    # Event handling
    # -------------------------
    def _on_event(self, msg: String) -> None:
        """
        Handle string events. Format examples:
        - "greet_known", "compliment", "user_seen", "low_battery", "bump", "idle"
        - "sentiment:positive" | "sentiment:neutral" | "sentiment:negative"
        - "set:happy" | "set:anxious:0.7"
        """
        payload = (msg.data or "").strip()
        if not payload:
            return

        # Direct set command
        if payload.lower().startswith("set:"):
            self._handle_set_command(payload[4:])
            return

        # Sentiment nudge
        if payload.lower().startswith("sentiment:"):
            self._handle_sentiment(payload.split(":", 1)[1].strip().lower())
            return

        impulse = self._event_to_impulse(payload.lower())
        self._apply_impulse(impulse)

    def _handle_set_command(self, arg: str) -> None:
        """
        Parse "STATE" or "STATE:intensity" and set state/intensity directly.
        """
        parts = [p.strip().lower() for p in arg.split(":") if p.strip()]
        if not parts:
            return
        label = parts[0]
        new_state = LABEL_TO_STATE.get(label)
        if new_state is None:
            self.get_logger().warn(f"Unknown state label in set command: {label}")
            return
        if len(parts) > 1:
            try:
                new_intensity = clamp(float(parts[1]), 0.0, 1.0)
            except Exception:
                new_intensity = 0.6
        else:
            new_intensity = 0.6

        self._state = int(new_state)
        self._intensity = float(new_intensity)
        # Set affect roughly aligned with the mood
        v, a = self._default_affect_for_state(self._state)
        self._valence = float(v)
        self._arousal = float(a)
        self._cause = f"set:{label}"
        self._push_tag(f"set:{label}")

    def _handle_sentiment(self, sentiment: str) -> None:
        """
        sentiment: "positive" | "neutral" | "negative"
        Only affects valence/arousal lightly; does not force a state change.
        """
        if sentiment == "positive":
            imp = MoodImpulse(
                new_state=None,
                d_intensity=0.05,
                d_valence=+0.20,
                d_arousal=+0.05,
                tags=["sentiment_positive"],
                cause="sentiment_positive",
            )
        elif sentiment == "negative":
            imp = MoodImpulse(
                new_state=None,
                d_intensity=0.05,
                d_valence=-0.25,
                d_arousal=+0.05,
                tags=["sentiment_negative"],
                cause="sentiment_negative",
            )
        else:
            imp = MoodImpulse(
                new_state=None,
                d_intensity=0.0,
                d_valence=0.0,
                d_arousal=0.0,
                tags=["sentiment_neutral"],
                cause="sentiment_neutral",
            )
        self._apply_impulse(imp)

    def _event_to_impulse(self, evt: str) -> MoodImpulse:
        """
        Map an event string to a mood impulse. Extend this mapping as needed.
        """
        mapping: Dict[str, MoodImpulse] = {
            "greet_known": MoodImpulse(
                EmotionState.HAPPY,
                0.35,
                +0.45,
                +0.20,
                ["social", "greet"],
                "greet_known",
            ),
            "compliment": MoodImpulse(
                EmotionState.HAPPY,
                0.30,
                +0.40,
                +0.15,
                ["social", "compliment"],
                "compliment",
            ),
            "user_seen": MoodImpulse(
                EmotionState.CURIOUS,
                0.25,
                +0.10,
                +0.20,
                ["social", "user_seen"],
                "user_seen",
            ),
            "novelty": MoodImpulse(
                EmotionState.CURIOUS, 0.20, +0.05, +0.25, ["novelty"], "novelty"
            ),
            "curious": MoodImpulse(
                EmotionState.CURIOUS, 0.20, +0.05, +0.25, ["curious"], "curious"
            ),
            "low_battery": MoodImpulse(
                EmotionState.ANXIOUS, 0.35, -0.40, +0.30, ["low_battery"], "low_battery"
            ),
            "bump": MoodImpulse(
                EmotionState.ANNOYED, 0.40, -0.55, +0.30, ["bump"], "bump"
            ),
            "obstacle": MoodImpulse(
                EmotionState.ANNOYED, 0.25, -0.30, +0.20, ["obstacle"], "obstacle"
            ),
            "frustrated": MoodImpulse(
                EmotionState.ANNOYED, 0.30, -0.45, +0.25, ["frustrated"], "frustrated"
            ),
            "idle": MoodImpulse(
                EmotionState.TIRED, 0.15, -0.05, -0.10, ["idle"], "idle"
            ),
            "sleepy": MoodImpulse(
                EmotionState.TIRED, 0.25, -0.10, -0.15, ["sleepy"], "sleepy"
            ),
            "surprised": MoodImpulse(
                EmotionState.SURPRISED, 0.35, +0.05, +0.45, ["surprised"], "surprised"
            ),
            "wow": MoodImpulse(
                EmotionState.SURPRISED, 0.30, +0.05, +0.40, ["wow"], "wow"
            ),
            "excited": MoodImpulse(
                EmotionState.EXCITED, 0.35, +0.35, +0.40, ["excited"], "excited"
            ),
            "sad": MoodImpulse(EmotionState.SAD, 0.30, -0.55, -0.20, ["sad"], "sad"),
            "neutral": MoodImpulse(
                EmotionState.NEUTRAL, 0.0, 0.0, 0.0, ["neutral"], "neutral"
            ),
        }
        return mapping.get(evt, MoodImpulse(None, 0.05, 0.0, 0.0, [evt], evt))

    def _default_affect_for_state(self, state: int) -> Tuple[float, float]:
        """
        Provide a default (valence, arousal) pair for the given discrete state.
        """
        if state == EmotionState.HAPPY:
            return (+0.6, 0.6)
        if state == EmotionState.CURIOUS:
            return (+0.2, 0.6)
        if state == EmotionState.SAD:
            return (-0.6, 0.3)
        if state == EmotionState.ANXIOUS:
            return (-0.5, 0.7)
        if state == EmotionState.ANNOYED:
            return (-0.5, 0.6)
        if state == EmotionState.SURPRISED:
            return (+0.1, 0.8)
        if state == EmotionState.TIRED:
            return (-0.2, 0.2)
        if state == EmotionState.EXCITED:
            return (+0.5, 0.8)
        # NEUTRAL
        return (0.0, 0.3)

    def _apply_impulse(self, imp: MoodImpulse) -> None:
        """
        Apply an impulse to the current mood/affect, clamped to valid ranges.
        """
        if imp.new_state is not None:
            self._state = int(imp.new_state)
            # When state changes, gently blend toward default affect for that state
            dv, da = self._default_affect_for_state(self._state)
            # Blend 50% toward default to avoid abrupt jumps if sentiment nudges were in play
            self._valence = clamp(0.5 * self._valence + 0.5 * dv, -1.0, 1.0)
            self._arousal = clamp(0.5 * self._arousal + 0.5 * da, 0.0, 1.0)

        self._intensity = clamp(self._intensity + imp.d_intensity, 0.0, 1.0)
        self._valence = clamp(self._valence + imp.d_valence, -1.0, 1.0)
        self._arousal = clamp(self._arousal + imp.d_arousal, 0.0, 1.0)
        self._cause = imp.cause
        for t in imp.tags:
            self._push_tag(t)

    def _push_tag(self, t: str) -> None:
        now = time.time()
        self._tags.append((t, now))
        # Keep only recent tags based on retention window
        self._tags = [
            (tag, ts) for (tag, ts) in self._tags if (now - ts) <= self.tags_retention_s
        ]

    # -------------------------
    # Update/Publish loop
    # -------------------------
    def _on_timer(self) -> None:
        now = time.time()
        dt = max(1e-6, now - self._last_update_t)
        self._last_update_t = now

        # Decay intensity toward baseline
        self._intensity = exp_half_life_decay(
            self._intensity, self.baseline_intensity, self.decay_half_life_s, dt
        )

        # Decay affect toward neutral (0)
        self._valence = exp_half_life_decay(
            self._valence, 0.0, self.affect_half_life_s, dt
        )
        self._arousal = exp_half_life_decay(
            self._arousal, 0.3, self.affect_half_life_s, dt
        )

        # If intensity is near baseline and affect is near neutral, drift discrete state back to baseline state
        if (
            abs(self._intensity - self.baseline_intensity) < 0.05
            and abs(self._valence) < 0.08
            and abs(self._arousal - 0.3) < 0.08
        ):
            self._state = self.baseline_state
            # Keep a mild cause indicating natural decay
            self._cause = "decay"

        # Cull old tags
        self._tags = [
            (tag, ts) for (tag, ts) in self._tags if (now - ts) <= self.tags_retention_s
        ]

        # Publish
        msg = EmotionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""  # unused

        msg.state = self._state
        msg.state_label = STATE_LABELS.get(self._state, "unknown")
        msg.intensity = float(self._intensity)
        msg.valence = float(self._valence)
        msg.arousal = float(self._arousal)
        msg.tags = [tag for (tag, _) in self._tags]
        msg.cause = self._cause

        # TTL hint: time to decay to within threshold of baseline_intensity
        msg.ttl_seconds = float(
            ttl_from_decay(
                self._intensity,
                self.baseline_intensity,
                self.decay_half_life_s,
                threshold=0.10,
            )
        )

        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmotionEngine()
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
