# synapse_interfaces/EmotionState.msg
# Global emotional/mood state of the robot as maintained by the Emotion Engine.
# Consumers (e.g., Face Display, Conversation Manager) can use this to render expressions
# and modulate behavior and language generation.
#
# Conventions:
# - intensity in [0.0, 1.0]
# - valence in [-1.0, 1.0]  (negative = unpleasant, positive = pleasant)
# - arousal in [0.0, 1.0]   (calm to excited)
# - ttl_seconds is a hint for the remaining lifetime of the current state before decay/transition.
#
# Header:
# - stamp: time when this state snapshot was produced
# - frame_id: unused (set to empty)
std_msgs/Header header

# Primary, discrete mood enum. Use as a coarse-grained state for UI/behavior switches.
uint8 NEUTRAL=0
uint8 HAPPY=1
uint8 CURIOUS=2
uint8 SAD=3
uint8 ANXIOUS=4
uint8 ANNOYED=5
uint8 SURPRISED=6
uint8 TIRED=7
uint8 EXCITED=8

# Current primary mood.
uint8 state

# Optional human-readable label for `state` (e.g., "happy", "neutral").
string state_label

# Intensity of the current mood in [0.0, 1.0].
float32 intensity

# Continuous affect dimensions for finer control.
# valence in [-1.0, 1.0], arousal in [0.0, 1.0]
float32 valence
float32 arousal

# Free-form tags to annotate context (e.g., ["low_battery","bumped","user_john"])
string[] tags

# Short explanation for the current state (e.g., "greeted John", "bumped into chair").
string cause

# Optional time-to-live hint in seconds for current affect before significant decay/transition.
float32 ttl_seconds
