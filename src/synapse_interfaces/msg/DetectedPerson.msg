# synapse_interfaces/DetectedPerson.msg
# Represents a single detected person from perception, including identity hypothesis,
# image-space bounding box, and estimated 3D position relative to header.frame_id.

# Standard message header.
# - stamp: time of detection
# - frame_id: coordinate frame for position (e.g., "camera_link", "base_link", or "map")
std_msgs/Header header

# Short-term tracker ID assigned by the perception/tracking node.
int32 track_id

# Identity fields
# - is_known: true if matched against a known person in memory
# - person_id: stable, persistent identifier (e.g., UUID) if known, else empty string
# - name: resolved display name if known, else empty string
bool is_known
string person_id
string name

# Confidence scores in range [0.0, 1.0]
# - recognition_confidence: face recognition or identity match likelihood
# - speaking_confidence: probability the person is currently speaking (if available)
float32 recognition_confidence
float32 speaking_confidence

# 2D image-space bounding box (pixels) relative to the source image resolution.
# Set to 0 if not available.
uint32 bbox_x
uint32 bbox_y
uint32 bbox_width
uint32 bbox_height

# Estimated 3D position of the person's head/face center in the frame specified by header.frame_id.
# If computed from depth/camera, this is typically in meters.
geometry_msgs/Point position

# Convenience kinematics relative to the sensor/robot in header.frame_id
# - distance_m: Euclidean distance to position
# - bearing_rad: azimuthal bearing (left/right) in radians; positive is counterclockwise
float32 distance_m
float32 bearing_rad

# Optional perceived facial emotion of the person, if available.
# Examples: "happy", "neutral", "sad", "surprised", "angry", "fear", "disgust"
string perceived_emotion
float32 perceived_emotion_confidence
