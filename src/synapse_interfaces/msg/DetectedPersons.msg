# synapse_interfaces/DetectedPersons.msg
# Represents a set of detected persons in the current frame, including an optional
# primary person-of-interest selected by heuristic (e.g., closest/center/speaking).
#
# Header:
# - stamp: time of detection set
# - frame_id: coordinate frame for all positions in this message (e.g., "camera_link", "base_link", or "map")
std_msgs/Header header

# List of detected persons for this frame
synapse_interfaces/DetectedPerson[] persons

# Index into `persons` for the selected primary person.
# -1 if no primary person is selected.
int32 primary_index

# Optional explanation for why the primary was selected (e.g., "closest", "centered", "speaking")
string primary_reason
