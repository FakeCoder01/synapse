# Data for a single detected person
string person_id          # Unique ID (e.g., "person_1" or "uuid_...")
string name               # Known name (e.g., "Jane" or "Unknown")
bool is_known             # True if this person is in the database
int32[] bounding_box      # [x_min, y_min, x_max, y_max]
# Stub for future feature, can be empty for now
float32[] face_encoding
