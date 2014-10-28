string id
string type

time last_update_time
time creation_time

# Only has pose in case of shape
bool has_shape
geometry_msgs/Pose pose

# Convex Hull
geometry_msgs/Point[] convex_hull
geometry_msgs/Point center_point
float32 z_min
float32 z_max

# Key value pairs
string[] string_keys
string[] string_values
string[] numeric_keys
float32[] numeric_values

