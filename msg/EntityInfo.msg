string id
string type

time last_update_time
time creation_time

# Only has pose in case of shape
bool has_shape
geometry_msgs/Pose pose

# Custom data
string data

# Convex Hull
geometry_msgs/Point[] convex_hull
geometry_msgs/Point center_point
float32 z_min
float32 z_max

