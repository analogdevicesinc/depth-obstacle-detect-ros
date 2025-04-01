# depth_obstacle_detect_ros_msgs
Custom ROS Message package dependency for depth_obstacle_detect_ros package detection state output.
## Background
The Message contains three data fields. The header contains the header information for the messages. The `obstacle_detected` variable is an array of booleans of whether or not a certain cell in the in the grid has an obstacle. The `obstacle_distance` variable is an array of 64 bit floating point values that stores the minimum distance of each cell in the grid.
## Message Structure
```
std_msgs/Header header
bool[] obstacle_detected
float64[] obstacle_distance
```