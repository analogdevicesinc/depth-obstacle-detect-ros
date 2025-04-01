<h1 style="text-align: center;" > Depth Obstacle Detection ROS</h1>

# Overview
The **Depth Obstacle Detection ROS** package for the Robot Operating System (ROS) identifies surfaces near the depth camera within a specified threshold. The image is divided into a grid of multiple cells, and the minimum value in each cell is detected. Values below the threshold are highlighted in red to indicate the presence of an obstacle. There are two packages in this installation, the core package `depth_obstacle_detect_ros` which is dependant on `depth_obstacle_detect_ros_msgs`.


[![Jazzy](https://img.shields.io/badge/-JAZZY-green?style=plastic&logo=ros)](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html) [![Ubuntu 24.04](https://img.shields.io/badge/-UBUNTU%2024.04-orange?style=plastic&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/noble/) [![License](https://img.shields.io/badge/License-BSD_3--Clause-purple.svg)](./LICENSE) ![x86_64](https://img.shields.io/badge/x86__64-blue?style=plastic&logo=intel&logoColor=white)

> [!NOTE]  
> This node requires a custom ros message package to be installed in the same workspace. It can be found [here.](./depth_obstacle_detect_ros_msgs)  

# Background
The obstacle detection node subscribes to `/tof_cam/rect/depth` by default. The launch file can be edited to make the node subscribe to a topic that publishes depth images from a depth camera. Image encoding with `mono16` or `16UC1` are supported by the node. The node divides an image into a grid of specified number of cells vertically and horizontally. The minimum value of each cell is taken into account to decide if there is an obstacle within the range specified by the threshold. The [depth_obstacle_detect_ros_msgs](depth_obstacle_detect_ros_msgs/README.md) is used to publish obstacle information for any external use.

# Hardware
 - Any Depth Camera, such as [EVAL-ADTF3175D-NXZ](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/eval-adtf3175.html) Module
 - A Desktop with Ubuntu 24.04, preferably with x86_64 architecture.


# Setting up topics
1. **Update Topic Names**: Ensure that the topic names in the launch file match the corresponding topics in your ROS environment.
2. **Set Namespace**: Define the desired namespace by setting the `namespace` field.
3. **Edit Parameters**: Modify the parameters in the launch file as needed to suit your specific requirements.

# Parameters
|Parameter Field|Type|Description|
|:---|:---:|---:|
|obstacle_range_limit|double|Detection Distance in meters|
|width_regions|int|Number of segments horizontally|
|height_regions|int|Number of segments vertically|
|depth_topic|string|Topic where depth images are published|
|camera_info_topic|string|Topic where camera information is published|
|detect_topic|string|Topic where images marking obstacles are published|
|obstacle_state_topic|string|Topic where obstacle detection true or false grid is published|
|cam_id|string|Camera ID for message Header information|
|verbose|bool|Output debug information|


# Launch Commands
The obstacle detection node can be run using:
`ros2 launch depth_obstacle_detect_ros depth_obstacle_detect_ros.launch` 

The obstacle detection component can be run using:
`ros2 launch depth_obstacle_detect_ros obstacle_detect_component.launch.py`

## Obstacle Detection output
<img src="./depth_obstacle_detect_ros/docs/images/obstacle_detection.png" alt="Output_Image" width="900"/>