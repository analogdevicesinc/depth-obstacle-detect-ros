from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(ComposableNodeContainer(
        name='obstacle_detection_node',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_obstacle_detect_ros',
                plugin='DepthObstacleDetect',
                name='obstacle_detect_component',
                parameters=[{
                'depth_topic':'/cam1/depth_image',
                'camera_info_topic':'/cam1/camera_info',
                'detect_topic':'/cam1/detect',
                'cam_id':'cam1',
                'verbose': True,
                'width_regions':12,
                'height_regions':9,
                'obstacle_range_limit':1.0,
                'obstacle_state_topic':'/cam1/obstacle_state'
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    ))
    return ld