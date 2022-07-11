from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    static_transsform_node = Node(package = "tf2_ros",
                                    executable = "static_transform_publisher",
                                    arguments = "--x 0 --y 0 --z 0 --roll 3.14 --pitch 1.57 --yaw 0 --frame-id base_link --child-frame-id imu".split(' '))

    camera_src_node = Node(package = "oakd_s2",
                       executable = "oakd_s2_src")

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[('/stereo_camera/left/camera_info', '/camera_info_left'),
                    ('/stereo_camera/right/camera_info', '/camera_info_right')],
        parameters=[{
                    'use_sim_time': False,
                    'denoise_input_images': True,
                    'rectified_images': True,
                    'enable_slam_visualization': True,
                    'enable_observations_view': True,
                    'enable_landmarks_view': True,
                    'enable_debug_mode': False,
                    'publish_tf': True,
                    'enable_imu': True,
                    'debug_dump_path': '/tmp/elbrus',
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'input_imu_frame': 'imu',
                    }]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    return LaunchDescription([static_transsform_node, camera_src_node, visual_slam_launch_container])