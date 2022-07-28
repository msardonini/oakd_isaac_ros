from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# [ -0.7071068, 0, 0.7071068, 0 ]


def generate_launch_description():
    static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=
        "--x 0.08038944 --y -0.00407412 --z -0.02168634 --qx -0.7071068 --qy 0.0 --qz 0.7071068 --qw -0.0 --frame-id base_link --child-frame-id imu"
        .split(' '))

    camera_src_node = Node(package="oakd_s2",
                           executable="oakd_s2_src",
                           parameters=[{
                               'rectify': False
                           }])

    visual_slam_node = ComposableNode(
        # visual_slam_node = Node(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        # remappings=[('/stereo_camera/left/camera_info', '/left/camera_info'),
        #             ('/stereo_camera/right/camera_info', '/right/camera_info')],
        parameters=[{
            'use_sim_time': False,
            'denoise_input_images': True,
            'rectified_images': False,
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
            'gravitational_force': [9.8, 0.0, 0.0]
        }])

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen')

    return LaunchDescription(
        [static_transform_node, camera_src_node, visual_slam_launch_container])
