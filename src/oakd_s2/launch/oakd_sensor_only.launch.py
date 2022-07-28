from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    static_transsform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=
        "--x 0.08038944 --y -0.00407412 --z -0.02168634 --qx 0.7108743 --qy 0.7033043 --qz 0.0027012 --qw -0.0036683 --frame-id base_link --child-frame-id imu"
        .split(' '))

    camera_src_node = Node(package="oakd_s2",
                           executable="oakd_s2_src",
                           parameters=[{
                               'rectify': False
                           }])

    return LaunchDescription([static_transsform_node, camera_src_node])
