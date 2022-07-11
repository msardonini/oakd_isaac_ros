from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    static_transsform_node = Node(package = "tf2_ros",
                                    executable = "static_transform_publisher",
                                    arguments = "--x 0 --y 0 --z 0 --roll 3.14 --pitch -1.57 --yaw 0 --frame-id base_link --child-frame-id imu".split(' '))

    camera_src_node = Node(package = "oakd_s2",
                       executable = "oakd_s2_src")

    return LaunchDescription([static_transsform_node, camera_src_node])
