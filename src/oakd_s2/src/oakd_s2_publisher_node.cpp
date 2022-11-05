#include <chrono>
#include <thread>

#include "oakd_s2/oakd_ros_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "yaml-cpp/yaml.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("stereo_inertial_node");

  node->declare_parameter("fps", 30);
  node->declare_parameter("rectify", false);
  node->declare_parameter("imu_sample_rate", 200);

  spdlog::info("rectify: {}", (node->get_parameter("rectify").as_bool() ? "true" : "false"));

  auto stereo_params = YAML::LoadFile("/workspaces/isaac_ros-dev/src/oakd_s2/conf/oakd_s2_plumb_bob.yaml");

  OakdRosPublisher oak(node, stereo_params["stereo_calibration"]);
  oak.spin();

  rclcpp::shutdown();
  return 0;
}
