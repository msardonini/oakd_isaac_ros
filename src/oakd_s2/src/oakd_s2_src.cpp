#include <chrono>
#include <thread>

#include "oakd_s2/oakd.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("stereo_inertial_node");

  node->declare_parameter("fps", 30);
  node->declare_parameter("rectify", true);
  node->declare_parameter("imu_sample_rate", 400);

  std::cout << " rectify " << (node->get_parameter("rectify").as_bool() ? "true" : "false") << std::endl;

  OakD oak(node);
  oak.spin();

  rclcpp::shutdown();
  return 0;
}
