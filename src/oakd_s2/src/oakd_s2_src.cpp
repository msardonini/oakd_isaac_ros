#include <chrono>
#include <thread>

#include "oakd_s2/oakd.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  OakD oak;
  oak.spin();

  rclcpp::shutdown();
  return 0;
}
