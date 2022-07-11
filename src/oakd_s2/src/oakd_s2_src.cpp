#include <chrono>
#include <thread>

#include "oakd_s2/oakd.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OakD>());

  // OakD oak;
  // while (rclcpp::ok()) {
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  // }

  rclcpp::shutdown();
  return 0;
}
