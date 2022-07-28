#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <queue>
#include <thread>

#include "depthai/depthai.hpp"
#include "flyStereo/sensor_io/oakd.h"
#include "flyStereo/stereo_calibration.h"
#include "opencv2/core/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

class Pimpl;

class OakdRosPublisher : public OakD {
 public:
  OakdRosPublisher(std::shared_ptr<rclcpp::Node> node, std::optional<StereoCalibration> calibration);

  ~OakdRosPublisher();

  void spin();

 private:
  void Init();

  void publish_stereo_image_thread();

  void publish_imu_thread();

  std::shared_ptr<rclcpp::Node> node_;

  std::atomic<bool> is_running_ = {false};

  std::unique_ptr<StereoCalibration> calibration_;
  dai::CalibrationHandler calibration_handler_;

  std::chrono::time_point<std::chrono::steady_clock> start_time_;

  // ROS publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_left_cam_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_right_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_right_cam_info_;
  // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;

  std::unique_ptr<Pimpl> pimpl_;

  std::thread publish_stereo_image_thread_;
  std::thread publish_imu_thread_;
};
