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

namespace smsgs = sensor_msgs::msg;

/**
 * @brief Forward declare our Pimpl object
 *
 */
class Pimpl;

/**
 * @brief Object that connects to the OakD stereo camera and publishes the images and IMU data in ROS format
 *
 */
class OakdRosPublisher : public OakD {
 public:
  /**
   * @brief Construct a new Oakd Ros Publisher object
   *
   * @param node shared_ptr to the ROS node. Must have parameters "fps" and "rectify"
   * @param calibration The calibration data for the camera
   */
  OakdRosPublisher(std::shared_ptr<rclcpp::Node> node, std::optional<StereoCalibration> calibration);

  /**
   * @brief Destroy the Oakd Ros Publisher object
   *
   */
  ~OakdRosPublisher();

  /**
   * @brief Spin, blocks the calling thread and lets the publisher thread run in the background
   *
   */
  void spin();

 private:
  /**
   * @brief Initialize the OakD stereo camera and start the publisher thread
   *
   */
  void Init();

  /**
   * @brief Function which publishes the images in a loop. This will run until the destructor is called
   *
   */
  void publish_stereo_image_thread();

  /**
   * @brief Function which publishes the IMU data in a loop. This will run until the destructor is called
   *
   */
  void publish_imu_thread();

  std::shared_ptr<rclcpp::Node> node_;              //< The rose node
  std::atomic<bool> is_running_ = {false};          //< Whether the publisher thread is running
  std::unique_ptr<StereoCalibration> calibration_;  //< The stereo calibration of the camera
  dai::CalibrationHandler calibration_handler_;     //< The calibration handler for interfacing with the camera hardware
  std::chrono::time_point<std::chrono::steady_clock> start_time_;  //< The time when the publisher thread started

  // Aliases to shorten our declarations
  template <typename T>
  using RpubT = rclcpp::Publisher<T>;

  RpubT<smsgs::Image>::SharedPtr publisher_left_image_;           //< The ROS publisher for the left image
  RpubT<smsgs::CameraInfo>::SharedPtr publisher_left_cam_info_;   //< The ROS publisher for the left camera info
  RpubT<smsgs::Image>::SharedPtr publisher_right_image_;          //< The ROS publisher for the right image
  RpubT<smsgs::CameraInfo>::SharedPtr publisher_right_cam_info_;  //< The ROS publisher for the right camera info

  std::thread publish_stereo_image_thread_;  //< The thread object for the stereo image publisher
  std::thread publish_imu_thread_;           //< The thread object for the IMU publisher

  std::unique_ptr<Pimpl> pimpl_;  //< Objects for the dai::RosBridge live here. This uses the pimpl idiom as a
                                  // workaround to a bug in the dai::RosBridge implementation
};
