#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <queue>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "depthai/depthai.hpp"
#include "opencv2/core/core.hpp"

class Pimpl;

class OakD {
public:
  OakD();

  ~OakD();

  int GetSynchronizedData(cv::Mat_<uint8_t> &left_image,
                          cv::Mat_<uint8_t> &right_image,
                          std::vector<dai::IMUPacket> &imu_data_a,
                          uint64_t &current_frame_time);

  void spin();

private:
  void Init();

  void publish_stereo_image_thread();

  void publish_imu_thread();

  std::vector<dai::IMUPacket> GetImuData(std::chrono::milliseconds timeout,
                                         bool &has_timed_out);

  std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>>
  GetStereoImagePair(std::chrono::milliseconds timeout, bool &has_timed_out);

  std::shared_ptr<rclcpp::Node> node_;

  std::atomic<bool> is_running_ = {false};

  dai::Pipeline pipeline_;
  std::unique_ptr<dai::Device> device_;

  std::shared_ptr<dai::node::StereoDepth> stereo_;
  std::shared_ptr<dai::DataOutputQueue> queue_left_;
  std::shared_ptr<dai::DataOutputQueue> queue_left_rect_;
  std::shared_ptr<dai::DataOutputQueue> queue_right_;
  std::shared_ptr<dai::DataOutputQueue> queue_right_rect_;
  std::shared_ptr<dai::node::XLinkOut> xout_left_;
  std::shared_ptr<dai::node::XLinkOut> xout_left_rect_;
  std::shared_ptr<dai::node::XLinkOut> xout_right_;
  std::shared_ptr<dai::node::XLinkOut> xout_right_rect_;
  std::shared_ptr<dai::node::MonoCamera> mono_left_;
  std::shared_ptr<dai::node::MonoCamera> mono_right_;
  std::shared_ptr<dai::node::IMU> imu_;
  std::shared_ptr<dai::node::XLinkOut> xout_imu_;
  std::shared_ptr<dai::DataOutputQueue> queue_imu_;
  std::queue<dai::IMUPacket> local_queue_imu_;

  std::chrono::time_point<std::chrono::steady_clock> start_time_;

  // ROS publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_left_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      publisher_left_cam_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_right_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      publisher_right_cam_info_;
  // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;

  std::unique_ptr<Pimpl> pimpl_;

  std::thread publish_stereo_image_thread_;
  std::thread publish_imu_thread_;
};
