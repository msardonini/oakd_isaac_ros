#include "oakd_s2/oakd_ros_publisher.h"

#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "flyStereo/stereo_calibration_dai_convert.h"
#include "opencv2/calib3d.hpp"
#include "stereo_calibration_ros_convert.hpp"

// #include "oakd_s2/opencv_adapter.h"
#include "cv_bridge/cv_bridge.h"

static constexpr auto timeout_period = std::chrono::milliseconds(1000);
static const std::string imu_topic_name = "/visual_slam/imu";
static const std::string cam0_topic_name = "/stereo_camera/left/image";
static const std::string cam0_info_topic_name = "/stereo_camera/left/camera_info";
static const std::string cam1_topic_name = "/stereo_camera/right/image";
static const std::string cam1_info_topic_name = "/stereo_camera/right/camera_info";

/**
 * @brief Pimpl object to hold the dai::rosBridge objects. This is a pimpl because several functions in dai::rosBridge's
 * header file are not inlined and therefore cannot be compiled in multiple translation units.
 */
class Pimpl {
 public:
  std::unique_ptr<dai::rosBridge::ImuConverter> imu_converter;
  std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>> imu_publisher;
};

OakdRosPublisher::OakdRosPublisher(std::shared_ptr<rclcpp::Node> node, std::optional<StereoCalibration> calibration)
    : OakD(node->get_parameter("fps").as_int(), node->get_parameter("rectify").as_bool()), node_(node) {
  if (calibration) {
    calibration_ = std::make_unique<StereoCalibration>(*calibration);

    calibration_handler_ = stereo_calibration_convert<dai::CalibrationHandler>(*calibration_);
    pipeline_.setCalibrationData(calibration_handler_);
  }

  Init();
}

OakdRosPublisher::~OakdRosPublisher() {
  is_running_ = false;

  if (publish_imu_thread_.joinable()) {
    publish_imu_thread_.join();
  }

  if (publish_stereo_image_thread_.joinable()) {
    publish_stereo_image_thread_.join();
  }
}

void OakdRosPublisher::spin() { rclcpp::spin(node_); }

void OakdRosPublisher::Init() {
  // Initialize ROS publishers
  publisher_left_image_ = node_->create_publisher<sensor_msgs::msg::Image>(cam0_topic_name, 1);
  publisher_left_cam_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(cam0_info_topic_name, 1);
  publisher_right_image_ = node_->create_publisher<sensor_msgs::msg::Image>(cam1_topic_name, 1);
  publisher_right_cam_info_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(cam1_info_topic_name, 1);

  is_running_ = true;
  start_time_ = std::chrono::steady_clock::now();
  publish_stereo_image_thread_ = std::thread(&OakdRosPublisher::publish_stereo_image_thread, this);
  publish_imu_thread_ = std::thread(&OakdRosPublisher::publish_imu_thread, this);
}

static auto get_ts_now(
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> ts_data) {
  auto rclNow = rclcpp::Clock().now();
  auto steadyTime = std::chrono::steady_clock::now();
  auto diffTime = steadyTime - ts_data;
  return rclNow - diffTime;
}

void OakdRosPublisher::publish_stereo_image_thread() {
  while (is_running_.load()) {
    bool has_timed_out = false;
    auto stereo_pair = GetStereoImagePair(timeout_period, has_timed_out);

    if (has_timed_out) {
      std::cerr << " Error timeout getting stereo images!" << std::endl;
      continue;
    }

    std_msgs::msg::Header header;
    header.stamp = get_ts_now(stereo_pair.first->getTimestamp());
    header.frame_id = "odom";

    cv_bridge::CvImage left_bridge(header, "mono8", stereo_pair.first->getCvFrame());
    cv_bridge::CvImage right_bridge(header, "mono8", stereo_pair.second->getCvFrame());

    auto stereo_cam_info = stereo_calibration_convert<std::array<sensor_msgs::msg::CameraInfo, 2>>(*calibration_);

    stereo_cam_info[0].header = header;
    stereo_cam_info[1].header = header;

    publisher_left_image_->publish(*left_bridge.toImageMsg());
    publisher_left_cam_info_->publish(stereo_cam_info[0]);
    publisher_right_image_->publish(*right_bridge.toImageMsg());
    publisher_right_cam_info_->publish(stereo_cam_info[1]);
  }
}

void OakdRosPublisher::publish_imu_thread() {
  double linear_accel_cov = 0.0;
  double angular_vel_cov = 0.02;
  dai::ros::ImuSyncMethod imu_mode = static_cast<dai::ros::ImuSyncMethod>(1);  // TODO read into what this means

  pimpl_ = std::make_unique<Pimpl>();

  pimpl_->imu_converter =
      std::make_unique<dai::rosBridge::ImuConverter>("oak_imu", imu_mode, linear_accel_cov, angular_vel_cov);

  pimpl_->imu_publisher = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>>(
      queue_imu_, node_, std::string(imu_topic_name),
      std::bind(&dai::rosBridge::ImuConverter::toRosMsg, pimpl_->imu_converter.get(), std::placeholders::_1,
                std::placeholders::_2),
      30, "", "imu");
  pimpl_->imu_publisher->addPublisherCallback();
}
