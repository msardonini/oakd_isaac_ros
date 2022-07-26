#include "oakd_s2/oakd_ros_publisher.h"

#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "opencv2/calib3d.hpp"

// #include "oakd_s2/opencv_adapter.h"
#include "cv_bridge/cv_bridge.h"

static std::atomic<bool> lrcheck{true};
static std::atomic<bool> extended{false};
static std::atomic<bool> subpixel{false};
static constexpr auto timeout_period = std::chrono::milliseconds(1000);
static constexpr auto stereo_sync_time_threshold_us =
    25;  //< If left/right images have a timestamp difference of more than this, we will
         // assume that the images are not synchronized.

class Pimpl {
 public:
  std::unique_ptr<dai::rosBridge::ImuConverter> imu_converter;
  std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>> imu_publisher;

  std::unique_ptr<dai::rosBridge::ImageConverter> left_converter;
  std::unique_ptr<dai::rosBridge::ImageConverter> right_converter;
  std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>> left_publisher;
  std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>> right_publisher;
};

OakdRosPublisher::OakdRosPublisher(std::shared_ptr<rclcpp::Node> node, std::optional<StereoCalibration> calibration)
    : OakD(node->get_parameter("fps").as_int(), node->get_parameter("rectify").as_bool()), node_(node) {
  Init();

  if (calibration) {
    calibration_ = std::make_unique<StereoCalibration>(*calibration);
  }
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
  publisher_left_image_ = node_->create_publisher<sensor_msgs::msg::Image>("/stereo_camera/left/image", 1);
  publisher_left_cam_info_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo_camera/left/camera_info", 1);
  publisher_right_image_ = node_->create_publisher<sensor_msgs::msg::Image>("/stereo_camera/right/image", 1);
  publisher_right_cam_info_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>("/stereo_camera/right/camera_info", 1);

  start_time_ = std::chrono::steady_clock::now();

  is_running_ = true;
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

auto get_camera_calibration(dai::CalibrationHandler &calibration_handler, dai::CameraBoardSocket socket,
                            cv::Size imsize) {
  auto cam0_intrinsics_dai =
      calibration_handler.getCameraIntrinsics(socket, std::make_tuple(imsize.width, imsize.height));
  std::array<double, 9> cam0_intrinsics_ros;

  std::size_t ind = 0;
  std::for_each(cam0_intrinsics_dai.begin(), cam0_intrinsics_dai.end(), [&](auto val) {
    std::for_each(val.begin(), val.end(), [&](auto val) {
      if (ind >= 9) {
        throw std::runtime_error("Invalid camera intrinsics");
      }
      cam0_intrinsics_ros[ind++] = val;
    });
  });

  auto cam0_distortion_dai = calibration_handler.getDistortionCoefficients(socket);

  std::vector<double> cam0_distortion_ros;
  std::for_each(cam0_distortion_dai.begin(), cam0_distortion_dai.end(),
                [&](auto val) { cam0_distortion_ros.push_back(val); });

  return std::make_pair(cam0_intrinsics_ros, cam0_distortion_ros);
}

auto calculate_projection_matrices(std::array<double, 9> K0, std::vector<double> D0, std::array<double, 9> K1,
                                   std::vector<double> D1, cv::Size imsize, std::array<double, 9> R,
                                   std::array<double, 3> T) {
  cv::Mat_<double> K0_cv = cv::Mat_<double>(3, 3, K0.data());
  cv::Mat_<double> D0_cv = cv::Mat_<double>(D0.size(), 1, D0.data());
  cv::Mat_<double> K1_cv = cv::Mat_<double>(3, 3, K1.data());
  cv::Mat_<double> D1_cv = cv::Mat_<double>(D1.size(), 1, D1.data());
  cv::Mat_<double> R_cv = cv::Mat_<double>(3, 3, R.data());
  cv::Mat_<double> T_cv = cv::Mat_<double>(3, 1, T.data());

  cv::Mat_<double> R0_cv, R1_cv;
  cv::Mat_<double> P0_cv, P1_cv;
  cv::Mat_<double> Q_cv;

  cv::stereoRectify(K0_cv, D0_cv, K1_cv, D1_cv, imsize, R_cv, T_cv, R0_cv, R1_cv, P0_cv, P1_cv, Q_cv);

  // Convert the Projection matrices back to arrays
  auto conv_func = [](auto &dst, cv::Mat_<double> &src) {
    std::memcpy(dst.data(), src.data, src.elemSize() * src.total());
  };
  std::array<double, 12> P0, P1;
  conv_func(P0, P0_cv);
  conv_func(P1, P1_cv);

  return std::make_pair(P0, P1);
}

void OakdRosPublisher::publish_stereo_image_thread() {
  // auto calibration_handler = device_->readCalibration();

  // pimpl_->left_converter =
  // std::make_unique<dai::rosBridge::ImageConverter>("odom", true);
  // pimpl_->right_converter =
  // std::make_unique<dai::rosBridge::ImageConverter>("odom", true);

  // int mono_width = 1280;
  // int mono_height = 720;
  // auto left_camera_info =
  // pimpl_->left_converter->calibrationToCameraInfo(calibration_handler,
  // dai::CameraBoardSocket::LEFT, mono_width, mono_height); auto
  // right_camera_info =
  // pimpl_->right_converter->calibrationToCameraInfo(calibration_handler,
  // dai::CameraBoardSocket::RIGHT, mono_width, mono_height);

  // const std::string left_pub_name("/stereo_camera/left/image");
  // const std::string right_pub_name("/stereo_camera/right/image");
  // pimpl_->left_publisher =
  // std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
  // dai::ImgFrame>>(
  //     queue_left_rect_,
  //     node_,
  //     left_pub_name,
  //     std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
  //     pimpl_->left_converter.get(), std::placeholders::_1,
  //     std::placeholders::_2), 3, left_camera_info, "stereo_camera/left");

  // pimpl_->right_publisher =
  // std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image,
  // dai::ImgFrame>>(
  //     queue_right_rect_,
  //     node_,
  //     right_pub_name,
  //     std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
  //     pimpl_->right_converter.get(), std::placeholders::_1,
  //     std::placeholders::_2), 3, right_camera_info, "stereo_camera/right");
  // pimpl_->left_publisher->addPublisherCallback();
  // pimpl_->right_publisher->addPublisherCallback();

  // UNCOMMENT BELOW TO PUBLISH STEREO IMAGES

  std::array<double, 9> K0, K1;
  std::vector<double> D0, D1;
  std::array<double, 12> P0, P1;
  std::array<double, 9> R;
  std::array<double, 3> T;
  if (calibration_) {
    auto convert_to_array = [](auto &input, auto &output) {
      for (auto i = 0; i < input.rows; i++) {
        for (auto j = 0; j < input.cols; j++) {
          output[i * input.cols + j] = input(i, j);
        }
      }
    };
    auto convert_to_array_vec = [](auto &input, auto &output) {
      for (auto i = 0; i < input.channels; i++) {
        output[i] = input(i);
      }
    };

    convert_to_array(calibration_->K_cam0, K0);
    convert_to_array(calibration_->K_cam1, K1);
    D0 = calibration_->D_cam0;
    D1 = calibration_->D_cam1;
    convert_to_array(calibration_->R_cam0_cam1, R);
    convert_to_array_vec(calibration_->T_cam0_cam1, T);

    std::tie(P0, P1) = calculate_projection_matrices(K0, D0, K1, D1, cv::Size(1280, 720), R, T);

    std::cout << "K0: " << std::endl;
    std::for_each(K0.begin(), K0.end(), [](auto &x) { std::cout << x << " "; });
    std::cout << std::endl;

    std::cout << "K0 cal " << calibration_->K_cam0 << std::endl;

  } else {
    auto calibration_handler = device_->readCalibration();

    std::tie(K0, D0) = get_camera_calibration(calibration_handler, dai::CameraBoardSocket::LEFT, cv::Size(1280, 720));
    std::tie(K1, D1) = get_camera_calibration(calibration_handler, dai::CameraBoardSocket::RIGHT, cv::Size(1280, 720));

    // debug - print the k matrices
    std::cout << "K0: " << std::endl;
    std::for_each(K0.begin(), K0.end(), [&](auto val) { std::cout << val << " "; });
    std::cout << std::endl;
    std::cout << "K1: " << std::endl;
    std::for_each(K1.begin(), K1.end(), [&](auto val) { std::cout << val << " "; });
    std::cout << std::endl;

    auto camera_extrinsic_dai =
        calibration_handler.getCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RIGHT);

    for (std::size_t i = 0; i < 3; i++) {
      for (std::size_t j = 0; j < 3; j++) {
        R[i * 3 + j] = camera_extrinsic_dai[i][j];
      }
      T[i] = camera_extrinsic_dai[i][3] / 100.0;  // DAI uses cm, ROS uses m
    }

    // debug - print the extrinsic matrices
    std::cout << "R: " << std::endl;
    std::for_each(R.begin(), R.end(), [&](auto val) { std::cout << val << " "; });
    std::cout << std::endl;
    std::cout << "T: " << std::endl;
    std::for_each(T.begin(), T.end(), [&](auto val) { std::cout << val << " "; });
    std::cout << std::endl;

    std::tie(P0, P1) = calculate_projection_matrices(K0, D0, K1, D1, cv::Size(1280, 720), R, T);
  }

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

    sensor_msgs::msg::CameraInfo left_info;
    left_info.header = header;
    left_info.k = K0;  // TODO figure out why it works better when they are flipped
    left_info.d = D0;
    left_info.p = P0;

    left_info.distortion_model = "rational_polynomial";
    left_info.height = stereo_pair.first->getHeight();
    left_info.width = stereo_pair.first->getWidth();

    sensor_msgs::msg::CameraInfo right_info;
    right_info.header = header;
    right_info.k = K1;
    right_info.d = D1;
    right_info.p = P1;
    right_info.distortion_model = "rational_polynomial";
    right_info.height = stereo_pair.second->getHeight();
    right_info.width = stereo_pair.second->getWidth();

    publisher_left_image_->publish(*left_bridge.toImageMsg());
    publisher_left_cam_info_->publish(left_info);
    publisher_right_image_->publish(*right_bridge.toImageMsg());
    publisher_right_cam_info_->publish(right_info);
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
      queue_imu_, node_, std::string("/visual_slam/imu"),
      std::bind(&dai::rosBridge::ImuConverter::toRosMsg, pimpl_->imu_converter.get(), std::placeholders::_1,
                std::placeholders::_2),
      30, "", "imu");
  pimpl_->imu_publisher->addPublisherCallback();

  // while (is_running_.load()) {

  //   bool has_timed_out;
  //   auto imu_data = GetImuData(timeout_period, has_timed_out);

  //   if (has_timed_out) {
  //     std::cerr << "timed out getting imu data" << std::endl;

  //     // Sleep so we don't spam the system with polls
  //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //     continue;
  //   }

  //   for (auto &packet : imu_data) {
  //     auto time_since_start_ns =
  //     std::chrono::duration_cast<std::chrono::nanoseconds>(packet.gyroscope.timestamp.get()
  //     - start_time_).count(); std_msgs::msg::Header header;
  //     header.stamp.nanosec = time_since_start_ns % 1000000000;
  //     header.stamp.sec = time_since_start_ns / 1000000000;
  //     header.frame_id = "imu";

  //     sensor_msgs::msg::Imu imu_msg;
  //     imu_msg.header = header;
  //     imu_msg.angular_velocity.x = packet.gyroscope.x;
  //     imu_msg.angular_velocity.y = packet.gyroscope.y;
  //     imu_msg.angular_velocity.z = packet.gyroscope.z;
  //     imu_msg.linear_acceleration.x = packet.acceleroMeter.x;
  //     imu_msg.linear_acceleration.y = packet.acceleroMeter.y;
  //     imu_msg.linear_acceleration.z = packet.acceleroMeter.z;
  //     publisher_imu_->publish(imu_msg);
  //   }
  // }
}
