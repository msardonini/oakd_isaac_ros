#include "oakd_s2/oakd.h"

#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"

// #include "oakd_s2/opencv_adapter.h"
#include "cv_bridge/cv_bridge.h"

static std::atomic<bool> lrcheck{true};
static std::atomic<bool> extended{false};
static std::atomic<bool> subpixel{false};
static constexpr auto timeout_period = std::chrono::milliseconds(1000);

class Pimpl {
public:
  std::unique_ptr<dai::rosBridge::ImuConverter> imu_converter;
  std::unique_ptr<
      dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>>
      imu_publisher;

  std::unique_ptr<dai::rosBridge::ImageConverter> left_converter;
  std::unique_ptr<dai::rosBridge::ImageConverter> right_converter;
  std::unique_ptr<
      dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>
      left_publisher;
  std::unique_ptr<
      dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>
      right_publisher;
};

OakD::OakD() { Init(); }

OakD::~OakD() {
  is_running_ = false;

  if (publish_imu_thread_.joinable()) {
    publish_imu_thread_.join();
  }

  if (publish_stereo_image_thread_.joinable()) {
    publish_stereo_image_thread_.join();
  }
}

void OakD::spin() { rclcpp::spin(node_); }

void OakD::Init() {
  node_ = rclcpp::Node::make_shared("stereo_inertial_node");

  // Initialize ROS publishers
  publisher_left_image_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/stereo_camera/left/image", 1);
  publisher_left_cam_info_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>(
          "/stereo_camera/left/camera_info", 1);
  publisher_right_image_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/stereo_camera/right/image", 1);
  publisher_right_cam_info_ =
      node_->create_publisher<sensor_msgs::msg::CameraInfo>(
          "/stereo_camera/right/camera_info", 1);

  // Define sources and outputs
  mono_left_ = pipeline_.create<dai::node::MonoCamera>();
  mono_right_ = pipeline_.create<dai::node::MonoCamera>();
  xout_left_ = pipeline_.create<dai::node::XLinkOut>();
  xout_right_ = pipeline_.create<dai::node::XLinkOut>();
  stereo_ = pipeline_.create<dai::node::StereoDepth>();
  xout_left_rect_ = pipeline_.create<dai::node::XLinkOut>();
  xout_right_rect_ = pipeline_.create<dai::node::XLinkOut>();

  xout_left_->setStreamName("left");
  xout_right_->setStreamName("right");
  xout_left_rect_->setStreamName("rectified_left");
  xout_right_rect_->setStreamName("rectified_right");

  // Define sources and outputs
  imu_ = pipeline_.create<dai::node::IMU>();
  xout_imu_ = pipeline_.create<dai::node::XLinkOut>();
  xout_imu_->setStreamName("imu");

  // enable ROTATION_VECTOR at 400 hz rate
  // imu_->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 200);
  imu_->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED, 200);
  // imu_->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 200);
  // imu_->enableIMUSensor(dai::IMUSensor::LINEAR_ACCELERATION, 200);
  imu_->enableIMUSensor(dai::IMUSensor::ACCELEROMETER, 200);
  // imu_->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);

  // it's recommended to set both setBatchReportThreshold and setMaxBatchReports
  // to 20 when integrating in a pipeline with a lot of input/output connections
  // above this threshold packets will be sent in batch of X, if the host is not
  // blocked and USB bandwidth is available
  imu_->setBatchReportThreshold(1);
  // maximum number of IMU packets in a batch, if it's reached device will block
  // sending until host can receive it if lower or equal to batchReportThreshold
  // then the sending is always blocking on device useful to reduce device's CPU
  // load  and number of lost packets, if CPU load is high on device side due to
  // multiple nodes
  imu_->setMaxBatchReports(40);

  // Link plugins IMU -> XLINK
  imu_->out.link(xout_imu_->input);

  // Properties
  mono_left_->setBoardSocket(dai::CameraBoardSocket::LEFT);
  mono_left_->setResolution(
      dai::MonoCameraProperties::SensorResolution::THE_720_P);
  mono_right_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  mono_right_->setResolution(
      dai::MonoCameraProperties::SensorResolution::THE_720_P);

  // StereoDepth
  stereo_->setDefaultProfilePreset(
      dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
  stereo_->setRectifyEdgeFillColor(0); // black, to better see the cutout
  // stereo_->setInputResolution(1280, 720);
  stereo_->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
  stereo_->setLeftRightCheck(lrcheck);
  stereo_->setExtendedDisparity(extended);
  stereo_->setSubpixel(subpixel);

  // Linking
  mono_left_->out.link(stereo_->left);
  mono_right_->out.link(stereo_->right);

  stereo_->syncedLeft.link(xout_left_->input);
  stereo_->syncedRight.link(xout_right_->input);

  stereo_->rectifiedLeft.link(xout_left_rect_->input);
  stereo_->rectifiedRight.link(xout_right_rect_->input);

  // Linking
  // mono_left_->out.link(xout_left_->input);
  // mono_right_->out.link(xout_right_->input);

  device_ = std::make_unique<dai::Device>(pipeline_);

  auto calibrationHandler = device_->readCalibration();

  // Output queues will be used to get the grayscale frames from the outputs
  // defined above
  queue_left_ = device_->getOutputQueue("left", 1, false);
  queue_right_ = device_->getOutputQueue("right", 1, false);
  queue_imu_ = device_->getOutputQueue("imu", 30, false);
  queue_left_rect_ = device_->getOutputQueue("rectified_left", 3, false);
  queue_right_rect_ = device_->getOutputQueue("rectified_right", 3, false);

  start_time_ = std::chrono::steady_clock::now();

  is_running_ = true;
  publish_stereo_image_thread_ =
      std::thread(&OakD::publish_stereo_image_thread, this);
  publish_imu_thread_ = std::thread(&OakD::publish_imu_thread, this);
}

std::vector<dai::IMUPacket> OakD::GetImuData(std::chrono::milliseconds timeout,
                                             bool &has_timed_out) {
  auto imu_data = queue_imu_->get<dai::IMUData>(timeout, has_timed_out);

  if (has_timed_out) {
    has_timed_out = true;
    return std::vector<dai::IMUPacket>();
  } else {
    return std::move(imu_data->packets);
  }
}

std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>>
OakD::GetStereoImagePair(std::chrono::milliseconds timeout,
                         bool &has_timed_out) {
  bool has_timed_out_check = false;
  auto in_left =
      queue_left_rect_->get<dai::ImgFrame>(timeout, has_timed_out_check);
  has_timed_out |= has_timed_out_check;
  auto in_right =
      queue_right_rect_->get<dai::ImgFrame>(timeout, has_timed_out_check);
  has_timed_out |= has_timed_out_check;

  if (has_timed_out) {
    has_timed_out = true;
    return {};
  } else {
    return std::make_pair(in_left, in_right);
  }
}

int OakD::GetSynchronizedData(cv::Mat_<uint8_t> &left_image,
                              cv::Mat_<uint8_t> &right_image,
                              std::vector<dai::IMUPacket> &imu_data_a,
                              uint64_t &current_frame_time) {
  bool has_timed_out = false;
  bool has_timed_out_check = false;

  auto stereo_pair =
      this->GetStereoImagePair(timeout_period, has_timed_out_check);
  has_timed_out |= has_timed_out_check;
  auto imu_data = this->GetImuData(timeout_period, has_timed_out_check);
  has_timed_out |= has_timed_out_check;

  if (has_timed_out) {
    std::cerr << " Error poll timed out!" << std::endl;
    return -1;
  }

  auto left_cam_time = stereo_pair.first->getTimestamp();
  left_image = stereo_pair.first->getCvFrame();
  right_image = stereo_pair.second->getCvFrame();

  // Get all the IMU data that is in the time range of the current frame
  std::for_each(imu_data.begin(), imu_data.end(),
                [&](auto &val) { local_queue_imu_.push(std::move(val)); });
  imu_data_a.clear();
  while (!local_queue_imu_.empty()) {
    auto imu_time = local_queue_imu_.front().gyroscope.timestamp.get();
    if (imu_time > left_cam_time) {
      break;
    }

    imu_data_a.push_back(local_queue_imu_.front());
    local_queue_imu_.pop();
  }

  current_frame_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           left_cam_time - start_time_)
                           .count();

  // auto handler = dai::CalibrationHandler();
  // auto left_d =
  // handler.getDistortionCoefficients(dai::CameraBoardSocket::LEFT); auto right
  // = handler.getDistortionCoefficients(dai::CameraBoardSocket::RIGHT);

  // std::for_each(left_d.begin(), left_d.end(), [](auto val) {std::cout << ", "
  // << val;}); std::cout << std::endl; std::for_each(right.begin(),
  // right.end(), [](auto val) {std::cout << ", " << val;});

  return 0;
}

static auto
get_ts_now(std::chrono::time_point<std::chrono::steady_clock,
                                   std::chrono::steady_clock::duration>
               ts_data) {
  auto rclNow = rclcpp::Clock().now();
  auto steadyTime = std::chrono::steady_clock::now();
  auto diffTime = steadyTime - ts_data;
  return rclNow - diffTime;
}

void OakD::publish_stereo_image_thread() {
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

    cv_bridge::CvImage left_bridge(header, "mono8",
                                   stereo_pair.first->getCvFrame());
    cv_bridge::CvImage right_bridge(header, "mono8",
                                    stereo_pair.second->getCvFrame());

    // ---------- TODO REMOVE ----------
    sensor_msgs::msg::CameraInfo left_info;
    left_info.header = header;
    left_info.k = std::array<double, 9>{782.6277760615313,
                                        0.,
                                        634.4643730415885,
                                        0.,
                                        780.2346160675271,
                                        385.614038492839,
                                        0.,
                                        0.,
                                        1.};
    left_info.d =
        std::vector<double>{0.0492716660282836, -0.11871655338048466,
                            0.0035855531787139148, -0.0003228386098753934};
    left_info.p = std::array<double, 12>{

        780.84529267, 0., 622.50637817, 0., 0., 780.84529267,
        395.02053452, 0., 0.,           0., 1., 0.};

    left_info.distortion_model = "pinhole";
    left_info.height = stereo_pair.first->getHeight();
    left_info.width = stereo_pair.first->getWidth();

    sensor_msgs::msg::CameraInfo right_info;
    right_info.header = header;
    right_info.k = std::array<double, 9>{783.9095741544915,
                                         0.,
                                         635.3491224357887,
                                         0.,
                                         781.4559692704684,
                                         385.5555177626138,
                                         0.,
                                         0.,
                                         1.};
    right_info.d =
        std::vector<double>{0.017225154487590526, -0.0632091367047388,
                            0.0018905301214379726, 0.00014073945226710378};
    right_info.p = std::array<double, 12>{
        780.84529267, 0., 622.50637817, -59.20166859, 0., 780.84529267,
        395.02053452, 0., 0.,           0.,           1., 0.,
    };

    right_info.distortion_model = "pinhole";
    right_info.height = stereo_pair.second->getHeight();
    right_info.width = stereo_pair.second->getWidth();
    // ---------- END TODO REMOVE ----------

    publisher_left_image_->publish(*left_bridge.toImageMsg());
    publisher_left_cam_info_->publish(left_info);
    publisher_right_image_->publish(*right_bridge.toImageMsg());
    publisher_right_cam_info_->publish(right_info);
  }
}

void OakD::publish_imu_thread() {
  double linear_accel_cov = 0.0;
  double angular_vel_cov = 0.02;
  dai::ros::ImuSyncMethod imu_mode =
      static_cast<dai::ros::ImuSyncMethod>(1); // TODO read into what this means

  pimpl_ = std::make_unique<Pimpl>();

  pimpl_->imu_converter = std::make_unique<dai::rosBridge::ImuConverter>(
      "oak_imu", imu_mode, linear_accel_cov, angular_vel_cov);

  pimpl_->imu_publisher = std::make_unique<
      dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>>(
      queue_imu_, node_, std::string("/visual_slam/imu"),
      std::bind(&dai::rosBridge::ImuConverter::toRosMsg,
                pimpl_->imu_converter.get(), std::placeholders::_1,
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
