#include "oakd_s2/oakd.h"


// #include "oakd_s2/opencv_adapter.h"
#include "cv_bridge/cv_bridge.h"

static std::atomic<bool> lrcheck{true};
static std::atomic<bool> extended{false};
static std::atomic<bool> subpixel{false};
static constexpr auto timeout_period = std::chrono::milliseconds(1000);


OakD::OakD() : Node("oakd") {
  Init();
}

OakD::~OakD() {
  is_running_ = false;

  if (publish_imu_thread_.joinable()) {
    publish_imu_thread_.join();
  }

  if (publish_stereo_image_thread_.joinable()) {
    publish_stereo_image_thread_.join();
  }
}

void OakD::Init() {
  // Initialize ROS publishers
  publisher_left_image_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo_camera/left/image", 1);
  publisher_left_cam_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info_left", 1);
  publisher_right_image_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo_camera/right/image", 1);
  publisher_right_cam_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info_right", 1);
  publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/visual_slam/imu", 1);

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
  imu_->enableIMUSensor(dai::IMUSensor::LINEAR_ACCELERATION, 200);

  // it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline
  // with a lot of input/output connections above this threshold packets will be sent in batch of X, if the host is not
  // blocked and USB bandwidth is available
  imu_->setBatchReportThreshold(1);
  // maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
  // if lower or equal to batchReportThreshold then the sending is always blocking on device
  // useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple
  // nodes
  imu_->setMaxBatchReports(40);

  // Link plugins IMU -> XLINK
  imu_->out.link(xout_imu_->input);

  // Properties
  mono_left_->setBoardSocket(dai::CameraBoardSocket::LEFT);
  mono_left_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
  mono_right_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  mono_right_->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);


  // StereoDepth
  stereo_->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
  stereo_->setRectifyEdgeFillColor(0);  // black, to better see the cutout
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

  // Output queues will be used to get the grayscale frames from the outputs defined above
  queue_left_ = device_->getOutputQueue("left", 1, false);
  queue_right_ = device_->getOutputQueue("right", 1, false);
  queue_imu_ = device_->getOutputQueue("imu", 1, true);
  queue_left_rect_ = device_->getOutputQueue("rectified_left", 1, false);
  queue_right_rect_ = device_->getOutputQueue("rectified_right", 1, false);

  start_time_ = std::chrono::steady_clock::now();

  is_running_ = true;
  publish_stereo_image_thread_ = std::thread(&OakD::publish_stereo_image_thread, this);
  publish_imu_thread_ = std::thread(&OakD::publish_imu_thread, this);
}

std::vector<dai::IMUPacket> OakD::GetImuData(std::chrono::milliseconds timeout, bool &has_timed_out) {
  auto imu_data = queue_imu_->get<dai::IMUData>(timeout, has_timed_out);

  if (has_timed_out) {
    has_timed_out = true;
    return std::vector<dai::IMUPacket>();
  } else {
    return std::move(imu_data->packets);
  }
}

std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>> OakD::GetStereoImagePair(std::chrono::milliseconds timeout, bool &has_timed_out) {
  bool has_timed_out_check = false;
  auto in_left = queue_left_rect_->get<dai::ImgFrame>(timeout, has_timed_out_check);
  has_timed_out |= has_timed_out_check;
  auto in_right = queue_right_rect_->get<dai::ImgFrame>(timeout, has_timed_out_check);
  has_timed_out |= has_timed_out_check;

  if (has_timed_out) {
    has_timed_out = true;
    return {};
  } else {
    return std::make_pair(in_left, in_right);
  }
}


int OakD::GetSynchronizedData(cv::Mat_<uint8_t> &left_image, cv::Mat_<uint8_t> &right_image,
                                      std::vector<dai::IMUPacket> &imu_data_a, uint64_t &current_frame_time) {
  bool has_timed_out = false;
  bool has_timed_out_check = false;

  auto stereo_pair = this->GetStereoImagePair(timeout_period, has_timed_out_check);
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

  current_frame_time = std::chrono::duration_cast<std::chrono::microseconds>(left_cam_time - start_time_).count();

  // auto handler = dai::CalibrationHandler();
  // auto left_d = handler.getDistortionCoefficients(dai::CameraBoardSocket::LEFT);
  // auto right = handler.getDistortionCoefficients(dai::CameraBoardSocket::RIGHT);

  // std::for_each(left_d.begin(), left_d.end(), [](auto val) {std::cout << ", " << val;});
  // std::cout << std::endl;
  // std::for_each(right.begin(), right.end(), [](auto val) {std::cout << ", " << val;});

  return 0;
}

void OakD::publish_stereo_image_thread() {
  while (is_running_.load()) {
    bool has_timed_out = false;
    auto stereo_pair = GetStereoImagePair(timeout_period, has_timed_out);

    if (has_timed_out) {
      std::cerr << " Error timeout getting stereo images!" << std::endl;
      continue;
    }

    auto time_since_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stereo_pair.first->getTimestamp() - start_time_).count();
    std_msgs::msg::Header header;
    header.stamp.nanosec = time_since_start_ns % 1000000000;
    header.stamp.sec = time_since_start_ns / 1000000000;
    header.frame_id = "odom";


    cv_bridge::CvImage left_bridge(header, "mono8", stereo_pair.first->getCvFrame());
    cv_bridge::CvImage right_bridge(header, "mono8", stereo_pair.second->getCvFrame());


    // ---------- TODO REMOVE ----------
    sensor_msgs::msg::CameraInfo left_info;
    left_info.header = header;
    left_info.k = std::array<double, 9>{796.38562012,   0.        , 661.84564209,
         0.        , 796.38562012, 412.57632446,
         0.        ,   0.        ,   1.        };
    left_info.d = std::vector<double>{-9.37869453e+00,  9.77382812e+01, -1.61361357e-04,  9.01929045e-04,
       -5.47321243e+01, -9.42689991e+00,  9.71486511e+01, -5.31322670e+01};
    left_info.p = std::array<double, 12>{

    797.6177063 ,   0.        , 668.36348724,   0.        ,
         0.        , 797.6177063 , 416.41225433,   0.        ,
         0.        ,   0.        ,   1.        ,   0.

     };

    left_info.distortion_model = "pinhole";
    left_info.height = stereo_pair.first->getHeight();
    left_info.width = stereo_pair.first->getWidth();

    sensor_msgs::msg::CameraInfo right_info;
    right_info.header = header;
    right_info.k = std::array<double, 9>{798.84979248,   0.        , 643.73754883, 0.        , 798.84979248, 415.42175293, 0.        ,   0.        ,   1.        };
    right_info.d = std::vector<double>{-9.53572750e+00,  1.01452194e+02, -5.90408163e-04,  1.25376263e-03,
       -5.61496582e+01, -9.58054447e+00,  1.00816727e+02, -5.43621254e+01};
    right_info.p = std::array<double, 12>{
        797.6177063 ,   0.        , 668.36348724, -59.70244762,
         0.        , 797.6177063 , 416.41225433,   0.        ,
         0.        ,   0.        ,   1.        ,   0.
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
  while (is_running_.load()) {

    bool has_timed_out;
    auto imu_data = GetImuData(timeout_period, has_timed_out);

    if (has_timed_out) {
      std::cerr << "timed out getting imu data" << std::endl;

      // Sleep so we don't spam the system with polls
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    for (auto &packet : imu_data) {
      auto time_since_start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(packet.gyroscope.timestamp.get() - start_time_).count();
      std_msgs::msg::Header header;
      header.stamp.nanosec = time_since_start_ns % 1000000000;
      header.stamp.sec = time_since_start_ns / 1000000000;
      header.frame_id = "imu";

      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header = header;
      imu_msg.angular_velocity.x = packet.gyroscope.x;
      imu_msg.angular_velocity.y = packet.gyroscope.y;
      imu_msg.angular_velocity.z = packet.gyroscope.z;
      imu_msg.linear_acceleration.x = packet.acceleroMeter.x;
      imu_msg.linear_acceleration.y = packet.acceleroMeter.y;
      imu_msg.linear_acceleration.z = packet.acceleroMeter.z;
      publisher_imu_->publish(imu_msg);
    }
  }

}