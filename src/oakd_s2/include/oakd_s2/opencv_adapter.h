#pragma once

#include <utility>

#include "opencv2/core/core.hpp"
#include "sensor_msgs/msg/image.h"

template <>
struct rclcpp::TypeAdapter<std::pair<cv::Mat_<uint8_t>, uint64_t>,
                           sensor_msgs__msg__Image> {
  using is_specialized = std::true_type;
  using custom_type = std::pair<cv::Mat_<uint8_t>, uint64_t>;
  using ros_message_type = sensor_msgs__msg__Image;

  static void convert_to_ros_message(const custom_type &source,
                                     ros_message_type &destination) {
    destination.header.stamp.sec = source.second / 1E6;
    destination.header.stamp.nanosec = (source.second % 1E6) * 1E3;
    destination.height = source.first.rows;
    destination.width = source.first.cols;
    destination.encoding = sensor_msgs::image_encodings::UC8;
    destination.is_bigendian = false;
    destination.step = source.first.step;
    destination.data = source.first.data;
  }

  static void convert_to_custom(const ros_message_type &source,
                                custom_type &destination) {
    throw std::runtime_error("not implemented");
  }
};
