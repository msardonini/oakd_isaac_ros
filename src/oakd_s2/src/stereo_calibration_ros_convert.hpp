#pragma once

#include "flyStereo/stereo_calibration.h"
#include "sensor_msgs/msg/camera_info.hpp"

/**
 * @brief Template specialization for the StereoCalibrationConvert class, enabling the conversion of StereoCalibration
 * to 2x sensor_msgs::msg::CameraInfo messages. One for the left camera and one for the right camera.
 *
 */
template <>
struct StereoCalibrationConvert<std::array<sensor_msgs::msg::CameraInfo, 2>> {
 public:
  /**
   * @brief Convert a StereoCalibration object to a std::array<sensor_msgs::msg::CameraInfo, 2>. The output array is
   * filled with the left, then right camera info.
   *
   * @param calibration The StereoCalibration object to convert.
   * @return std::array<sensor_msgs::msg::CameraInfo, 2>
   */
  static std::array<sensor_msgs::msg::CameraInfo, 2> convert(const StereoCalibration &calibration) {
    sensor_msgs::msg::CameraInfo left_cam_info;
    left_cam_info.k = convert_to_array(calibration.K_cam0);
    left_cam_info.d = calibration.D_cam0;
    left_cam_info.r = convert_to_array(calibration.R0);
    left_cam_info.p = convert_to_array(calibration.P0);
    left_cam_info.width = calibration.image_size.width;
    left_cam_info.height = calibration.image_size.height;
    left_cam_info.roi = rect_to_roi(calibration.valid_roi_left);
    left_cam_info.distortion_model = "rational_polynomial";

    sensor_msgs::msg::CameraInfo right_cam_info;
    right_cam_info.k = convert_to_array(calibration.K_cam1);
    right_cam_info.d = calibration.D_cam1;
    right_cam_info.r = convert_to_array(calibration.R0);
    right_cam_info.p = convert_to_array(calibration.P1);
    right_cam_info.width = calibration.image_size.width;
    right_cam_info.height = calibration.image_size.height;
    right_cam_info.roi = rect_to_roi(calibration.valid_roi_right);
    right_cam_info.distortion_model = "rational_polynomial";

    return {left_cam_info, right_cam_info};
  }

 private:
  /**
   * @brief Helper function to convert a cv::Matx-like array to an std::array<double, N>
   *
   * @tparam InputT The cv::Matx-like array type
   * @tparam ArrSize The size of the output array
   * @param input The cv::Matx array
   * @return std::array<double, ArrSize>
   */
  template <typename InputT, std::size_t ArrSize = InputT::rows *InputT::cols>
  static std::array<double, ArrSize> convert_to_array(const InputT &input) {
    std::array<double, ArrSize> output;
    for (auto i = 0; i < input.rows; i++) {
      for (auto j = 0; j < input.cols; j++) {
        output[i * input.cols + j] = input(i, j);
      }
    }
    return output;
  }

  /**
   * @brief Helper function to convert a cv::Rect to a sensor_msgs::msg::RegionOfInterest
   *
   * @param rect The cv::Rect to convert
   * @return sensor_msgs::msg::RegionOfInterest
   */
  static sensor_msgs::msg::RegionOfInterest rect_to_roi(const cv::Rect &rect) {
    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset = rect.x;
    roi.y_offset = rect.y;
    roi.width = rect.width;
    roi.height = rect.height;
    return roi;
  }
};
