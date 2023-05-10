// Copyright 2023 Forssea Robotics
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#ifndef OCULUS_ROS2__SONAR_VIEWER_HPP_
#define OCULUS_ROS2__SONAR_VIEWER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include <oculus_interfaces/msg/ping.hpp>
#include <oculus_ros2/conversions.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

class SonarViewer {
public:
  explicit SonarViewer(rclcpp::Node* node);
  ~SonarViewer();
  void publishFan(const oculus::PingMessage::ConstPtr& ping, const std::string& frame_id = "", const int& data_depth = 0) const;
  void publishFan(
      const oculus_interfaces::msg::Ping& ros_ping_msg, const std::string& frame_id = "", const int& data_depth = 0) const;
  template <typename DataType>
  void publishFan(const int& width,
      const int& height,
      const int& offset,
      const std::vector<DataType>& ping_data,
      const int& master_mode,
      const double& ping_rage,
      const std_msgs::msg::Header& header) const;
  void streamAndFilter(const oculus::PingMessage::ConstPtr& ping, cv::Mat& data);

protected:
  const double LOW_FREQUENCY_BEARING_APERTURE = 65.;
  const double HIGHT_FREQUENCY_BEARING_APERTURE = 40.;
  const int SIZE_OF_GAIN = 4;

private:
  const rclcpp::Node* node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

template <typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in) {
  const auto start = static_cast<double>(start_in);
  const auto end = static_cast<double>(end_in);
  const auto num = static_cast<double>(num_in);
  std::vector<double> linspaced(std::max(0, num_in - 1));

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num - 1);
  std::generate(linspaced.begin(), linspaced.end(), [i = 0, &delta]() mutable { return i++ * delta; });
  linspaced.push_back(end);  // I want to ensure that start and end
                             // are exactly the same as the input
  return linspaced;
}

template <typename DataType>
void SonarViewer::publishFan(const int& width,
    const int& height,
    const int& offset,
    const std::vector<DataType>& ping_data,
    const int& master_mode,
    const double& ping_range,
    const std_msgs::msg::Header& header) const {
  static_assert(std::is_same<DataType, uint8_t>::value || std::is_same<DataType, uint16_t>::value,
      "publishFan can only be build for uint8_t and uint16_t");

  const int step = width + SIZE_OF_GAIN;

  // Create rawDataMat from ping_data
  cv::Mat rawDataMat(height, step, CV_8U);
  std::memcpy(rawDataMat.data, ping_data.data() + offset, height * step);

  const double bearing = (master_mode == 1) ? LOW_FREQUENCY_BEARING_APERTURE : HIGHT_FREQUENCY_BEARING_APERTURE;
  const std::vector<double> ranges = linspace(0., ping_range, height);
  const int image_width = 2 * std::sin(bearing * M_PI / 180) * ranges.size();
  cv::Mat mono_img;
  const int cv_encoding = std::is_same<DataType, uint8_t>::value ? CV_8UC1 : CV_16UC1;
  mono_img = cv::Mat::ones(cv::Size(image_width, ranges.size()), cv_encoding) *
             std::numeric_limits<DataType>::max();  // Initialize a white image

  const float theta_shift = 1.5 * 180;  // TODO(JaouadROS, 1.5 is a magic number, what is it?)
  const cv::Point origin(image_width / 2, ranges.size());

  cv::parallel_for_(cv::Range(0, ranges.size()), [&](const cv::Range& range) {  // TODO(??, optimize for cuda)
    for (int r = range.start; r < range.end; r++) {
      std::vector<cv::Point> pts;
      cv::ellipse2Poly(origin, cv::Size(r, r), theta_shift, -bearing, bearing, 1, pts);

      std::vector<cv::Point> arc_points;
      arc_points.push_back(pts[0]);

      for (size_t k = 0; k < (pts.size() - 1); k++) {
        cv::LineIterator it(mono_img, pts[k], pts[k + 1], 4);  // TODO(JaouadROS, 4 is a magic number, what is it?)
        for (int i = 1; i < it.count; i++, ++it) arc_points.push_back(it.pos());
      }

      cv::Mat data_rows_resized;
      cv::resize(rawDataMat.row(r), data_rows_resized, cv::Size(arc_points.size(), arc_points.size()));

      for (size_t k = 0; k < arc_points.size(); k++) mono_img.at<DataType>(arc_points[k]) = data_rows_resized.at<DataType>(1, k);
    }
  });

  // Publish sonar conic image
  sensor_msgs::msg::Image msg;
  const char* ros_image_encoding =
      std::is_same<DataType, uint8_t>::value ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::MONO16;
  cv_bridge::CvImage(header, ros_image_encoding, mono_img).toImageMsg(msg);
  image_publisher_->publish(msg);
}

#endif  // OCULUS_ROS2__SONAR_VIEWER_HPP_
