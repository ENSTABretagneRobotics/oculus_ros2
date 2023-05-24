/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, ENSTA-Bretagne
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
  void publishFan(const oculus::PingMessage::ConstPtr& ping, const int& data_depth, const std::string& frame_id = "") const;
  void publishFan(
      const oculus_interfaces::msg::Ping& ros_ping_msg, const int& data_depth, const std::string& frame_id = "") const;
  template <typename DataType>
  void publishFan(const int& width,
      const int& height,
      const int& offset,
      const std::vector<uint8_t>& ping_data,
      const int& master_mode,
      const std_msgs::msg::Header& header) const;

protected:
  const double LOW_FREQUENCY_BEARING_APERTURE_ = 65.;
  const double HIGHT_FREQUENCY_BEARING_APERTURE_ = 40.;
  const int SIZE_OF_GAIN_ = 4;

private:
  const rclcpp::Node* node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

template <typename DataType>
void SonarViewer::publishFan(const int& width,
    const int& height,
    const int& offset,
    const std::vector<uint8_t>& ping_data,
    const int& master_mode,
    const std_msgs::msg::Header& header) const {
  static_assert(std::is_same<DataType, uint8_t>::value || std::is_same<DataType, uint16_t>::value,
      "publishFan can only be build for uint8_t and uint16_t");

  const int step = width + SIZE_OF_GAIN_;

  // Create rawDataMat from ping_data
  cv::Mat rawDataMat(height, step, CV_8U);
  std::memcpy(rawDataMat.data, ping_data.data() + offset, height * step);

  const double bearing = (master_mode == 1) ? LOW_FREQUENCY_BEARING_APERTURE_ : HIGHT_FREQUENCY_BEARING_APERTURE_;
  const int image_width = 2 * std::sin(bearing * M_PI / 180) * height;
  cv::Mat mono_img;
  if constexpr (std::is_same<DataType, uint8_t>::value) {
    mono_img = cv::Mat::ones(cv::Size(image_width, height), CV_8UC1) * std::numeric_limits<DataType>::max();
  } else {
    mono_img = cv::Mat::ones(cv::Size(image_width, height), CV_16UC1) * std::numeric_limits<DataType>::max();
  }

  const float theta_shift = 1.5 * 180;  // TODO(JaouadROS, 1.5 is a magic number, what is it?)
  const cv::Point origin(image_width / 2, height);

  cv::parallel_for_(cv::Range(0, height), [&](const cv::Range& range) {  // TODO(??, optimize for cuda)
    for (int r = range.start; r < range.end; r++) {
      std::vector<cv::Point> pts;
      cv::ellipse2Poly(origin, cv::Size(r, r), theta_shift, -bearing, bearing, 1, pts);

      std::vector<cv::Point> arc_points;
      arc_points.push_back(pts[0]);

      for (size_t k = 0; k < (pts.size() - 1); k++) {
        cv::LineIterator it(mono_img, pts[k], pts[k + 1], 4);  // TODO(JaouadROS, 4 is a magic number, what is it?)
        for (int i = 1; i < it.count; i++, ++it) {
          arc_points.push_back(it.pos());
        }
      }

      cv::Mat data_rows_resized;
      cv::resize(rawDataMat.row(r), data_rows_resized, cv::Size(arc_points.size(), arc_points.size()));

      for (size_t k = 0; k < arc_points.size(); k++) {
        mono_img.at<DataType>(arc_points[k]) = data_rows_resized.at<DataType>(1, k);
      }
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
