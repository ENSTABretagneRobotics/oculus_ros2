/**
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

#include <oculus_ros2/sonar_viewer.hpp>

SonarViewer::SonarViewer(rclcpp::Node* node) : node_(node) {
  image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("image", 10);
}

SonarViewer::~SonarViewer() {}

void SonarViewer::publishFan(const oculus_interfaces::msg::Ping& ros_ping_msg) const {
  // const int offset = ping->ping_data_offset(); // TODO(hugoyvrn)
  const int offset = -16;  // quick fix TODO(hugoyvrn, why 229?)

  publishFan(
      ros_ping_msg.n_beams, ros_ping_msg.n_ranges, offset, ros_ping_msg.ping_data, ros_ping_msg.master_mode, ros_ping_msg.header);
}

void SonarViewer::publishFan(const oculus::PingMessage::ConstPtr& ping, const std::string& frame_id) const {
  std_msgs::msg::Header header;
  header.stamp = oculus::toMsg(ping->timestamp());
  header.frame_id = frame_id;

  if (!ping->has_gains()) {
    RCLCPP_WARN(node_->get_logger(), "Gains are not send by the sonar. The conic image view is wrong.");
  }
  publishFan(ping->bearing_count(), ping->range_count(), ping->ping_data_offset(), ping->data(), ping->master_mode(), header);
}

void SonarViewer::publishFan(const int& width,
    const int& height,
    const int& offset,
    const std::vector<uint8_t>& ping_data,
    const int& master_mode,
    const std_msgs::msg::Header& header) const {
  const int step = width + SIZE_OF_GAIN_;
  const float theta_shift = 270;
  const int mat_encoding = CV_8U;
  const char* ros_image_encoding = sensor_msgs::image_encodings::MONO8;

  cv::Mat rawDataMat(height, step, mat_encoding);

  std::memcpy(rawDataMat.data, ping_data.data() + offset, height * step);

  {  // Correct range gainsfloat gain_min =
    float raw_gain_min = std::numeric_limits<float>::max();
    float raw_gain_max = std::numeric_limits<float>::min();
    for (int i = 0; i < height; i++) {
      raw_gain_min = std::min(raw_gain_min, static_cast<float>(rawDataMat.at<uint32_t>(i, 0)));
      raw_gain_max = std::max(raw_gain_max, static_cast<float>(rawDataMat.at<uint32_t>(i, 0)));
    }
    const float gain_nomalization = std::sqrt(raw_gain_max) / 255;
    for (int i = 0; i < height; i++) {
      const float gain_i = gain_nomalization / std::sqrt(rawDataMat.at<uint32_t>(i, 0));
      for (int j = SIZE_OF_GAIN_; j < step; j++) {
        const float new_pixel_val = rawDataMat.at<uint8_t>(i, j) * gain_i;
        rawDataMat.at<uint8_t>(i, j) = std::min(std::max(static_cast<float>(0), new_pixel_val), static_cast<float>(255));
      }
    }
  }

  const double bearing = (master_mode == 1) ? LOW_FREQUENCY_BEARING_APERTURE_ : HIGHT_FREQUENCY_BEARING_APERTURE_;
  const int image_width = 2 * std::sin(bearing * M_PI / 180) * height;
  cv::Mat mono_img =
      cv::Mat::ones(cv::Size(image_width, height), CV_MAKETYPE(mat_encoding, 1)) * std::numeric_limits<uint8_t>::max();
  const cv::Point origin(image_width / 2, height);

  cv::parallel_for_(cv::Range(0, height), [&](const cv::Range& range) {  // TODO(??, optimize for cuda)
    for (int r = range.start; r < range.end; r++) {
      std::vector<cv::Point> pts;
      cv::ellipse2Poly(origin, cv::Size(r, r), theta_shift, -bearing, bearing, 1, pts);

      std::vector<cv::Point> arc_points;
      arc_points.push_back(pts[0]);

      for (size_t k = 0; k < (pts.size() - 1); k++) {
        cv::LineIterator it(mono_img, pts[k], pts[k + 1], SIZE_OF_GAIN_);
        for (int i = 1; i < it.count; i++, ++it) {
          arc_points.push_back(it.pos());
        }
      }

      cv::Mat data_rows_resized;
      cv::resize(rawDataMat.row(r), data_rows_resized, cv::Size(arc_points.size(), arc_points.size()));

      for (size_t k = 0; k < arc_points.size(); k++) {
        mono_img.at<uint8_t>(arc_points[k]) = data_rows_resized.at<uint8_t>(1, k);
      }
    }
  });

  // Publish sonar conic image
  sensor_msgs::msg::Image msg;
  cv_bridge::CvImage(header, ros_image_encoding, mono_img).toImageMsg(msg);
  image_publisher_->publish(msg);
}
