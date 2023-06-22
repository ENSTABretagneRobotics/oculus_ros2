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

  const double bearing = (master_mode == 1) ? LOW_FREQUENCY_BEARING_APERTURE_ * M_PI / 180 : HIGHT_FREQUENCY_BEARING_APERTURE_ * M_PI / 180;
  const float bearing_ratio = 2 * bearing / width;
  const int negative_height = static_cast<int>(std::floor(height * std::sin(-bearing)));
  const int positive_height = static_cast<int>(std::ceil(height * std::sin(bearing)));
  const int image_width = positive_height - negative_height;
  const int origin_width = abs(negative_height);  // x coordinate of the origin
  const cv::Size image_size(image_width, height);
  cv::Mat map(image_size, CV_32FC2);
  cv::parallel_for_(cv::Range(0, map.total()), [&](const cv::Range& range)
  {
    for (auto i = range.start; i < range.end; i++)
    {
      int y = i / map.cols;
      int x = i % map.cols;

      // Calculate range and bearing of this pixel from origin
      const float dx = x - origin_width;
      const float dy = map.rows - y;

      const float range = sqrt(dx * dx + dy * dy);
      const float bearing_x_y = atan2(dx, dy);

      float xp = range;
      // Linear interpolation, TODO: use a better interpolation method
      float yp = (bearing_x_y + bearing) / bearing_ratio;

      map.at<cv::Vec2f>(cv::Point(x, y)) = cv::Vec2f(xp, yp);
    }
  });

  cv::Mat source_map_1, source_map_2;
  cv::convertMaps(map, cv::Mat(), source_map_1, source_map_2, CV_16SC2);
  cv::Mat sonar_mat_data(height, width, mat_encoding);
  for (int i = 0; i < height; ++i)
  {
    std::copy(ping_data.begin() + offset + i * step + SIZE_OF_GAIN_,  // Start at the first sonar_data after gain_data
        ping_data.begin() + offset + (i + 1) * step,  // End just before the next gain_data
        sonar_mat_data.ptr<uint8_t>(i));
  }
  cv::Mat out = cv::Mat::ones(cv::Size(image_width, height), CV_MAKETYPE(mat_encoding, 1)) * std::numeric_limits<uint8_t>::max();
  cv::remap(sonar_mat_data.t(), out, source_map_1, source_map_2, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  // Publish sonar conic image
  sensor_msgs::msg::Image msg;
  cv_bridge::CvImage(header, ros_image_encoding, out).toImageMsg(msg);
  image_publisher_->publish(msg);
}
