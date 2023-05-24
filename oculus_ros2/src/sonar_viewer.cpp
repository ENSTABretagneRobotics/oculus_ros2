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

#include <oculus_ros2/sonar_viewer.hpp>

SonarViewer::SonarViewer(rclcpp::Node* node) : node_(node) {
  image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("image", 10);
}

SonarViewer::~SonarViewer() {}

void SonarViewer::publishFan(
    const oculus_interfaces::msg::Ping& ros_ping_msg, const int& data_depth, const std::string& frame_id) const {
  // const int offset = ping->ping_data_offset(); // TODO(hugoyvrn)
  const int offset = 229;  // TODO(JaouadROS, 229 is a magic number)

  if (data_depth == 0) {
    publishFan<uint8_t>(ros_ping_msg.n_beams, ros_ping_msg.n_ranges, offset, ros_ping_msg.ping_data, ros_ping_msg.master_mode,
      ros_ping_msg.header);
  } else {
    publishFan<uint16_t>(ros_ping_msg.n_beams, ros_ping_msg.n_ranges, offset, ros_ping_msg.ping_data, ros_ping_msg.master_mode,
      ros_ping_msg.header);
  }
}

void SonarViewer::publishFan(
    const oculus::PingMessage::ConstPtr& ping, const int& data_depth, const std::string& frame_id) const {
  std_msgs::msg::Header header;
  header.stamp = oculus::toMsg(ping->timestamp());
  header.frame_id = frame_id;
  // publishFan<data_depth == 0 ? uint8_t : uint16_t>(ping->bearing_count(), ping->range_count(), ping->ping_data_offset(),
  //     ping->data(), ping->master_mode(), ping->range(), header);
  if (data_depth == 0) {
    publishFan<uint8_t>(ping->bearing_count(), ping->range_count(), ping->ping_data_offset(), ping->data(), ping->master_mode(),
        header);
  } else {
    publishFan<uint8_t>(ping->bearing_count(), ping->range_count(), ping->ping_data_offset(), ping->data(), ping->master_mode(),
        header);  // TODO(hugoyvrn, handle publishFan<uint16_t> for 16bits data depth)
  }
}
