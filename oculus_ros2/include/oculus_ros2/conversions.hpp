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

#ifndef OCULUS_ROS2__CONVERSIONS_HPP_
#define OCULUS_ROS2__CONVERSIONS_HPP_

#include <oculus_driver/Oculus.h>
#include <oculus_driver/OculusMessage.h>
#include <oculus_driver/SonarDriver.h>

#include <oculus_interfaces/msg/oculus_fire_config.hpp>
#include <oculus_interfaces/msg/oculus_header.hpp>
#include <oculus_interfaces/msg/oculus_ping.hpp>
#include <oculus_interfaces/msg/oculus_status.hpp>
#include <oculus_interfaces/msg/oculus_version_info.hpp>
#include <oculus_interfaces/msg/ping.hpp>
#include <rclcpp/rclcpp.hpp>

// #include <sensor_msgs/Image.h>

namespace oculus {

inline rclcpp::Time toMsg(const SonarDriver::TimePoint& stamp) {
  size_t nano = std::chrono::duration_cast<std::chrono::nanoseconds>(stamp.time_since_epoch()).count();
  size_t seconds = nano / 1000000000;
  return rclcpp::Time(seconds, nano - 1000000000 * seconds);
}

inline void toMsg(oculus_interfaces::msg::OculusHeader& msg, const OculusMessageHeader& header) {
  msg.oculus_id = header.oculusId;
  msg.src_device_id = header.srcDeviceId;
  msg.dst_device_id = header.dstDeviceId;
  msg.msg_id = header.msgId;
  msg.msg_version = header.msgVersion;
  msg.payload_size = header.payloadSize;
  msg.spare2 = header.spare2;
}

inline void toMsg(oculus_interfaces::msg::OculusVersionInfo& msg, const OculusVersionInfo& version) {
  msg.firmware_version0 = version.firmwareVersion0;
  msg.firmware_date0 = version.firmwareDate0;
  msg.firmware_version1 = version.firmwareVersion1;
  msg.firmware_date1 = version.firmwareDate1;
  msg.firmware_version2 = version.firmwareVersion2;
  msg.firmware_date2 = version.firmwareDate2;
}

inline void toMsg(oculus_interfaces::msg::OculusStatus& msg, const OculusStatusMsg& status) {
  toMsg(msg.hdr, status.hdr);

  msg.device_id = status.deviceId;
  msg.device_type = status.deviceType;
  msg.part_number = status.partNumber;
  msg.status = status.status;

  toMsg(msg.version_info, status.versinInfo);

  msg.ip_addr = status.ipAddr;
  msg.ip_mask = status.ipMask;
  msg.connected_ip_addr = status.connectedIpAddr;

  msg.mac_addr0 = status.macAddr0;
  msg.mac_addr1 = status.macAddr1;
  msg.mac_addr2 = status.macAddr2;
  msg.mac_addr3 = status.macAddr3;
  msg.mac_addr4 = status.macAddr4;
  msg.mac_addr5 = status.macAddr5;

  msg.temperature0 = status.temperature0;
  msg.temperature1 = status.temperature1;
  msg.temperature2 = status.temperature2;
  msg.temperature3 = status.temperature3;
  msg.temperature4 = status.temperature4;
  msg.temperature5 = status.temperature5;
  msg.temperature6 = status.temperature6;
  msg.temperature7 = status.temperature7;
  msg.pressure = status.pressure;
}

inline void toMsg(oculus_interfaces::msg::OculusFireConfig& msg, const OculusSimpleFireMessage& fireConfig) {
  toMsg(msg.head, fireConfig.head);

  msg.master_mode = fireConfig.masterMode;
  msg.ping_rate = fireConfig.pingRate;
  msg.network_speed = fireConfig.networkSpeed;
  msg.gamma_correction = fireConfig.gammaCorrection;
  msg.flags = fireConfig.flags;
  msg.range = fireConfig.range;
  msg.gain_percent = fireConfig.gainPercent;
  msg.speed_of_sound = fireConfig.speedOfSound;
  msg.salinity = fireConfig.salinity;
}

inline void toMsg(oculus_interfaces::msg::OculusPing& msg, const OculusSimplePingResult& ping) {
  toMsg(msg.fire_message, ping.fireMessage);
  msg.ping_id = ping.pingId;
  msg.status = ping.status;
  msg.frequency = ping.frequency;
  msg.temperature = ping.temperature;
  msg.pressure = ping.pressure;
  msg.speeed_of_sound_used = ping.speeedOfSoundUsed;
  msg.ping_start_time = ping.pingStartTime;
  msg.data_size = ping.dataSize;
  msg.range_resolution = ping.rangeResolution;
  msg.n_ranges = ping.nRanges;
  msg.n_beams = ping.nBeams;
  msg.image_offset = ping.imageOffset;
  msg.image_size = ping.imageSize;
  msg.message_size = ping.messageSize;
}

inline void toMsg(oculus_interfaces::msg::Ping& msg, const oculus::PingMessage::ConstPtr& ping) {
  msg.header.stamp = toMsg(ping->timestamp());

  msg.ping_id = ping->ping_index();
  msg.ping_firing_date = ping->ping_firing_date();
  msg.range = ping->range();
  msg.gain_percent = ping->gain_percent();
  msg.frequency = ping->frequency();
  msg.speed_of_sound_used = ping->speed_of_sound_used();
  msg.range_resolution = ping->range_resolution();
  msg.temperature = ping->temperature();
  msg.pressure = ping->pressure();
  msg.master_mode = ping->master_mode();
  msg.has_gains = ping->has_gains();
  msg.n_ranges = ping->range_count();
  msg.n_beams = ping->bearing_count();
  msg.step = ping->step();
  msg.sample_size = ping->sample_size();

  msg.bearings.assign(ping->bearing_data(), ping->bearing_data() + ping->bearing_count());
  msg.ping_data = ping->data();
}

}  // namespace oculus

#endif  // OCULUS_ROS2__CONVERSIONS_HPP_
