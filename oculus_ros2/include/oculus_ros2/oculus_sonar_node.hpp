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

#ifndef OCULUS_ROS2__OCULUS_SONAR_NODE_HPP_
#define OCULUS_ROS2__OCULUS_SONAR_NODE_HPP_

#include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>

#include <future>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <oculus_interfaces/msg/oculus_status.hpp>
#include <oculus_interfaces/msg/ping.hpp>
#include <oculus_ros2/conversions.hpp>
#include <oculus_ros2/sonar_viewer.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>

struct SonarParameters {
  int frequency_mode;
  int ping_rate;
  int data_depth;
  int nbeams;
  bool gain_assist;
  double range;
  int gamma_correction;
  double gain_percent;
  double sound_speed;
  bool use_salinity;
  double salinity;
};

namespace flagByte {
const int RANGE_AS_METERS = 0x01;  // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
const int DATA_DEPTH = 0x02;  // bit 1: 0 = 8 bit data, 1 = 16 bit data  // inverted ?
const int SEND_GAINS = 0x04;  // bit 2: 0 = won't send gain, 1 = send gain
const int SIMPLE_PING = 0x08;  // bit 3: 0 = send full return message, 1 = send simple return message
const int GAIN_ASSIST = 0x10;  // bit 4: gain assist?
// const int ?? = 0x20;  // bit 5: ?
const int NBEAMS = 0x40;  // bit 6: enable 512 beams
// const int ?? = 0x80;  // bit 7: ?
}  // namespace flagByte

namespace params {

const double TEMPERATURE_WARN_DEFAULT_VALUE = 30.;
const double TEMPERATURE_STOP_DEFAULT_VALUE = 35.;
const bool RUN_MODE_DEFAULT_VALUE = false;

struct BoolParam {
  const std::string name;
  const bool default_val;
  const std::string desc;
};

const BoolParam GAIN_ASSIT = {"gain_assist", true, ""};
const BoolParam USE_SALINITY = {"use_salinity", true, "Use salinity to calculate sound_speed."};

const std::vector<BoolParam> BOOL = {GAIN_ASSIT, USE_SALINITY};

struct IntParam {
  const std::string name;
  const int min;
  const int max;
  const int default_val;
  const std::string desc;
};

const IntParam FREQUENCY_MODE = {"frequency_mode", 0, 1, 0,
    "Sonar beam frequency mode.\n"
    "\t1: Low frequency (1.2MHz, wide aperture).\n"
    "\t2: High frequency (2.1Mhz, narrow aperture)."};

const IntParam PING_RATE = {"ping_rate", 0, 5, 2,
    "Frequency of ping fires.\n\t" + std::to_string(pingRateNormal) + ": 10Hz max ping rate.\n\t" + std::to_string(pingRateHigh) +
        ": 15Hz max ping rate.\n\t" + std::to_string(pingRateHighest) + ": 40Hz max ping rate.\n\t" +
        std::to_string(pingRateLow) + ": 5Hz max ping rate.\n\t" + std::to_string(pingRateLowest) + ": 2Hz max ping rate.\n\t" +
        static_cast<char>(pingRateStandby) + ": Standby mode (no ping fire)."};
const IntParam DATA_DEPTH = {"data_depth", 0, 1, 1,
    "Ping data encoding bit count.\n"
    "\t0: Ping data encoded on 8bits.\n"
    "\t1: Ping data encoded on 16bits."};
const IntParam NBEAMS = {"nbeams", 0, 1, 1,
    "Number of ping beams.\n"
    "\t0: Oculus outputs 256 beams.\n"
    "\t1: Oculus outputs 512 beams."};
const IntParam GAMMA_CORRECTION = {"gamma_correction", 0, 255, 153, "Gamma correction, min=0, max=255."};

const std::vector<IntParam> INT = {FREQUENCY_MODE, PING_RATE, DATA_DEPTH, NBEAMS, GAMMA_CORRECTION};

struct DoubleParam {
  const std::string name;
  const double min;
  const double max;
  const double step;
  const double default_val;
  const std::string desc;
};

const DoubleParam RANGE = {"range", .3, 40., .1, 20., "Sonar range (in meters), min=0.3, max=40.0."};
const DoubleParam GAIN_PERCENT = {"gain_percent", .1, 100., .1, 50., "Gain percentage (%), min=0.1, max=100.0."};
const DoubleParam SOUND_SPEED = {"sound_speed", 1400., 1600., .1, 1500.,
    "Sound speed (in m/s, set to 0 for it to be calculated using salinity), min=1400.0, max=1600.0."};
const DoubleParam SALINITY = {"salinity", 0., 100., .1, 0.,
    "Salinity (in parts per thousand (ppt,ppm,g/kg), "
    "used to calculate sound speed if needed), min=0.0, max=100"};

const std::vector<DoubleParam> DOUBLE = {RANGE, GAIN_PERCENT, SOUND_SPEED, SALINITY};

}  // namespace params

class OculusSonarNode : public rclcpp::Node {
public:
  OculusSonarNode();
  ~OculusSonarNode();

protected:
  const std::vector<std::string> dynamic_parameters_names_{"frequency_mode", "ping_rate", "data_depth", "nbeams", "gain_assist",
      "range", "gamma_correction", "gain_percent", "sound_speed", "use_salinity", "salinity", "run"};

  SonarParameters currentSonarParameters_;
  SonarParameters currentRosParameters_;
  oculus::SonarDriver::PingConfig currentConfig_;

  bool is_running_;  // State value. Same value as ros parameter "run"
  bool is_overheating_ = false;  // State value

  mutable std::shared_mutex param_mutex_;  // multithreading protection

  int get_subscription_count() const;

private:
  std::shared_ptr<oculus::SonarDriver> sonar_driver_;
  oculus::AsyncService io_service_;
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  SonarViewer sonar_viewer_;
  const std::string frame_id_;
  const double temperature_warn_limit_;
  const double temperature_stop_limit_;
  rclcpp::Publisher<oculus_interfaces::msg::OculusStatus>::SharedPtr status_publisher_{nullptr};
  rclcpp::Publisher<oculus_interfaces::msg::Ping>::SharedPtr ping_publisher_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_publisher_{nullptr};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_{nullptr};

  template <class T>
  void updateRosConfigForParam(T& currentSonar_param, const T& new_param, const std::string& param_name);
  void updateRosConfig();
  template <class T>
  void handleFeedbackForParam(rcl_interfaces::msg::SetParametersResult& result,
      const rclcpp::Parameter& param,
      const T& old_val,
      const T& new_val,
      const std::string& param_name) const;
  void updateLocalParameters(SonarParameters& parameters, const std::vector<rclcpp::Parameter>& new_parameters);
  void updateLocalParameters(SonarParameters& parameters, oculus::SonarDriver::PingConfig feedback);
  void sendParamToSonar(rclcpp::Parameter param, rcl_interfaces::msg::SetParametersResult result);
  rcl_interfaces::msg::SetParametersResult setConfigCallback(const std::vector<rclcpp::Parameter>& parameters);

  void enableRunMode();
  void disableRunMode();
  void checkOverheating(const double& new_temperature);
  void checkFlag(uint8_t flags);
  void publishStatus(const OculusStatusMsg& status);
  void publishPing(const oculus::PingMessage::ConstPtr& pingMetadata);
  void handleDummy();
};

template <class T>
void OculusSonarNode::updateRosConfigForParam(T& currentSonar_param, const T& new_param, const std::string& param_name) {
  if (currentSonar_param != new_param) {
    this->remove_on_set_parameters_callback(this->param_cb_.get());
    RCLCPP_WARN_STREAM(this->get_logger(),
        "The parameter " << param_name << " has change by it self from " << currentSonar_param << " to " << new_param);
    currentSonar_param = new_param;
    this->set_parameter(rclcpp::Parameter(param_name, new_param));
    this->param_cb_ =
        this->add_on_set_parameters_callback(std::bind(&OculusSonarNode::setConfigCallback, this, std::placeholders::_1));
  }
}

template <class T>
void OculusSonarNode::handleFeedbackForParam(rcl_interfaces::msg::SetParametersResult& result,
    const rclcpp::Parameter& param,
    const T& old_val,
    const T& new_val,
    const std::string& param_name) const {
  if (old_val != new_val) {
    if (param.get_name() == param_name) {
      result.successful = false;
      RCLCPP_WARN_STREAM(this->get_logger(), "Could not update " << param_name);
      result.reason.append("Could not update " + param_name + ".\n");
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(),
          param_name << " change from " << old_val << " to " << new_val << " when updating the parameter " << param.get_name());
      result.reason.append(param_name + " change.\n");
    }
  }
}

#endif  // OCULUS_ROS2__OCULUS_SONAR_NODE_HPP_
