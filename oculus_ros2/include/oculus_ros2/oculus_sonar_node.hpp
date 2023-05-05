// Copyright 2023 Forssea Robotics
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

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

typedef struct {
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
} rosParameters;

class OculusSonarNode : public rclcpp::Node {
public:
  OculusSonarNode();
  ~OculusSonarNode();

protected:
  const std::vector<std::string> dynamic_parameters_names_{"frequency_mode", "ping_rate", "data_depth", "nbeams", "gain_assist",
      "range", "gamma_correction", "gain_percent", "sound_speed", "use_salinity", "salinity", "run"};
  rosParameters currentSonarParameters_;
  rosParameters currentRosParameters_;
  oculus::SonarDriver::PingConfig currentConfig_;

  bool is_in_run_mode_;  // Same value as ros parameter "run"

  mutable std::shared_mutex param_mutex_;  // multithreading protection

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
  void updateRosConfigForParam(
      T& currentSonar_param, const T& new_param, const std::string& ros_param_name, const std::string& param_name);
  template <class T>
  void updateRosConfigForParam(T& currentSonar_param, const T& new_param, const std::string& param_name);
  void updateRosConfig();
  template <class T>
  void handleFeedbackForParam(rcl_interfaces::msg::SetParametersResult& result,
      const rclcpp::Parameter& param,
      const T& old_val,
      const T& new_val,
      const std::string& param_name,
      const std::string& param_name_to_display = std::string()) const;
  void updateParameters(rosParameters& parameters, const std::vector<rclcpp::Parameter>& new_parameters);
  void updateParameters(rosParameters& parameters, oculus::SonarDriver::PingConfig feedback);
  void sendParamToSonar(rclcpp::Parameter param, rcl_interfaces::msg::SetParametersResult result);
  rcl_interfaces::msg::SetParametersResult setConfigCallback(const std::vector<rclcpp::Parameter>& parameters);

  void enableRunMode();
  void desableRunMode();
  void publishStatus(const OculusStatusMsg& status) const;
  void publishPing(const oculus::PingMessage::ConstPtr& pingMetadata);
  void handleDummy() const;
};

template <class T>
void OculusSonarNode::updateRosConfigForParam(T& currentSonar_param, const T& new_param, const std::string& param_name) {
  updateRosConfigForParam(currentSonar_param, new_param, param_name, param_name);
}

template <class T>
void OculusSonarNode::updateRosConfigForParam(
    T& currentSonar_param, const T& new_param, const std::string& ros_param_name, const std::string& param_name) {
  if (currentSonar_param != new_param) {
    this->remove_on_set_parameters_callback(this->param_cb_.get());
    RCLCPP_WARN_STREAM(this->get_logger(),
        "The parameter " << param_name << " has change by it self from " << currentSonar_param << " to " << new_param);
    currentSonar_param = new_param;
    this->set_parameter(rclcpp::Parameter(ros_param_name, new_param));
    this->param_cb_ =
        this->add_on_set_parameters_callback(std::bind(&OculusSonarNode::setConfigCallback, this, std::placeholders::_1));
  }
}

template <class T>
void OculusSonarNode::handleFeedbackForParam(rcl_interfaces::msg::SetParametersResult& result,
    const rclcpp::Parameter& param,
    const T& old_val,
    const T& new_val,
    const std::string& param_name,
    const std::string& param_name_to_display) const {
  if (old_val != new_val) {
    std::string param_name_to_display = param_name_to_display == "" ? param_name : param_name_to_display;
    if (param.get_name() == param_name) {
      // if (parameters.size() == 1)
      result.successful = false;
      RCLCPP_WARN_STREAM(this->get_logger(), "Could not update " << param_name_to_display);
      result.reason.append("Could not update " + param_name_to_display + ".\n");
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(), param_name_to_display << " change from " << old_val << " to " << new_val
                                                                    << " when updating the parameter " << param.get_name());
      result.reason.append(param_name_to_display + " change.\n");
    }
  }
}

#endif  // OCULUS_ROS2__OCULUS_SONAR_NODE_HPP_
