#include <iostream>
#include <sstream>
#include <thread>
#include <future>

#include "rclcpp/rclcpp.hpp"

#include "conversions.h"

#include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>

#include "oculus_interfaces/msg/oculus_status.hpp"
#include "oculus_interfaces/msg/ping.hpp"

#include "sonar_viewer.h"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

typedef struct
{
  std::string frame_id;
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

class OculusSonarNode : public rclcpp::Node
{
public:
  OculusSonarNode();
  ~OculusSonarNode();

protected:
  const std::vector<std::string> parameters_names{"frequency_mode", "ping_rate", "data_depth", "nbeams", "gain_assist", "range", "gamma_correction", "gain_percent", "sound_speed", "use_salinity", "salinity"};
  rosParameters currentSonarParameters;
  rosParameters currentRosParameters;
  oculus::SonarDriver::PingConfig currentConfig;

  bool is_in_standby_mode = false; // Same value as ros paramater "standby"

  mutable std::shared_mutex param_mutex; ///< multithreading protection

private:
  std::shared_ptr<oculus::SonarDriver> sonar_driver_;
  oculus::AsyncService io_service_;
  SonarViewer sonar_viewer;

  std::string ping_topic_ = "ping";
  std::string status_topic_ = "status";
  rclcpp::Publisher<oculus_interfaces::msg::OculusStatus>::SharedPtr status_publisher_{nullptr};
  rclcpp::Publisher<oculus_interfaces::msg::Ping>::SharedPtr ping_publisher_{nullptr};
  const double temperature_stop_limit = 35.; // TODO(hugoyvrn, paramètre static)
  const double temperature_warn_limit = 30.; // TODO(hugoyvrn, paramètre static)

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_{nullptr};

  void update_ros_config_for_param(auto &currentSonar_param, const auto &new_param, const std::string &ros_param_name, const std::string &param_name);
  void update_ros_config_for_param(auto &currentSonar_param, const auto &new_param, const std::string &param_name);
  void update_ros_config();
  void handle_feedback_for_param(rcl_interfaces::msg::SetParametersResult &result, const rclcpp::Parameter &param, const auto &old_val, const auto &new_val, const std::string &param_name, const std::string &param_name_to_display = std::string()) const;
  void update_parameters(rosParameters &parameters, const std::vector<rclcpp::Parameter> &new_parameters);
  void update_parameters(rosParameters &parameters, oculus::SonarDriver::PingConfig feedback);
  void send_param_to_sonar(rclcpp::Parameter param, rcl_interfaces::msg::SetParametersResult result);
  rcl_interfaces::msg::SetParametersResult set_config_callback(const std::vector<rclcpp::Parameter> &parameters);

  void publish_status(const OculusStatusMsg &status);
  void publish_ping(const oculus::PingMessage::ConstPtr &pingMetadata);
  void handle_dummy();
};
