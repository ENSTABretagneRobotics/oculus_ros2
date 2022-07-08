#include <iostream>
#include <sstream>
#include <thread>
#include <future>

#include "rclcpp/rclcpp.hpp"

#include "conversions.h"

#include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>

#include "oculus_interfaces/msg/oculus_status.hpp"
#include "oculus_interfaces/msg/oculus_stamped_ping.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

class OculusSonarNode : public rclcpp::Node
{
  public:
    OculusSonarNode();
    ~OculusSonarNode();


  private:
    std::shared_ptr<narval::oculus::SonarDriver> sonar_driver_;
    narval::oculus::AsyncService io_service_;

    std::string ping_topic_, status_topic_;
    rclcpp::Publisher<oculus_interfaces::msg::OculusStatus>::SharedPtr status_publisher_{nullptr};
    rclcpp::Publisher<oculus_interfaces::msg::OculusStampedPing>::SharedPtr ping_publisher_{nullptr};
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_{nullptr};

    rcl_interfaces::msg::SetParametersResult set_config_callback(const std::vector<rclcpp::Parameter> & parameters);
    
    void publish_status(const OculusStatusMsg& status);
    void publish_ping(const OculusSimplePingResult& pingMetadata,
                      const std::vector<uint8_t>& pingData);
    void handle_dummy();
};