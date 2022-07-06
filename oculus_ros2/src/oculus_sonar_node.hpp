#include <iostream>
#include <sstream>
#include <thread>
#include <future>

#include "rclcpp/rclcpp.hpp"

#include "conversions.h"

#include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>

#include "oculus_sonar/msg/oculus_status.hpp"
#include "oculus_sonar/msg/oculus_stamped_ping.hpp"

class OculusSonarNode : public rclcpp::Node
{
  public:
    OculusSonarNode();
    ~OculusSonarNode();


  private:
    narval::oculus::SonarDriver sonar_driver_{nullptr};
    narval::oculus::AsyncService io_service_{nullptr};

    rclcpp::Publisher<oculus_sonar::msg::OculusStatus>::SharedPtr status_publisher_{nullptr};
    rclcpp::Publisher<oculus_sonar::msg::OculusStampedPing>::SharedPtr ping_publisher_{nullptr};
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_{nullptr};

    rcl_interfaces::msg::SetParametersResult set_config_callback(const std::vector<rclcpp::Parameter> & parameters);
    
    void publish_status(const OculusStatusMsg& status);
    void publish_ping(const OculusSimplePingResult& pingMetadata,
                      const std::vector<uint8_t>& pingData);
    void handle_dummy(const OculusMessageHeader& header);
};