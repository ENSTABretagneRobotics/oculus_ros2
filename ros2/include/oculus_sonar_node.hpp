#include <iostream>
#include <sstream>
#include <thread>
#include <future>

#include "rclcpp/rclcpp.hpp"

#include <narval_oculus/AsyncService.h>
#include <narval_oculus/SonarDriver.h>

#include <oculus_sonar/OculusStatus.h>
#include <oculus_sonar/OculusStampedPing.h>

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
    
    void publish_status(rclcpp::Publisher<oculus_sonar::msg::OculusStatus>::SharedPtr publisher, 
                        const OculusStatusMsg& status);
    void publish_ping(narval::oculus::SonarDriver* sonarDriver,
                      rclcpp::Publisher<oculus_sonar::msg::OculusStampedPing>::SharedPtr publisher, 
                      const OculusSimplePingResult& pingMetadata,
                      const std::vector<uint8_t>& pingData);
    void handle_dummy(narval::oculus::SonarDriver* sonarDriver, 
                      rclcpp::Publisher<oculus_sonar::msg::OculusStampedPing>::SharedPtr pingPublisher,
                      const OculusMessageHeader& header);
};