#ifndef OCULUS_VIEWER_NODE_HPP_
#define OCULUS_VIEWER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

// #include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>
#include "oculus_interfaces/msg/ping.hpp"

#include "sonar_viewer.hpp"

class OculusViewerNode : public rclcpp::Node
{
public:
  OculusViewerNode();
  ~OculusViewerNode();

private:
   // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  SonarViewer sonar_viewer;
  rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr ping_subscription_;
  void ping_callback(const oculus_interfaces::msg::Ping &ping_msg) const;
};

#endif  // OCULUS_VIEWER_NODE_HPP_
