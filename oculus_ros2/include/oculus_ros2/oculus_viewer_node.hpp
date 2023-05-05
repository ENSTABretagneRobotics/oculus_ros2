// Copyright 2023 Forssea Robotics
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#ifndef OCULUS_ROS2__OCULUS_VIEWER_NODE_HPP_
#define OCULUS_ROS2__OCULUS_VIEWER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

// #include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>

#include <oculus_interfaces/msg/ping.hpp>
#include <oculus_ros2/sonar_viewer.hpp>

class OculusViewerNode : public rclcpp::Node {
public:
  OculusViewerNode();
  ~OculusViewerNode();

private:
  // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  SonarViewer sonar_viewer_;
  rclcpp::Subscription<oculus_interfaces::msg::Ping>::SharedPtr ping_subscription_;
  void ping_callback(const oculus_interfaces::msg::Ping& ping_msg) const;
};

#endif  // OCULUS_ROS2__OCULUS_VIEWER_NODE_HPP_
