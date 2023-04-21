#ifndef SONAR_VIEWER_H
#define SONAR_VIEWER_H

#include <iostream>

#include "rclcpp/rclcpp.hpp"

// #include <oculus_driver/AsyncService.h>
// #include <oculus_driver/SonarDriver.h>
// #include "oculus_interfaces/msg/ping.hpp"
#include "sensor_msgs/msg/image.hpp"

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/core.hpp>

// #include "oculus_sonar_node.hpp"

class SonarViewer
{
public:
    // SonarViewer();
    SonarViewer(rclcpp::Node::SharedPtr &node);
    ~SonarViewer();
    // void stream_and_filter(const oculus::PingMessage::ConstPtr &ping);
    // sensor_msgs::msg::Image publish_fan(const oculus::PingMessage::ConstPtr &ping);

    private:
    //     //   sensor_msgs::ImagePtr msg;
    //     cv::Mat data;
        const rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

#endif /* SONAR_VIEWER_H */