#ifndef SONAR_VIEWER_H
#define SONAR_VIEWER_H

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>
#include "oculus_interfaces/msg/ping.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>

class SonarViewer
{
public:
    // SonarViewer(bool *arg);
    SonarViewer(rclcpp::Node *node);
    ~SonarViewer();
    void publish_fan(const oculus::PingMessage::ConstPtr &ping) const;
    void publish_fan(const oculus_interfaces::msg::Ping &ros_ping_msg) const;
    void publish_fan(const int &width, const int &height, const int &offset, const std::vector<uint8_t> &ping_data, const int &master_mode, const double &ping_rage) const;
    void stream_and_filter(const oculus::PingMessage::ConstPtr &ping, cv::Mat &data);
    // void stream_and_filter(const oculus::PingMessage::ConstPtr &ping);
    // sensor_msgs::msg::Image publish_fan(const oculus::PingMessage::ConstPtr &ping);

private:
    //     //   sensor_msgs::ImagePtr msg;
    const rclcpp::Node *node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

template <typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in)
{
    std::vector<double> linspaced;

    double start = static_cast<double>(start_in);
    double end = static_cast<double>(end_in);
    double num = static_cast<double>(num_in);

    if (num == 0)
    {
        return linspaced;
    }
    if (num == 1)
    {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for (int i = 0; i < num - 1; ++i)
    {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
                              // are exactly the same as the input
    return linspaced;
}

#endif /* SONAR_VIEWER_H */