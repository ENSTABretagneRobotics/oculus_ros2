#ifndef SONAR_VIEWER_H
#define SONAR_VIEWER_H

#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>
#include "oculus_interfaces/msg/ping.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>

class SonarViewer
{
public:
    SonarViewer();
    ~SonarViewer();
    void publish_fan(const oculus::PingMessage::ConstPtr &ping);

private:
    cv::Mat data;
};

#endif /* SONAR_VIEWER_H */