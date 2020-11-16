#include <iostream>
#include <sstream>
using namespace std;

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "ros/ros.h"

#include <narval_oculus/Sonar.h>

#include <oculus_sonar/OculusStatus.h>

#include <conversions.h>

void publish_status(ros::Publisher& publisher, const OculusStatusMsg& status)
{
    static oculus_sonar::OculusStatus msg;
    
    narval::oculus::copy_to_ros(msg, status);

    publisher.publish(msg);
}

void publish_ping(ros::Publisher& publisher, const OculusSimplePingResult& pingMetadata,
                                             const std::vector<uint8_t>& pingData)
{
    static oculus_sonar::OculusPing msg;
    
    narval::oculus::copy_to_ros(msg, pingMetadata);

    publisher.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oculus_sonar");
    ros::NodeHandle node;

    narval::oculus::Sonar sonarClient;

    ros::Publisher statusPublisher = node.advertise<oculus_sonar::OculusStatus>("status", 100);
    sonarClient.add_status_callback(&publish_status, statusPublisher);

    ros::Publisher pingPublisher = node.advertise<oculus_sonar::OculusPing>("ping", 100);
    sonarClient.add_ping_callback(&publish_ping, pingPublisher);

    sonarClient.start();

    ros::spin();

    return 0;
}
// %EndTag(FULLTEXT)%
