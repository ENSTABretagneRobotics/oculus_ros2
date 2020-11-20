#include <iostream>
#include <sstream>
using namespace std;

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "ros/ros.h"

#include <narval_oculus/Sonar.h>

#include <oculus_sonar/OculusStatus.h>

#include <dynamic_reconfigure/server.h>
#include <oculus_sonar/OculusSonarConfig.h>

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
    msg.data.resize(pingData.size());
    for(int i = 0; i < msg.data.size(); i++)
        msg.data[i] = pingData[i];

    publisher.publish(msg);
}

std::ostream& operator<<(std::ostream& os, const oculus_sonar::OculusSonarConfig& config)
{
    cout << "Got new config :"
         << "\n  - frequency_mode    : " << config.frequency_mode
         << "\n  - ping_rate         : " << config.ping_rate
         << "\n  - network speed     : " << config.network_speed
         << "\n  - gamma correction  : " << config.gamma_correction
         << "\n  - data depth        : " << config.data_depth
         << "\n  - send gain         : " << config.send_gain
         << "\n  - hf_range          : " << config.hf_range;
    return os;
}

void config_request(narval::oculus::SonarClient* sonarClient, oculus_sonar::OculusSonarConfig& config, uint32_t level)
{
    auto currentConfig = sonarClient->current_fire_config();
    cout << "============= current config :\n"
         << currentConfig << endl;
    cout << config << endl
         << "Level : " << level << endl;

    currentConfig.masterMode = config.frequency_mode;
    switch(config.ping_rate)
    {
        case 0: currentConfig.pingRate = pingRateNormal;  break;
        case 1: currentConfig.pingRate = pingRateHigh;    break;
        case 2: currentConfig.pingRate = pingRateHighest; break;
        case 3: currentConfig.pingRate = pingRateLow;     break;
        case 4: currentConfig.pingRate = pingRateLowest;  break;
        case 5: currentConfig.pingRate = pingRateStandby; break;
        default:break;
    }
    currentConfig.networkSpeed    = config.network_speed;
    currentConfig.gammaCorrection = config.gamma_correction;

    // flags
    currentConfig.flags = 0x9; // always in meters, simple ping
    switch(config.data_depth)
    {
        case oculus_sonar::OculusSonar_8bits:
            break;
        case oculus_sonar::OculusSonar_16bits:
            currentConfig.flags = currentConfig.flags | 0x2;
            break;
        default:break;
    }
    if(config.send_gain)
        currentConfig.flags = currentConfig.flags | 0x4;

    currentConfig.range        = config.hf_range;
    //currentConfig.gainPercent  = config.
    currentConfig.speedOfSound = 0.0;
    currentConfig.salinity     = 0.0;

    currentConfig.head.srcDeviceId = 0;
    currentConfig.head.dstDeviceId = 0;
    sonarClient->send_fire_config(currentConfig);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oculus_sonar");
    ros::NodeHandle node;

    narval::oculus::Sonar sonarClient;
    
    // sonar status publisher
    ros::Publisher statusPublisher = node.advertise<oculus_sonar::OculusStatus>("status", 100);
    sonarClient.add_status_callback(&publish_status, statusPublisher);

    // ping publisher
    ros::Publisher pingPublisher = node.advertise<oculus_sonar::OculusPing>("ping", 100);
    sonarClient.add_ping_callback(&publish_ping, pingPublisher);

    // config server
    dynamic_reconfigure::Server<oculus_sonar::OculusSonarConfig> configServer;
    configServer.setCallback(boost::bind(&config_request, &sonarClient, _1, _2));

    sonarClient.start();

    ros::spin();

    sonarClient.stop();

    return 0;
}
// %EndTag(FULLTEXT)%
