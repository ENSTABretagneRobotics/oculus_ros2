#include <iostream>
#include <sstream>
#include <thread>
#include <future>
using namespace std;


#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/thread/recursive_mutex.hpp>
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

void publish_ping(narval::oculus::SonarDriver* sonarDriver,
                  ros::Publisher& publisher, 
                  const OculusSimplePingResult& pingMetadata,
                  const std::vector<uint8_t>& pingData)
{
    static oculus_sonar::OculusPing msg;

    if(publisher.getNumSubscribers() == 0) {
        cout << "Going to standby mode" << endl;
        sonarDriver->standby();
        return;
    }
    
    narval::oculus::copy_to_ros(msg, pingMetadata);
    msg.data.resize(pingData.size());
    for(int i = 0; i < msg.data.size(); i++)
        msg.data[i] = pingData[i];

    publisher.publish(msg);
}

void handle_dummy(narval::oculus::SonarDriver* sonarDriver, 
                  ros::Publisher& pingPublisher,
                  const OculusMessageHeader& header)
{
    if(pingPublisher.getNumSubscribers() > 0) {
        cout << "Exiting standby mode" << endl;
        sonarDriver->resume();
    }
}

std::ostream& operator<<(std::ostream& os, const oculus_sonar::OculusSonarConfig& config)
{
    cout << "Got new config :"
         << "\n  - frequency_mode    : " << config.frequency_mode
         << "\n  - ping_rate         : " << config.ping_rate
         << "\n  - data depth        : " << config.data_depth
         << "\n  - send gain         : " << config.send_gain
         << "\n  - range             : " << config.range
         << "\n  - gamma correction  : " << config.gamma_correction
         << "\n  - gain percent      : " << config.gain_percent
         << "\n  - sound speed       : " << config.sound_speed
         << "\n  - use salinity      : " << config.use_salinity
         << "\n  - salinity          : " << config.salinity;
    return os;
}

void publish_config(narval::oculus::SonarDriver* sonarDriver,
    dynamic_reconfigure::Server<oculus_sonar::OculusSonarConfig>* configServer,
    boost::recursive_mutex* configMutex,
    const OculusMessageHeader& header, const std::vector<uint8_t>& data)
{
    oculus_sonar::OculusSonarConfig config;
    boost::recursive_mutex::scoped_lock lock(*configMutex);

    // Publishing config only if we have a dummy or simple ping
    switch(header.msgId) {
        case messageSimplePingResult: break;
        case messageDummy:            break;
        default:                      return;
    };

    auto lastConfig = sonarDriver->last_ping_config();

    config.frequency_mode   = lastConfig.masterMode;
    config.ping_rate        = lastConfig.pingRate; // is broken (?) sonar side
    config.data_depth       = lastConfig.flags & 0x2;
    config.send_gain        = lastConfig.flags & 0x4;
    config.range            = lastConfig.range;
    config.gamma_correction = lastConfig.gammaCorrection;
    config.gain_percent     = lastConfig.gainPercent;
    config.sound_speed      = lastConfig.speedOfSound;
    config.salinity         = lastConfig.salinity;

    configServer->updateConfig(config);
}

void config_request(narval::oculus::SonarDriver* sonarDriver, 
                    oculus_sonar::OculusSonarConfig& config,
                    uint32_t level)
{
    // std::cout << "Received config : " << level << std::endl;
    // on node launch, the configuration server asks for current configuration
    // by setting level to the maximum possible value.
    if(level == std::numeric_limits<uint32_t>::max()) {
        auto lastConfig = sonarDriver->last_ping_config();
        config.frequency_mode   = lastConfig.masterMode;
        config.ping_rate        = 0;
        config.data_depth       = lastConfig.flags & 0x2;
        config.send_gain        = lastConfig.flags & 0x4;
        config.range            = lastConfig.range;
        config.gamma_correction = lastConfig.gammaCorrection;
        config.gain_percent     = lastConfig.gainPercent;
        config.sound_speed      = lastConfig.speedOfSound;
        config.salinity         = lastConfig.salinity;
        return;
    }

    narval::oculus::SonarDriver::PingConfig currentConfig;
    std::memset(&currentConfig, 0, sizeof(currentConfig));

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

    // flags
    currentConfig.flags = 0x9; // always in meters, simple ping
    switch(config.data_depth)
    {
        case oculus_sonar::OculusSonar_8bits:
            break;
        case oculus_sonar::OculusSonar_16bits:
            currentConfig.flags |= 0x2;
            break;
        default:break;
    }
    if(config.send_gain)
        currentConfig.flags |= 0x4;

    currentConfig.range           = config.range;
    currentConfig.gammaCorrection = config.gamma_correction;
    currentConfig.gainPercent     = config.gain_percent;

    if(config.use_salinity)
        currentConfig.speedOfSound = 0.0;
    else
        currentConfig.speedOfSound = config.sound_speed;
    currentConfig.salinity     = config.salinity;
    
    //sonarDriver->send_ping_config(currentConfig);
    // // a timeout would be nice
    auto feedback = sonarDriver->request_ping_config(currentConfig);
    config.frequency_mode   = feedback.masterMode;
    //config.ping_rate      = feedback.pingRate // is broken (?) sonar side
    config.data_depth       = feedback.flags & 0x2;
    config.send_gain        = feedback.flags & 0x4;
    config.range            = feedback.range;
    config.gamma_correction = feedback.gammaCorrection;
    config.gain_percent     = feedback.gainPercent;
    config.sound_speed      = feedback.speedOfSound;
    config.salinity         = feedback.salinity;
}

void set_config_callback(dynamic_reconfigure::Server<oculus_sonar::OculusSonarConfig>* configServer,
                         narval::oculus::Sonar* sonarDriver)
{
    configServer->setCallback(boost::bind(&config_request, sonarDriver, _1, _2));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "oculus_sonar");

    // Setting up namespace to node name (why not by default ??)
    std::string nodeName = ros::this_node::getName();
    if(nodeName[0] == '/') {
        nodeName = nodeName.substr(1, nodeName.size() - 1);
    }
    ros::NodeHandle node(nodeName);
    
    std::string pingTopic, statusTopic;
    node.param<std::string>("ping_topic",   pingTopic,   "ping");
    node.param<std::string>("status_topic", statusTopic, "status");

    narval::oculus::Sonar sonarDriver;
    
    // sonar status publisher
    ros::Publisher statusPublisher = node.advertise<oculus_sonar::OculusStatus>(statusTopic, 100);
    sonarDriver.add_status_callback(&publish_status, statusPublisher);

    // ping publisher
    ros::Publisher pingPublisher = node.advertise<oculus_sonar::OculusPing>(pingTopic, 100);
    sonarDriver.add_ping_callback(&publish_ping, &sonarDriver, pingPublisher);

    // callback on dummy messages to reactivate the pings as needed
    sonarDriver.add_dummy_callback(&handle_dummy, &sonarDriver, pingPublisher);

    // config server
    //boost::recursive_mutex configMutex;
    //dynamic_reconfigure::Server<oculus_sonar::OculusSonarConfig> configServer(configMutex, node);
    //sonarDriver.add_message_callback(&publish_config, &sonarDriver, &configServer, &configMutex);

    dynamic_reconfigure::Server<oculus_sonar::OculusSonarConfig> configServer(node);
    configServer.setCallback(boost::bind(&config_request, &sonarDriver, _1, _2));

    sonarDriver.start();
    ros::spin();
    sonarDriver.stop();

    return 0;
}
