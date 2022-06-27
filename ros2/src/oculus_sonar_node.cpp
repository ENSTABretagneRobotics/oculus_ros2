#include <iostream>
#include <sstream>
#include <thread>
#include <future>
using namespace std;


#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/thread/recursive_mutex.hpp>
#include "ros/ros.h"

#include <narval_oculus/AsyncService.h>
#include <narval_oculus/SonarDriver.h>

#include <oculus_sonar/OculusStatus.h>
#include <oculus_sonar/OculusStampedPing.h>

#include <dynamic_reconfigure/server.h>
#include <oculus_sonar/OculusSonarConfig.h>

#include <conversions.h>

using SonarDriver = narval::oculus::SonarDriver;

void publish_status(ros::Publisher& publisher, const OculusStatusMsg& status)
{
    static oculus_sonar::OculusStatus msg;
    
    narval::oculus::copy_to_ros(msg, status);

    publisher.publish(msg);
}

inline ros::Time to_ros_stamp(const SonarDriver::TimePoint& stamp)
{
    size_t nano = std::chrono::duration_cast<std::chrono::nanoseconds>(
        stamp.time_since_epoch()).count();
    size_t seconds = nano / 1000000000;
    return ros::Time(seconds, nano - 1000000000*seconds);
}

void publish_ping(SonarDriver* sonarDriver,
                  ros::Publisher& publisher, 
                  const OculusSimplePingResult& pingMetadata,
                  const std::vector<uint8_t>& pingData)
{
    static oculus_sonar::OculusStampedPing msg;

    if(publisher.getNumSubscribers() == 0) {
        cout << "Going to standby mode" << endl;
        sonarDriver->standby();
        //return;
    }
    
    narval::oculus::copy_to_ros(msg.ping, pingMetadata);
    msg.ping.data.resize(pingData.size());
    for(int i = 0; i < msg.ping.data.size(); i++)
        msg.ping.data[i] = pingData[i];

    msg.header.stamp    = to_ros_stamp(sonarDriver->last_header_stamp());
    msg.header.frame_id = "oculus_sonar";
    publisher.publish(msg);
}

void handle_dummy(SonarDriver* sonarDriver, 
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

void publish_config(SonarDriver* sonarDriver,
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
    config.data_depth       = (lastConfig.flags & 0x02) ? 1 : 0;
    config.send_gain        = (lastConfig.flags & 0x04) ? 1 : 0;
    //config.full_ping        = (lastConfig.flags & 0x08) ? 1 : 0;
    config.gain_assist      = (lastConfig.flags & 0x10) ? 1 : 0;
    config.nbeams           = (lastConfig.flags & 0x40) ? 1 : 0;
    config.range            = lastConfig.range;
    config.gamma_correction = lastConfig.gammaCorrection;
    config.gain_percent     = lastConfig.gainPercent;
    config.sound_speed      = lastConfig.speedOfSound;
    config.salinity         = lastConfig.salinity;

    configServer->updateConfig(config);
}

void config_request(SonarDriver* sonarDriver, 
                    oculus_sonar::OculusSonarConfig& config,
                    int32_t level)
{
    SonarDriver::PingConfig currentConfig;
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
    currentConfig.flags = 0x09; // always in meters, simple ping
    switch(config.data_depth)
    {
        case oculus_sonar::OculusSonar_8bits:
            break;
        case oculus_sonar::OculusSonar_16bits:
            currentConfig.flags |= 0x02;
            break;
        default:break;
    }
    switch(config.nbeams)
    {
        case oculus_sonar::OculusSonar_256beams:
            break;
        case oculus_sonar::OculusSonar_512beams:
            currentConfig.flags |= 0x40;
            break;
        default:break;
    }
    if(config.send_gain)
        currentConfig.flags |= 0x04;
    if(config.gain_assist)
        currentConfig.flags |= 0x10;

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
    config.data_depth       = (feedback.flags & 0x02) ? 1 : 0;
    config.send_gain        = (feedback.flags & 0x04) ? 1 : 0;
    //config.full_ping        = (feedback.flags & 0x08) ? 1 : 0;
    config.gain_assist      = (feedback.flags & 0x10) ? 1 : 0;
    config.nbeams           = (feedback.flags & 0x40) ? 1 : 0;
    config.range            = feedback.range;
    config.gamma_correction = feedback.gammaCorrection;
    config.gain_percent     = feedback.gainPercent;
    config.sound_speed      = feedback.speedOfSound;
    config.salinity         = feedback.salinity;
}

void set_config_callback(
    dynamic_reconfigure::Server<oculus_sonar::OculusSonarConfig>* configServer,
    SonarDriver* sonarDriver)
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

    ros::Publisher statusPublisher = node.advertise<oculus_sonar::OculusStatus>(statusTopic,
                                                                                100);
    ros::Publisher pingPublisher   = node.advertise<oculus_sonar::OculusStampedPing>(pingTopic,
                                                                                     100);

    narval::oculus::AsyncService ioService;
    SonarDriver sonarDriver(ioService.io_service());
    ioService.start();
    if(!sonarDriver.wait_next_message()) {
        std::cerr << "Timeout reached while waiting for a connection to the Oculus sonar. "
                  << "Is it properly connected ?" << std::endl;
    }
    
    sonarDriver.add_status_callback(&publish_status, statusPublisher);
    sonarDriver.add_ping_callback(&publish_ping, &sonarDriver, pingPublisher);
    // callback on dummy messages to reactivate the pings as needed
    sonarDriver.add_dummy_callback(&handle_dummy, &sonarDriver, pingPublisher);


    // config server
    //boost::recursive_mutex configMutex;
    //dynamic_reconfigure::Server<oculus_sonar::OculusSonarConfig> configServer(configMutex, node);
    //sonarDriver.add_message_callback(&publish_config, &sonarDriver, &configServer, &configMutex);

    dynamic_reconfigure::Server<oculus_sonar::OculusSonarConfig> configServer(node);
    configServer.setCallback(boost::bind(&config_request, &sonarDriver, _1, _2));

    ros::spin();
    ioService.stop();

    return 0;
}
