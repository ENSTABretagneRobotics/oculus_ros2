#include <iostream>
#include <cstring>
#include <sstream>
#include <limits>
using namespace std;

#include <narval_display/Display.h>
#include <narval_display/GLVector.h>
#include <narval_display/renderers/ImageRenderer.h>
using namespace narval::display;
using Shape = ImageRenderer::Shape;


#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "ros/ros.h"

#include <narval_oculus/Sonar.h>

#include <oculus_sonar/OculusStatus.h>

#include <dynamic_reconfigure/server.h>
#include <oculus_sonar/OculusSonarConfig.h>

#include <conversions.h>

// Have to copy because opengl doesnot play well outside of main thread
bool displayable = false;
OculusSimplePingResult pingMetadata_;
std::vector<uint8_t>   pingData_;
void update_display_data(const OculusSimplePingResult& pingMetadata,
                         const std::vector<uint8_t>& pingData)
{
    //cout << "============= Got ping\n" << pingMetadata << endl;
    pingMetadata_ = pingMetadata;
    pingData_ = pingData;
    displayable = true;
}


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

void config_request(narval::oculus::SonarClient* sonarClient, 
                    oculus_sonar::OculusSonarConfig& config,
                    uint32_t level)
{
    std::cout << "Received config : " << level << std::endl;
    // on node launch, the configuration server asks for current configuration
    // by setting level to the maximum possible value.
    if(level == std::numeric_limits<uint32_t>::max()) {
        auto feedback = sonarClient->current_fire_config();
        config.frequency_mode   = feedback.masterMode;
        config.ping_rate        = 0;
        config.data_depth       = feedback.flags & 0x2;
        config.send_gain        = feedback.flags & 0x4;
        config.range            = feedback.range;
        config.gamma_correction = feedback.gammaCorrection;
        config.gain_percent     = feedback.gainPercent;
        config.sound_speed      = feedback.speedOfSound;
        config.salinity         = feedback.salinity;
        return;
    }

    narval::oculus::SonarClient::PingConfig currentConfig;
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

    auto feedback = sonarClient->request_fire_config(currentConfig);
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

int main(int argc, char **argv)
{
    Display display;
    auto renderer = ImageRenderer::New();
    display.add_renderer(renderer);

    //ros::init(argc, argv, "oculus_sonar");
    ros::init(argc, argv, "oculus_sonar", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;

    narval::oculus::Sonar sonarClient;
    sonarClient.start(); // better to start it here.
    while(!sonarClient.connected());
    
    // sonar status publisher
    ros::Publisher statusPublisher = node.advertise<oculus_sonar::OculusStatus>("status", 100);
    sonarClient.add_status_callback(&publish_status, statusPublisher);

    // ping publisher
    ros::Publisher pingPublisher = node.advertise<oculus_sonar::OculusPing>("ping", 100);
    sonarClient.add_ping_callback(&publish_ping, pingPublisher);

    // config server
    dynamic_reconfigure::Server<oculus_sonar::OculusSonarConfig> configServer;
    configServer.setCallback(boost::bind(&config_request, &sonarClient, _1, _2));

    sonarClient.request_fire_config(narval::oculus::default_fire_config());

    sonarClient.add_ping_callback(&update_display_data);
    while(!display.should_close()) {
        if(displayable) {
            Shape imageShape;
            if(pingMetadata_.fireMessage.flags & 0x4) {
                imageShape = Shape({pingMetadata_.nBeams + 4u, pingMetadata_.nRanges});
                //imageShape = Shape({pingMetadata_.nBeams, pingMetadata_.nRanges});
            }
            else {
                imageShape = Shape({pingMetadata_.nBeams, pingMetadata_.nRanges});
            }
            renderer->set_image(imageShape, pingData_.data() + pingMetadata_.imageOffset);
        }
        display.draw();
        ros::spinOnce();
    }
    //ros::spin();

    sonarClient.stop();

    return 0;
}
