#include <iostream>
using namespace std;

#include <rtac_display/Display.h>
using namespace rtac::display;

#include <ros/ros.h>

#include <oculus_sonar/OculusPing.h>

#include "PingRenderer.h"

PingRenderer::Ptr pingRenderer;

void ping_callback(const oculus_sonar::OculusPing& ping)
{
    pingRenderer->set_ping_data(ping);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oculus_display", ros::init_options::NoSigintHandler);
    std::string nodeName = ros::this_node::getName();
    if(nodeName[0] == '/') {
        nodeName = nodeName.substr(1, nodeName.size() - 1);
    }
    ros::NodeHandle node(nodeName);

    std::string pingTopic;
    node.param<std::string>("ping_topic", pingTopic, "/oculus_sonar/ping");
    cout <<  pingTopic << endl;

    Display display;
    pingRenderer = display.create_renderer<PingRenderer>(View::New());
    pingRenderer->set_direction(rtac::display::FanRenderer::Direction::Down);

    ros::Subscriber sub = node.subscribe(pingTopic, 100, ping_callback);

    while(!display.should_close()) {
        ros::spinOnce();
        display.draw();
    }

    return 0;
}
