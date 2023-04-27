
#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include "oculus_viewer_node.hpp"

using SonarDriver = oculus::SonarDriver;

OculusViewerNode::OculusViewerNode() : Node("oculus_viewer"),
                                       sonar_viewer(static_cast<rclcpp::Node *>(this))
{
    ping_subscription_ = this->create_subscription<oculus_interfaces::msg::Ping>(
        "ping", 10, std::bind(&OculusViewerNode::ping_callback, this, std::placeholders::_1));
}

OculusViewerNode::~OculusViewerNode()
{
}

void OculusViewerNode::ping_callback(const oculus_interfaces::msg::Ping &ping_msg) const
{
     // RCLCPP_INFO(get_logger(), "I am in callback");
    sonar_viewer.publish_fan(ping_msg);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OculusViewerNode>());
    rclcpp::shutdown();
    return 0;
}
