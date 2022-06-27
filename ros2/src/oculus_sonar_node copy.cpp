using namespace std;

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <include/conversions.h>
#include <include/oculus_sonar_node.hpp>

using SonarDriver = narval::oculus::SonarDriver;

OculusSonarNode::OculusSonarNode() : Node('oculus_sonar')
{
    if (!this->has_parameter("ping_topic")) {
        this->declare_parameter<string>("ping_topic", "ping");
    }
    if (!this->has_parameter("status_topic")) {
        this->declare_parameter<string>("status_topic", "ping");
    }
    std::string pingTopic, statusTopic;
    this->get_parameter("ping_topic", pingTopic);
    this->get_parameter("status_topic", statusTopic);
    
    this->ping_publisher_ = this->create_publisher<oculus_sonar::msg::OculusStampedPing>(pingTopic, 100);
    this->status_publisher_ = this->create_publisher<oculus_sonar::msg::OculusStatus>(statusTopic, 100);

    this->sonar_driver_ = SonarDriver(this->io_service_->io_service());
    this->io_service_->start();
    if(!this->sonar_driver_.wait_next_message()) {
        std::cerr << "Timeout reached while waiting for a connection to the Oculus sonar. "
                  << "Is it properly connected ?" << std::endl;
    }
    this->sonar_driver_->add_status_callback(&publish_status, this->status_publisher_);
    this->sonar_driver_->add_ping_callback(&publish_ping, this->sonar_driver_, this->ping_publisher_);
    // callback on dummy messages to reactivate the pings as needed
    this->sonar_driver_->add_dummy_callback(&handle_dummy, this->sonar_driver_, this->ping_publisher_);
}

OculusSonarNode::~OculusSonarNode()
{
    this->io_service_->stop();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OculusSonarNode>());
    rclcpp::shutdown();
    return 0;
}