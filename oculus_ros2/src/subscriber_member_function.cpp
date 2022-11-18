/* 
=======================================================================
===  DEVELOPPEMENT NODE MUST BE REMOVE BEFORE DOING A PULL REQUEST  ===
===  hugo@forssea-robotics.fr                                       ===
=======================================================================
*/

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "oculus_interfaces/msg/oculus_stamped_ping.hpp"                                      // CHANGE

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<oculus_interfaces::msg::OculusStampedPing>(    // CHANGE
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const oculus_interfaces::msg::OculusStampedPing & msg) const  // CHANGE
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard");     // CHANGE
  }
  rclcpp::Subscription<oculus_interfaces::msg::OculusStampedPing>::SharedPtr subscription_;  // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}