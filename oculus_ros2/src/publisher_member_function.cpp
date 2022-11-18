

/* 
=======================================================================
===  DEVELOPPEMENT NODE MUST BE REMOVE BEFORE DOING A PULL REQUEST  ===
===  hugo@forssea-robotics.fr                                       ===
=======================================================================
*/

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "oculus_interfaces/msg/oculus_stamped_ping.hpp"                                         // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<oculus_interfaces::msg::OculusStampedPing>("topic", 10);  // CHANGE
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = oculus_interfaces::msg::OculusStampedPing();                                   // CHANGE
    // message.OculusStampedPing = this->count_++;                                                     
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing");    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<oculus_interfaces::msg::OculusStampedPing>::SharedPtr publisher_;             // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
