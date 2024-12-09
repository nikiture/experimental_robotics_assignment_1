#include "std_msgs/msg/empty.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

using std::placeholders::_1;

class Minimal_subscriber : public rclcpp::Node
{
public:
  Minimal_subscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Empty>("/place_circle", 10, std::bind(&Minimal_subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Empty::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Signal received!\n");
  }
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
};


int main (int argc, char* argv[]) {
	rclcpp::init (argc, argv);
	rclcpp::spin (std::make_shared <Minimal_subscriber> ());
	rclcpp::shutdown ();
	return 0;
}