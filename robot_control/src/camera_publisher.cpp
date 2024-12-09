#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

using namespace std::chrono_literals;

/* node used for testing the control and feedback topics for the camera joint rotation */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        this->declare_parameter("camera_rotation_speed", 0.1);
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_camera_controller/commands", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Float64MultiArray();

      auto msg_dim = std_msgs::msg::MultiArrayDimension();
      msg_dim.size = 1;
      message.layout.dim.push_back(msg_dim);

      auto des_speed = this->get_parameter("camera_rotation_speed").as_double();
      message.data.push_back(des_speed);

      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}