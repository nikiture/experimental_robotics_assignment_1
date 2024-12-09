#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
//#include "targets_interface/msg/targets_yaw.hpp"
//#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <memory>

using std::placeholders::_1;
class Simple_robot_controller : public rclcpp::Node {
	public:
		Simple_robot_controller (double x = 0, double w = 0) 
		: Node ("Robot_controller"), forward_speed (x), rotation_speed (w)
		{
			wheel_control_publisher = this->create_publisher <geometry_msgs::msg::Twist> ("/cmd_vel", 5); 
			auto interval = std::chrono::milliseconds (500);
			timer = this-> create_wall_timer (interval, std::bind (&Simple_robot_controller::timer_callback, this));
			rclcpp::TimerBase::SharedPtr timer;
		}
	private:
		rclcpp::Publisher <geometry_msgs::msg::Twist>::SharedPtr wheel_control_publisher;
		double forward_speed, rotation_speed;
		rclcpp::TimerBase::SharedPtr timer;
		void timer_callback () {
			auto command = geometry_msgs::msg::Twist ();
			command.linear.x = forward_speed;
			command.angular.z = rotation_speed;
			wheel_control_publisher->publish (command); 
		}
		
};
		
int main (int argc, char * argv []) {
	rclcpp::init (argc, argv);
	double x = 0, w= 0;
	if (argc > 1) {
		x = std::stod (argv [1]);
		w = std::stod (argv [2]);
	}
	rclcpp::spin (std::make_shared <Simple_robot_controller> (x, w));
	rclcpp::shutdown ();
	return 0;
}		
