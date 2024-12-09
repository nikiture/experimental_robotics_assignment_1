#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
//#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "targets_interface/msg/targets_yaw.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/empty.hpp"
#include "targets_interface/msg/robot_yaw.hpp"
//#include "tf2/LinearMath/Quaternion.h"
//#include "tf2/convert.h"
//#include "tf2/impl.h"
//#include "tf2/tf2.h"
//#include <Quaternion.h>
//#include "tf2/LineMath/Matrix3x3.h"
#include <cmath>
#include <chrono>
#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Robot_controller : public rclcpp::Node 
{
	public:
		
		Robot_controller () 
			: Node ("Robot_controller"), 
			forward_speed (0), 
			found_markers (false), 
			current_marker_idx (0), 
			angle_error(0)
		{
			this->declare_parameter("starting_rotation_speed", 0.0);
			max_rotation_speed = this->get_parameter("starting_rotation_speed").as_double();
			RCLCPP_INFO (this->get_logger(), "%lf", max_rotation_speed);
			/*forward_speed = 0;
			rotation_speed = 0;
			found_markers = false;
			current_marker_idx = 0;*/
			wheel_control_publisher = this->create_publisher <geometry_msgs::msg::Twist> ("/cmd_vel", 5); 
			//not finding "operator ""ms", calling std::chrono::duration instead
			//auto interval = std::chrono::milliseconds (100);
			timer = this-> create_wall_timer (100ms, std::bind (&Robot_controller::timer_callback, this));
			//yaw_publisher = this->create_publisher <geometry_msgs::msg::Pose> ("/current_yaw", 5);
			Odometry_subscription = this -> create_subscription <nav_msgs::msg::Odometry> ("/odom", 10, std::bind (&Robot_controller::odom_callback, this, _1));
			marker_subscription = this -> create_subscription <targets_interface::msg::TargetsYaw> ("/sorted_markers", 10, std::bind (&Robot_controller::marker_callback, this, _1));
			marker_circle_caller = this -> create_publisher <std_msgs::msg::Empty> ("/place_circle", 5);
			yaw_publisher = this-> create_publisher <targets_interface::msg::RobotYaw> ("/current_yaw", 5);
			//RCLCPP_INFO (this->get_logger(), "rotation speed: %lf", rotation_speed);
		}
	private:
		rclcpp::Publisher <geometry_msgs::msg::Twist>::SharedPtr wheel_control_publisher;
		rclcpp::Publisher <targets_interface::msg::RobotYaw>::SharedPtr yaw_publisher;
		double forward_speed;
		double rotation_speed, max_rotation_speed;
		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Subscription <targets_interface::msg::TargetsYaw>::SharedPtr marker_subscription;
		rclcpp::Subscription <nav_msgs::msg::Odometry>::SharedPtr Odometry_subscription;
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr marker_circle_caller;
		//rclcpp::Client<std_srvs::srv::Empty>::SharedPtr marker_circle_caller;
		
		std::vector <double> ordered_marker_orientation;
		std::vector <long int> ordered_marker_ids;
		double current_yaw;
		bool found_markers;
		long unsigned int current_marker_idx;
		double angle_error; 
		
		void marker_callback (const targets_interface::msg::TargetsYaw::SharedPtr marker_msg) {
			RCLCPP_INFO (this->get_logger(), "received marker list of %d elements", marker_msg -> targets_yaws.size());
			ordered_marker_orientation = marker_msg -> targets_yaws;
			/*for (auto i = 0; i < ordered_marker_orientation.size(); i++) {
	  			RCLCPP_INFO (this->get_logger(), "%lf\n", ordered_marker_orientation [i]);
	  		}*/
			found_markers = true;
		}
		void odom_callback (const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
			//obtain yaw from orientation quaternion
			auto q = odom_msg->pose.pose.orientation;
			auto tmp1 = 2 * (q.w * q.z + q.x * q.y);
			auto tmp2 = 1 - 2 * (q.y * q.y + q.z * q.z);
			current_yaw = atan2 (tmp1, tmp2);
			auto yaw_msg = targets_interface::msg::RobotYaw();
			yaw_msg.yaw = current_yaw;
			yaw_publisher -> publish(yaw_msg);
			/* double tmp1, tmp2;
			tf2::impl::getEulerYPR (odom_msg->pose.pose.orientation, current_yaw, tmp1, tmp2);
			 */
			//tf2::impl::getYaw (odom_msg->pose.pose.orientation, current_yaw);
			
			
			
			//current_yaw = odom_msg -> pose.pose.orientation.w;
			//RCLCPP_INFO (this->get_logger(), "%lf", odom_msg->pose.pose.orientation.z);
		}	
		void timer_callback () {
			rotation_speed = max_rotation_speed;
			if (found_markers) {
				angle_error = current_yaw - ordered_marker_orientation[current_marker_idx];
				//angle_error = current_yaw + ordered_marker_orientation[current_marker_idx];
				if (angle_error > 3.14)
		  			angle_error -= 6.28;
		  		if (angle_error < -3.14)
		  			angle_error += 6.28;
				
				//RCLCPP_INFO (this->get_logger(), "%lf %lf", current_yaw, ordered_marker_orientation[current_marker_idx]);
				/*if (angle_error > 0.1)
					rotation_speed = -0.2;
				else if (angle_error < 0.1)
					rotation_speed = 0.2;*/
				if (angle_error < 0.01 && angle_error > -0.01) {
					RCLCPP_INFO (this->get_logger(), "reached marker %d\n", current_marker_idx);
					rotation_speed = 0;
					current_marker_idx ++;
					//publish message to node putting circle around marker
					marker_circle_caller -> publish (std_msgs::msg::Empty());
					
				} else {
					/* rotation_speed = 0.2;
					if (angle_error < 0.5 && angle_error > -0.5)
						rotation_speed = 0.05;
					if (angle_error > 0.01) rotation_speed *= -1; */
					rotation_speed = - angle_error;
					rotation_speed *= 10;
					if (rotation_speed > max_rotation_speed) 
						rotation_speed = max_rotation_speed;
					if (rotation_speed < -max_rotation_speed) 
						rotation_speed = - max_rotation_speed;
					//RCLCPP_INFO_THROTTLE (this->get_logger(), *this->get_clock(), 500, "%lf", rotation_speed);
				}
				if (current_marker_idx >= ordered_marker_orientation.size()) {
					RCLCPP_INFO (this->get_logger(), "all markers reached\n");
					auto shutdown_command = geometry_msgs::msg::Twist ();
					shutdown_command.linear.x = 0;
					shutdown_command.angular.z = 0;
					wheel_control_publisher->publish (shutdown_command);
					//~Robot_controller.Node();
					//return 0;
					//~this;
					rclcpp::shutdown();
				}
			}
			auto command = geometry_msgs::msg::Twist ();
			command.linear.x = forward_speed;
			command.angular.z = rotation_speed;
			wheel_control_publisher->publish (command);
			/* auto current_pose = geometry_msgs::msg::Pose ();
			current_pose .orientation.z = 1;
			current_pose.orientation.w = current_yaw;
			yaw_publisher -> publish (current_pose);  */
		}
};


int main (int argc, char * argv []) {
	rclcpp::init (argc, argv);
	/*double x = 0, w= 0;
	if (argc > 1) {
		x = std::stod (argv [1]);
		w = std::stod (argv [2]);
	}*/
	rclcpp::spin (std::make_shared <Robot_controller> ());
	rclcpp::shutdown ();
	return 0;
}
