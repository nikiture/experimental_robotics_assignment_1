#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "targets_interface/msg/targets_yaw.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "targets_interface/msg/robot_yaw.hpp"

#include <cmath>
#include <chrono>
#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;


/* Node tasked to move the robot on its vertical axis, 
send its yaw to the marker detection and sorting node
and signal the marker circler node when to draw a circle around the markers */

class Robot_controller : public rclcpp::Node 
{
	public:
		
		Robot_controller () 
			: Node ("Robot_controller"), 
			found_markers (false), 
			current_marker_idx (0), 
			angle_error(0)
		{
			this->declare_parameter("starting_rotation_speed", 0.1);
			max_rotation_speed = this->get_parameter("starting_rotation_speed").as_double();

			wheel_control_publisher = this->create_publisher <geometry_msgs::msg::Twist> ("/cmd_vel", 5); 

			timer = this-> create_wall_timer (100ms, std::bind (&Robot_controller::timer_callback, this));
			
			Odometry_subscription = this -> create_subscription <nav_msgs::msg::Odometry> ("/odom", 10, std::bind (&Robot_controller::odom_callback, this, _1));
			
			marker_subscription = this -> create_subscription <targets_interface::msg::TargetsYaw> ("/sorted_markers", 10, std::bind (&Robot_controller::marker_callback, this, _1));
			
			marker_circle_caller = this -> create_publisher <std_msgs::msg::Empty> ("/place_circle", 5);
			
			yaw_publisher = this-> create_publisher <targets_interface::msg::RobotYaw> ("/current_yaw", 5);
		}
	private:
		rclcpp::Publisher <geometry_msgs::msg::Twist>::SharedPtr wheel_control_publisher;

		rclcpp::Publisher <targets_interface::msg::RobotYaw>::SharedPtr yaw_publisher;

		rclcpp::TimerBase::SharedPtr timer;
		
		rclcpp::Subscription <targets_interface::msg::TargetsYaw>::SharedPtr marker_subscription;
		
		rclcpp::Subscription <nav_msgs::msg::Odometry>::SharedPtr Odometry_subscription;
		
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr marker_circle_caller;

		std::vector <double> ordered_marker_orientation;
		std::vector <long int> ordered_marker_ids;
		
		double rotation_speed, max_rotation_speed;
		double current_yaw;
		bool found_markers;
		long unsigned int current_marker_idx;
		double angle_error; 
		
		void marker_callback (const targets_interface::msg::TargetsYaw::SharedPtr marker_msg) {
			//list of markers sorted by IDs received by marker detector node, transition from simple lap phase to marker reaching phase
			RCLCPP_INFO (this->get_logger(), "received marker list of %d elements", marker_msg -> targets_yaws.size());
			ordered_marker_orientation = marker_msg -> targets_yaws;
			found_markers = true;
		}
		void odom_callback (const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
			//obtain yaw from orientation quaternion
			auto q = odom_msg->pose.pose.orientation;
			auto tmp1 = 2 * (q.w * q.z + q.x * q.y);
			auto tmp2 = 1 - 2 * (q.y * q.y + q.z * q.z);
			current_yaw = atan2 (tmp1, tmp2);

			//transmit current yaw to the marker detection node
			auto yaw_msg = targets_interface::msg::RobotYaw();
			yaw_msg.yaw = current_yaw;
			yaw_publisher -> publish(yaw_msg);
		}	
		void timer_callback () {
			//main control loop, camera motion and circle drawing messages published from here
			rotation_speed = max_rotation_speed;
			if (found_markers) { //need to go to yaw of current marker to reach 
				angle_error = current_yaw - ordered_marker_orientation[current_marker_idx];

				if (angle_error > 3.14)
		  			angle_error -= 6.28;
		  		if (angle_error < -3.14)
		  			angle_error += 6.28;
				
				if (angle_error < 0.01 && angle_error > -0.01) {
					RCLCPP_INFO (this->get_logger(), "reached marker %d\n", current_marker_idx);
					//stop to give time to the circler node to draw the circle before the marker exits the camera's view
					rotation_speed = 0;
					//set target marker to next marker in sorted list
					current_marker_idx ++;
					//publish message to node drawing circle around marker
					marker_circle_caller -> publish (std_msgs::msg::Empty());
					
				} else {
					rotation_speed = - angle_error;
					rotation_speed *= 10;

					//saturate speed to avoid that the robots flips
					if (rotation_speed > max_rotation_speed) 
						rotation_speed = max_rotation_speed;
					if (rotation_speed < -max_rotation_speed) 
						rotation_speed = - max_rotation_speed;
				}
				if (current_marker_idx >= ordered_marker_orientation.size()) {
					//all markers reached, stopping robot and exitig
					RCLCPP_INFO (this->get_logger(), "all markers reached\n");
					auto shutdown_command = geometry_msgs::msg::Twist ();
					shutdown_command.linear.x = 0;
					shutdown_command.angular.z = 0;
					wheel_control_publisher->publish (shutdown_command);
					rclcpp::shutdown();
				}
			}
			//publish desired speed to controller
			auto command = geometry_msgs::msg::Twist ();
			command.linear.x = 0.0;
			command.angular.z = rotation_speed;
			wheel_control_publisher->publish (command);
		}
};


int main (int argc, char * argv []) {
	rclcpp::init (argc, argv);
	rclcpp::spin (std::make_shared <Robot_controller> ());
	rclcpp::shutdown ();
	return 0;
}
