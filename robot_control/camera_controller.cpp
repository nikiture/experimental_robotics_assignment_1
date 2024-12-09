#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/twist.hpp"
//#include "geometry_msgs/msg/pose.hpp"
//#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "targets_interface/msg/targets_yaw.hpp"
//#include "nav_msgs/msg/odometry.hpp"
//#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"

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
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;


class Camera_controller : public rclcpp::Node 
{
	public:
		
		Camera_controller () 
			: Node ("Camera_controller"), 
			forward_speed (0), 
			found_markers (false), 
			current_marker_idx (0), 
			angle_error(0)
		{
			this->declare_parameter("starting_rotation_speed", 0.1);
			max_rotation_speed = this->get_parameter("starting_rotation_speed").as_double();
			this->declare_parameter ("camera_control_topic", "joint_camera_controller/commands");
			this->declare_parameter ("control_joint_name", "camera_joint");
			this->declare_parameter ("joint_state_topic", "/dynamic_joint_states");
			this->declare_parameter ("state_control_variable", "position");
			//RCLCPP_INFO (this->get_logger(), "%lf", max_rotation_speed);

			control_joint_name = this -> get_parameter ("control_joint_name").as_string();
			/*forward_speed = 0;
			rotation_speed = 0;
			found_markers = false;
			current_marker_idx = 0;*/
			//wheel_control_publisher = this->create_publisher <geometry_msgs::msg::Twist> ("/cmd_vel", 5); 
			//not finding "operator ""ms", calling std::chrono::duration instead
			//auto interval = std::chrono::milliseconds (100);
			timer = this-> create_wall_timer (100ms, std::bind (&Camera_controller::timer_callback, this));
			//yaw_publisher = this->create_publisher <geometry_msgs::msg::Pose> ("/current_yaw", 5);
			auto control_topic = this -> get_parameter ("camera_control_topic").as_string();
			camera_joint_controller = this -> create_publisher <std_msgs::msg::Float64MultiArray> (control_topic, 5);
			//Odometry_subscription = this -> create_subscription <nav_msgs::msg::Odometry> ("/odom", 10, std::bind (&Camera_controller::odom_callback, this, _1));
			auto joint_state_topic = this -> get_parameter ("joint_state_topic").as_string();
			//Joint_yaw_subscription = this -> create_subscription <sensor_msgs::msg::JointState> (joint_state_topic, 10, std::bind (&Camera_controller::Joint_callback, this, _1));
			Joint_yaw_subscription = this -> create_subscription <control_msgs::msg::DynamicJointState> (joint_state_topic, 10, std::bind (&Camera_controller::Joint_callback, this, _1));
			marker_subscription = this -> create_subscription <targets_interface::msg::TargetsYaw> ("/sorted_markers", 10, std::bind (&Camera_controller::marker_callback, this, _1));
			marker_circle_caller = this -> create_publisher <std_msgs::msg::Empty> ("/place_circle", 5);
			yaw_publisher = this-> create_publisher <targets_interface::msg::RobotYaw> ("/current_yaw", 5);
			//RCLCPP_INFO (this->get_logger(), "rotation speed: %lf", rotation_speed);
		}
	private:
		//rclcpp::Publisher <geometry_msgs::msg::Twist>::SharedPtr wheel_control_publisher;
		rclcpp::Publisher <std_msgs::msg::Float64MultiArray>::SharedPtr camera_joint_controller;
		rclcpp::Publisher <targets_interface::msg::RobotYaw>::SharedPtr yaw_publisher;
		double forward_speed;
		double rotation_speed, max_rotation_speed;
		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Subscription <targets_interface::msg::TargetsYaw>::SharedPtr marker_subscription;
		//rclcpp::Subscription <sensor_msgs::msg::JointState>::SharedPtr Joint_yaw_subscription;
		rclcpp::Subscription <control_msgs::msg::DynamicJointState>::SharedPtr Joint_yaw_subscription;
		//rclcpp::Subscription <nav_msgs::msg::Odometry>::SharedPtr Odometry_subscription;
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr marker_circle_caller;
		//rclcpp::Client<std_srvs::srv::Empty>::SharedPtr marker_circle_caller;
		
		std::vector <double> ordered_marker_orientation;
		std::vector <long int> ordered_marker_ids;
		std::string control_joint_name;
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
		//void odom_callback (const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
		//void Joint_callback (const sensor_msgs::msg::JointState::SharedPtr joint_msg) {
		void Joint_callback (const control_msgs::msg::DynamicJointState::SharedPtr joint_msg) {
			
			//obtain yaw from orientation quaternion
			/* auto q = odom_msg->pose.pose.orientation;
			auto tmp1 = 2 * (q.w * q.z + q.x * q.y);
			auto tmp2 = 1 - 2 * (q.y * q.y + q.z * q.z);
			current_yaw = atan2 (tmp1, tmp2); */
			/* auto joint_idx = -1;
			for (auto i = 0; i < joint_msg->name.size(); i++) {
				if (joint_msg->name[i] == control_joint_name) {
					joint_idx = i;
					break;
				}
			}
			if (joint_idx >= 0) {
				current_yaw = joint_msg->position[joint_idx];
			} */
			auto control_state_interface = this -> get_parameter ("state_control_variable").as_string();
			long unsigned int name_idx = -1, interface_idx = -1; 
			/* for (long unsigned int i = 0; i < joint_msg->joint_names.size(); i++) {
				if (control_joint_name == joint_msg->joint_names[i]) {
					name_idx = i;
					break;
				}
			} */
			auto name_idx_cand = std::find (joint_msg -> joint_names.begin(), joint_msg->joint_names.end(), control_joint_name);
			if (name_idx_cand != joint_msg->joint_names.end()) {
				name_idx = std::distance(joint_msg->joint_names.begin(), name_idx_cand);
			}
			auto camera_interface_values = joint_msg->interface_values[name_idx];
			auto interface_idx_cand = std::find (camera_interface_values.interface_names.begin(), camera_interface_values.interface_names.end(), control_state_interface);
			if (interface_idx_cand != camera_interface_values.interface_names.end()) {
				interface_idx = std::distance(camera_interface_values.interface_names.begin(), interface_idx_cand);
			}
			current_yaw = camera_interface_values.values[interface_idx];

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
				//RCLCPP_INFO_THROTTLE (this->get_logger(), *this->get_clock(), 1000, "")
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
					rotation_speed *= 2;
					if (rotation_speed > max_rotation_speed) 
						rotation_speed = max_rotation_speed;
					if (rotation_speed < -max_rotation_speed) 
						rotation_speed = - max_rotation_speed;
					//RCLCPP_INFO_THROTTLE (this->get_logger(), *this->get_clock(), 500, "%lf", rotation_speed);
				}
				if (current_marker_idx >= ordered_marker_orientation.size()) {
					RCLCPP_INFO (this->get_logger(), "all markers reached\n");
					/* auto shutdown_command = geometry_msgs::msg::Twist ();
					shutdown_command.linear.x = 0;
					shutdown_command.angular.z = 0;
					wheel_control_publisher->publish (shutdown_command); */
					auto shutdown_command = std_msgs::msg::Float64MultiArray();
					shutdown_command.data.push_back(0.0);
					camera_joint_controller->publish(shutdown_command);
					//~Camera_controller.Node();
					//return 0;
					//~this;
					rclcpp::shutdown();
				}
			}
			/* auto command = geometry_msgs::msg::Twist ();
			command.linear.x = forward_speed;
			command.angular.z = rotation_speed;
			wheel_control_publisher->publish (command); */
			auto command = std_msgs::msg::Float64MultiArray();
			command.data.push_back(rotation_speed);
			camera_joint_controller->publish(command);
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
	rclcpp::spin (std::make_shared <Camera_controller> ());
	rclcpp::shutdown ();
	return 0;
}