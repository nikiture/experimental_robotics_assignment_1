#include "rclcpp/rclcpp.hpp"
#include "targets_interface/msg/targets_yaw.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "targets_interface/msg/robot_yaw.hpp"

#include <cmath>
#include <chrono>
#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;



/* Node tasked to move the camera's joint, 
send its configuration value (the "yaw of the camera") to the marker detection and sorting node
and signal the marker circler node when to draw a circle around the markers */


class Camera_controller : public rclcpp::Node 
{
	public:
		
		Camera_controller () 
			: Node ("Camera_controller"), 
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

			control_joint_name = this -> get_parameter ("control_joint_name").as_string();

			timer = this-> create_wall_timer (100ms, std::bind (&Camera_controller::timer_callback, this));

			auto control_topic = this -> get_parameter ("camera_control_topic").as_string();
			camera_joint_controller = this -> create_publisher <std_msgs::msg::Float64MultiArray> (control_topic, 5);
			
			auto joint_state_topic = this -> get_parameter ("joint_state_topic").as_string();

			Joint_yaw_subscription = this -> create_subscription <control_msgs::msg::DynamicJointState> (joint_state_topic, 10, std::bind (&Camera_controller::Joint_callback, this, _1));
			
			marker_subscription = this -> create_subscription <targets_interface::msg::TargetsYaw> ("/sorted_markers", 10, std::bind (&Camera_controller::marker_callback, this, _1));
			
			marker_circle_caller = this -> create_publisher <std_msgs::msg::Empty> ("/place_circle", 5);
			
			yaw_publisher = this-> create_publisher <targets_interface::msg::RobotYaw> ("/current_yaw", 5);
		}
	private:
		rclcpp::Publisher <std_msgs::msg::Float64MultiArray>::SharedPtr camera_joint_controller;

		rclcpp::Publisher <targets_interface::msg::RobotYaw>::SharedPtr yaw_publisher;
		
		rclcpp::TimerBase::SharedPtr timer;
		
		rclcpp::Subscription <targets_interface::msg::TargetsYaw>::SharedPtr marker_subscription;
		
		rclcpp::Subscription <control_msgs::msg::DynamicJointState>::SharedPtr Joint_yaw_subscription;
		
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr marker_circle_caller;
		
		std::string control_joint_name;

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
		void Joint_callback (const control_msgs::msg::DynamicJointState::SharedPtr joint_msg) {
			//get "camera yaw" from joint broadcaster, by finding the corresponding cotrolled joint (camera_joint) and reference variable (position) 
			auto control_state_interface = this -> get_parameter ("state_control_variable").as_string();
			
			long unsigned int name_idx = -1, interface_idx = -1; 

			//find control joint index
			auto name_idx_cand = std::find (joint_msg -> joint_names.begin(), joint_msg->joint_names.end(), control_joint_name);
			if (name_idx_cand != joint_msg->joint_names.end()) {
				name_idx = std::distance(joint_msg->joint_names.begin(), name_idx_cand);
			}

			//find controlled variable index
			auto camera_interface_values = joint_msg->interface_values[name_idx];
			auto interface_idx_cand = std::find (camera_interface_values.interface_names.begin(), camera_interface_values.interface_names.end(), control_state_interface);
			if (interface_idx_cand != camera_interface_values.interface_names.end()) {
				interface_idx = std::distance(camera_interface_values.interface_names.begin(), interface_idx_cand);
			}

			current_yaw = camera_interface_values.values[interface_idx];

			//transmit obtained "yaw" to the marker detection node
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
					rotation_speed *= 2;
					//saturate speed to avoid that the robots flips
					if (rotation_speed > max_rotation_speed) 
						rotation_speed = max_rotation_speed;
					if (rotation_speed < -max_rotation_speed) 
						rotation_speed = - max_rotation_speed;
				}
				if (current_marker_idx >= ordered_marker_orientation.size()) {
					//all markers reached, stopping robot and exitig
					RCLCPP_INFO (this->get_logger(), "all markers reached\n");
					auto shutdown_command = std_msgs::msg::Float64MultiArray();
					shutdown_command.data.push_back(0.0);
					camera_joint_controller->publish(shutdown_command);
					rclcpp::shutdown();
				}
			}
			//publish desired speed to controller
			auto command = std_msgs::msg::Float64MultiArray();
			command.data.push_back(rotation_speed);
			camera_joint_controller->publish(command);
		}
};


int main (int argc, char * argv []) {
	rclcpp::init (argc, argv);
	rclcpp::spin (std::make_shared <Camera_controller> ());
	rclcpp::shutdown ();
	return 0;
}