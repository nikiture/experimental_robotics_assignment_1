
#include "rclcpp/rclcpp.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "targets_interface/msg/targets_yaw.hpp"
#include "targets_interface/msg/robot_yaw.hpp"
#include <memory>
#include <vector>
#include <algorithm>
#include <map>
#include <cmath>
#include <string>
#include <chrono>


using std::placeholders::_1;


/* Node tasked to store the robot's "yaw" (depending on control node either the robot yaw or the camera joint rotation angle) 
at which each detected marker is first seen sorted based on the markers' Ids */

class MarkerDetector : public rclcpp::Node
{
	public:
		MarkerDetector()
			: Node("MarkerDetector"),
			full_lap (false),
			current_yaw (0), 
			stop_inputs(false)
		{
			this->declare_parameter("starting_rotation_speed", 0.1);
			
			auto rotation_speed = this->get_parameter("starting_rotation_speed").as_double();

			//setting up timer for achieving full lap with given starting speed (modifiable parameter upon launch, assumed constant until lap completed)
			auto interval = std::chrono::milliseconds (static_cast<long int> (ceil(1000 * 3 * (6.28 / rotation_speed))));
			lap_timer = this-> create_wall_timer (interval, std::bind (&MarkerDetector::full_lap_callback, this));
			
			
			aruco_marker_subscription = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
				"/aruco_markers", 10, std::bind(&MarkerDetector::aruco_callback, this, _1));

			total_marker_publisher = this -> create_publisher <targets_interface::msg::TargetsYaw> ("/sorted_markers", 10);
			
			current_yaw_sub = this -> create_subscription<targets_interface::msg::RobotYaw> (
				"/current_yaw", 5, std::bind(&MarkerDetector::yaw_callback, this, _1));
		}

	private:
		rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_marker_subscription;
		
		rclcpp::Subscription<targets_interface::msg::RobotYaw>::SharedPtr current_yaw_sub;
		
		rclcpp::Publisher <targets_interface::msg::TargetsYaw>::SharedPtr total_marker_publisher;
		
		rclcpp::TimerBase::SharedPtr lap_timer;
		
		std::map <long int, double> found_markers;
		
		bool full_lap;
		double current_yaw;
		bool stop_inputs;
		
		void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
			//if not yet achieved full lap keep finding and sorting markers
			if (stop_inputs) return;
			std::vector <long int> marker_ids = msg -> marker_ids;
			
			for (long unsigned int i = 0; i < marker_ids.size(); i++) {
				//place marker IDs and yaw of first discovery in map; IDs used as kay for the sorting
				found_markers.emplace(marker_ids[i], current_yaw);
			}
			if  (full_lap) {
				//publish current list (map) of markers' yaw location and shutdown this node
				RCLCPP_INFO (this->get_logger(), "all markers localized, sending to controller\n");
				targets_interface::msg::TargetsYaw total_markers;
				for (auto i = found_markers.begin(); i != found_markers.end(); i++) {
					total_markers.targets_yaws.push_back (i -> second);
				}
				total_marker_publisher -> publish (total_markers);
				
				stop_inputs = true;
				//wait a bit before shutting down to ensure publishment of markers
				rclcpp::sleep_for (std::chrono::milliseconds(5));
				rclcpp::shutdown();
			}
		}
		void yaw_callback (const targets_interface::msg::RobotYaw::SharedPtr yaw_msg) {
			//receive yaw (either real robot yaw or camera joint configuration)
			current_yaw = yaw_msg->yaw;
		}
		void full_lap_callback () {
			//achieved full lap, therefore all markers are to be sent to the control node
			full_lap = true;
			lap_timer->cancel();
		}		
};

int main (int argc, char* argv []) {
	rclcpp::init (argc, argv);
	MarkerDetector::SharedPtr aruco_sorter = std::make_shared <MarkerDetector> ();
	rclcpp::spin (aruco_sorter);
	rclcpp::shutdown();
	return 0;
}
