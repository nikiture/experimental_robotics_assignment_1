#include <memory>
#include <vector>
#include <algorithm>
//#include <utilities>

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp" 
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <map>
#include <cmath>
#include <string>
#include <chrono>
#include "targets_interface/msg/targets_yaw.hpp"
#include "targets_interface/msg/robot_yaw.hpp"
/* #include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h" */


using std::placeholders::_1;


//camera joint position trackable on /joint_states for camera, robot orientation on /odom/pose

class MarkerDetector : public rclcpp::Node
{
	public:
		MarkerDetector()
			: Node("MarkerDetector"),
			full_lap (false),
			current_yaw (0), 
			stop_inputs(false),
			received_camera_frame(false)
		{
			this->declare_parameter("starting_rotation_speed", 0.1);
			
			auto rotation_speed = this->get_parameter("starting_rotation_speed").as_double();
			RCLCPP_INFO (this->get_logger(), "%lf", rotation_speed);
			auto interval = std::chrono::milliseconds (static_cast<long int> (ceil(1000 * 3 * (6.28 / rotation_speed))));
			lap_timer = this-> create_wall_timer (interval, std::bind (&MarkerDetector::full_lap_callback, this));
			
			
			aruco_marker_subscription = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
			"/aruco_markers", 10, std::bind(&MarkerDetector::aruco_callback, this, _1));
			//yaw_update = this->create_subscription <geometry_msgs::msg::Pose> ("/current_yaw", 10, std::bind (&MarkerDetector::yaw_callback, this, _1));
			/* odom_sub = this -> create_subscription <nav_msgs::msg::Odometry> (
				"/odom", 2, std::bind (&MarkerDetector::odom_callback, this, _1)); */
			total_marker_publisher = this -> create_publisher <targets_interface::msg::TargetsYaw> ("/sorted_markers", 10);
			current_yaw_sub = this -> create_subscription<targets_interface::msg::RobotYaw> (
				"/current_yaw", 5, std::bind(&MarkerDetector::yaw_callback, this, _1));
			
			/* tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
			auto tf_interval = std::chrono::milliseconds (1); */
			//tf_timer = this->create_wall_timer(tf_interval, std::bind(&MarkerDetector::tf_callback, this));
			//camera_frame = this->declare_parameter<std::string>("target_frame", "camera_link");
			//robot_frame = this->declare_parameter<std::string>("robot_frame", "link_chassis");
		
		
		}

	private:
		void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
			if (stop_inputs) return;
			std::vector <long int> marker_ids = msg -> marker_ids;
			markers_yaw.clear();
			//RCLCPP_INFO (this->get_logger(), "currently found %d markers in camera: \n", marker_ids.size());
			//std::vector <geometry_msgs::msg::Pose> marker_pose = msg -> poses;*/
			for (long unsigned int i = 0; i < marker_ids.size(); i++) {
				/*if (std::ptrdiff_t (found_markers.find (*i), found_markers.end())  {
					marker_pose [*i].orientation.w += current_yaw;
					found_markers.emplace (*i, marker_pose [*i]);
				}*/

				// yaw from difference in position + current yaw
				/* auto mark_pose = msg->poses[i].position;
				auto diff = geometry_msgs::msg::Point();
				diff.x = mark_pose.x - curr_pos.x - camera_pos.x;
				diff.y = mark_pose.y - curr_pos.y - camera_pos.y;
				diff.z = mark_pose.z - curr_pos.z - camera_pos.z;
				markers_yaw.push_back(atan2(diff.y, diff.x) + current_yaw);*/
				
				
				
				
				
				
				//obtain yaw from orientation quaternion
				//are markers' poses'yaw along zaxis or y axis?
				/* t2 = +2.0 * (w * y - z * x)
				t2 = +1.0 if t2 > +1.0 else t2
				t2 = -1.0 if t2 < -1.0 else t2
				pitch_y = math.asin(t2)  */

				/*auto q = msg->poses[i].orientation;
				auto tmp1 = 2.0 * (q.w * q.y - q.z * q.x);
				tmp1 = std::min (1.0, tmp1);
				tmp1 = std::max (-1.0, tmp1);
				markers_yaw.push_back (asin(tmp1) + current_yaw);*/
				/* auto tmp1 = 2 * (-q.w * q.z + q.x * q.y);
				auto tmp2 = 1 - 2 * (q.y * q.y + q.z * q.z);
				markers_yaw.push_back (atan2 (tmp1, tmp2) + current_yaw); */
				//marker_pose [*i].orientation.w += current_yaw;
				//double marker_yaw = msg -> poses [*i].orientation.w + current_yaw;
				//double marker_yaw = - msg -> poses[i].orientation.w + current_yaw;
				//correction for angles in [-pi, pi] range
				/* if (markers_yaw[i] > 3.14)
					markers_yaw[i] -= 6.28;
				if (markers_yaw[i] < -3.14)
					markers_yaw[i] += 6.28; */
				
				
				//found_markers.emplace(std::make_pair(marker_ids[i], marker_yaw));
				//found_markers.emplace(marker_ids[i], markers_yaw[i]);
				found_markers.emplace(marker_ids[i], current_yaw);
				//found_markers.insert(std::make_pair<
				//RCLCPP_INFO (this->get_logger(), "%d , %lf\n", marker_ids[i], marker_yaw);
			}
			if  (full_lap) {
				//publish current list (map) of markers and unsubscribe from yaw updating topic
				//yaw_update.reset();
				RCLCPP_INFO (this->get_logger(), "all markers localized, sending to controller\n");
				targets_interface::msg::TargetsYaw total_markers;
				for (auto i = found_markers.begin(); i != found_markers.end(); i++) {
					total_markers.targets_yaws.push_back (i -> second);
				}
				/*for (auto i = 0; i < total_markers.targets_yaws.size(); i++) {
					RCLCPP_INFO (this->get_logger(), "%lf\n", total_markers.targets_yaws[i]);
				}*/
				//targets_interface::msg::TargetsYaw total_markers = markers_yaw;
				total_marker_publisher -> publish (total_markers);
				stop_inputs = true;
				rclcpp::sleep_for (std::chrono::milliseconds(5));
				rclcpp::shutdown();
			}
		}
		void yaw_callback (const targets_interface::msg::RobotYaw::SharedPtr yaw_msg) {
			/* curr_pos = odom_msg->pose.pose.position;
			if (stop_inputs||full_lap) return;
			//RCLCPP_INFO (this->get_logger(), "obtaining new yaw");
			auto q = odom_msg->pose.pose.orientation;
			auto tmp1 = 2 * (q.w * q.z + q.x * q.y);
			auto tmp2 = 1 - 2 * (q.y * q.y + q.z * q.z);
			double new_yaw = atan2 (tmp1, tmp2); */
			/* full_lap = (current_yaw > 0 && new_yaw < 0);
			RCLCPP_INFO (this->get_logger(), "%lf %lf", current_yaw, new_yaw); */
			current_yaw = yaw_msg->yaw;
			
		}
		void full_lap_callback () {
			full_lap = true;
			lap_timer->cancel();
		}
		/* void tf_callback () {
			geometry_msgs::msg::Vector3 t;
			try {
				t = tf_buffer_->lookupTransform(camera_frame, robot_frame, tf2::TimePointZero).transform.translation;
			} catch (const tf2::TransformException& exc) {
				RCLCPP_INFO (this->get_logger(), "camera transform not yet available");
				return;
			}
			//camera_pos = t.transform.translation;
			camera_pos.x = t.x;
			camera_pos.y = t.y;
			camera_pos.z = t.z;
			received_camera_frame = true;
			tf_timer->cancel();
		} */
		std::string camera_frame, robot_frame;
		rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_marker_subscription;
		//std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
		//std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		//rclcpp::Subscription <geometry_msgs::msg::Pose>::SharedPtr yaw_update;
		
		//rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
		rclcpp::Subscription<targets_interface::msg::RobotYaw>::SharedPtr current_yaw_sub;
		rclcpp::Publisher <targets_interface::msg::TargetsYaw>::SharedPtr total_marker_publisher;
		bool full_lap;
		bool received_camera_frame;
		rclcpp::TimerBase::SharedPtr lap_timer, tf_timer;
		std::map <long int, double> found_markers;
		std::vector<double> markers_yaw;
		double current_yaw;
		geometry_msgs::msg::Point curr_pos, camera_pos;
		bool stop_inputs;
};

int main (int argc, char* argv []) {
	rclcpp::init (argc, argv);
	MarkerDetector::SharedPtr aruco_printer = std::make_shared <MarkerDetector> ();
	rclcpp::spin (aruco_printer);
	rclcpp::shutdown();
	return 0;
}
