#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"
//#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <chrono>
#include <memory>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> 

//#include "geometry_msgs/msg/twist.hpp"
//#include "geometry_msgs/msg/pose.hpp"
//#include "targets_interface/msg/targets_yaw.hpp"
//#include "nav_msgs/msg/odometry.hpp"
//#include "std_msgs/msg/empty.hpp"
//#include <chrono>
//#include <memory>

using std::placeholders::_1;

class Marker_Circler : public rclcpp::Node {
	public :
		
		Marker_Circler ()
		: Node ("marker_circler"), height(0), width(0), start(true), central_idx(-2), marker_in_image (-1)
		{			
			camera_image_sub = this ->create_subscription<sensor_msgs::msg::Image> ("/camera/image_raw", 10, std::bind (&Marker_Circler::camera_callback, this, _1));
			//Odometry_subscription = this -> create_subscription <nav_msgs::msg::Odometry> ("/odom", 10, std::bind (&Robot_controller::odom_callback, this, _1));
			//marker_central_sub = this -> create_subscription <std_msgs::msg::Empty> ("/place_circle", 10, std::bind(&Marker_Circler::marker_callback, this, _1));
			image_pub = this -> create_publisher <sensor_msgs::msg::Image> ("/images/circled_markers", 5);
			marker_central_sub = this->create_subscription<std_msgs::msg::Empty> ("/place_circle", 10, std::bind(&Marker_Circler::marker_callback, this, _1));
			//marker_location = this->create_subscription<geometry_msgs::msg::PoseArray> ("aruco_poses", 5, std::bind(&Marker_Circler::marker_pose_callback, this, _1));
			//cam_info_getter = this->create_subscription<sensor_msgs::msg::CameraInfo> ("/camera/camera_info", 1, std::bind(&Marker_Circler::cam_info_callback, this, _1));
			//RCLCPP_INFO (this->get_logger(), "camera rotation");
			//cv::Rodrigues (cv::Mat(3, 1, CV_64FC1), id_matrix);
			id_matrix = cv::Mat::eye(3,3, CV_32F);
			//RCLCPP_INFO (this->get_logger(), "camera translation");
			camera_pos = cv::Mat (3,1, CV_32F);
			//RCLCPP_INFO (this->get_logger(), "construction complete");
		}
	private:
		rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_getter;
		rclcpp::Publisher <sensor_msgs::msg::Image>::SharedPtr image_pub;
		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr marker_location;
		bool start;
		int central_idx, marker_in_image;
		sensor_msgs::msg::Image::SharedPtr image_to_pub;
		cv_bridge::CvImagePtr curr_image;
		double center_height, center_width, height, width;
		rclcpp::Subscription <sensor_msgs::msg::Image>::SharedPtr camera_image_sub;
		rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr marker_central_sub;
		cv::Mat id_matrix;
		cv::Mat cam_matrix;
		//std::vector<float> cam_dist;
		cv::Mat cam_dist;
		//std::vector<std::vector<cv::Point2f>> marker_center;
		std::vector<cv::Point2f> marker_center;
		//cv::Mat marker_center;
		cv::Mat camera_pos;
		
		
		//rclcpp::Service <std_srvs::srv::Empty>::Shared_ptr marker_circle_serv;
		void camera_callback (const sensor_msgs::msg::Image::SharedPtr cam_msg) {
			curr_image = cv_bridge::toCvCopy (cam_msg, cam_msg -> encoding);
			height = cam_msg -> height;
			width = cam_msg -> width;
			/*if(start) {
				start = false;
				image_pub->publish(*cam_msg);
				RCLCPP_INFO(this->get_logger(), "first image printed");
			}*/
		}
		void marker_callback (const std_msgs::msg::Empty::SharedPtr msg) {
			//cv_bridge::CvImagePtr process_image =  
			//RCLCPP_INFO(this->get_logger(), "printing circled marker");
			//RCLCPP_INFO(this->get_logger(), "%lf %lf", center_width, center_height);
			
			
			//add a circle at the center of the current image (there should be a marker at the center), then publish image
			
			//cv::circle(curr_image->image, cv::Point(width / 2, height / 2), 100, CV_RGB(255,0,0));
			//cv::circle(curr_image->image, cv::Point(width / 2 - center_width, height / 2 - center_height), 100, CV_RGB(255,0,0));
			
			
			//cv::circle(curr_image->image, cv::Point(center_height + height / 2, center_width + width / 2), 35, CV_RGB(255,0,0));
			
			
			//cv::circle(curr_image->image, cv::Point(marker_center.y, marker_center.x), 100, CV_RGB(255,0,0));
			//cv::circle(curr_image->image, marker_center[0], 100, CV_RGB(255,0,0));
			cv::circle(curr_image->image, cv::Point(width / 2, height / 2), 35, CV_RGB(255,0,0));
			
			curr_image->header = std_msgs::msg::Header();
			image_to_pub = curr_image->toImageMsg();
			image_pub -> publish (*image_to_pub);
			
			//image_pub -> publish (*(curr_image -> toImageMsg()));
			//cv::imshow (OPENCV_WINDOW, curr_image->image);
		}
		void marker_pose_callback (const geometry_msgs::msg::PoseArray::SharedPtr pose_msg) {
			marker_in_image = pose_msg->poses.size();
			//RCLCPP_INFO(this->get_logger(), "%d", marker_in_image);
			central_idx =  marker_in_image / 2;
			/*auto central_marker = pose_msg->poses[central_idx];
			
			//std::vector<std::vector<double>> center_pose = {{central_marker.position.x, central_marker.position.y, central_marker.position.z}};
			cv::Mat center_pose(cv::Point3f(central_marker.position.x,
												central_marker.position.y,
												central_marker.position.z));
			marker_center.clear();
			RCLCPP_INFO (this->get_logger(), "%d", cam_matrix.total());
			for (int i = 0; i < 9; i++) {
				RCLCPP_INFO (this->get_logger(), "%lf", cam_matrix.at<float>(i));
			}

			/*for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					RCLCPP_INFO (this->get_logger(), "%lf", cam_matrix.at<CV_64FC1>(i,j));
				}
			}*/
			/*
			RCLCPP_INFO_THROTTLE (this->get_logger(), *this->get_clock(), 500, "projecting position on camera plane");
			//cam_matrix.reshape(3, 3);
			cv::projectPoints (center_pose, id_matrix, camera_pos, cam_matrix, cam_dist, marker_center);
			RCLCPP_INFO_THROTTLE (this->get_logger(), *this->get_clock(), 500, "projection completed");
			*/
			/*if (central_idx >= 0) {
				center_height = -pose_msg->poses[central_idx].position.z;
				center_width = -pose_msg->poses[central_idx].position.y;
			}*/
			
			central_idx = -1;
		}
		void cam_info_callback (const sensor_msgs::msg::CameraInfo::SharedPtr info_msg) {
			RCLCPP_INFO(this->get_logger(), "parsing camera info");
			//cam_matrix = cv::Mat(info_msg->k, true);
			//cam_matrix.reshape(3, 3);

			cam_matrix = cv::Mat (3,3, CV_32F);
			for (int i = 0; i < info_msg->k.size(); i++) {
				cam_matrix.at<float>(i) = info_msg->k[i];
			}
			//cam_dist = info_msg->d;
			/*std::vector<double> tmp_vec = info_msg->d;
			cam_dist = std::vector<float> (tmp_vec.begin(), tmp_vec.end());
			*/
			//cam_dist = cv::Mat (info_msg->d);
			cam_dist = cv::Mat (1, info_msg->d.size(), CV_32F);
			for (int i = 0; i < info_msg->d.size(); i++) {
				cam_dist.at<float>(i) = info_msg->d[i];
			}
			cam_info_getter.reset();
			RCLCPP_INFO(this->get_logger(), "parsing completed");
		}
};


int main (int argc, char* argv[]) {
	rclcpp::init (argc, argv);
	rclcpp::spin (std::make_shared <Marker_Circler> ());
	rclcpp::shutdown ();
	return 0;
}
