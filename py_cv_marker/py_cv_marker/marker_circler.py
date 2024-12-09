#!/usr/bin/python3
import rclpy
import rclpy.node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Empty, Header
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from ros2_aruco import transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import floor

class MarkerCircler (rclpy.node.Node):
    
    def __init__(self):
        super().__init__("marker_circler")
        
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", None)
        self.declare_parameter("circle_signal", "/place_circle")
        self.declare_parameter("marker_size", .0625)
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL")
        #seòf,declare_parameter("")
        

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))
    
        #image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        #info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        #self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        circle_topic = self.get_parameter("circle_signal").get_parameter_value().string_value


        self.info_sub = self.create_subscription(CameraInfo, 
                                                 info_topic,
                                                 self.info_callback,
                                                 qos_profile_sensor_data)

        self.create_subscription(Image, 
                                 image_topic,
                                 self.image_callback,
                                 qos_profile_sensor_data)

        self.create_subscription(Empty,
                                 circle_topic, 
                                 self.circle_callback,
                                 1)
        """ self.create_subscription (PoseArray,
                                  'aruco_poses',
                                  self.marker_pose_callback,
                                  5) """
        
        self.circled_pub = self.create_publisher(Image, 
                                                 "/images/circled_markers",
                                                 10)


        ##
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.curr_image = None
        self.image_to_pub = None
        self.image_encoding = None
        self.id_matrix = None
        #cv2.Rodrigues([0, 0, 0], self.id_matrix)
        #self.id_matrix = cv2.mat.eye(3)
        self.id_matrix = np.identity(3)
        self.marker_point = None
        self.color_array = np.array([0, 0, 0], np.uint8)
        #self.color_array =[255, 0, 0]
        #self.cv_color_array = np.array([0], np.uint8)
        #self.cv_color_array = None
        #self.cv_color_array = np.array([0, 0, 0], np.uint8)
        self.cv_color_array = 1
        cv2.cvtColor(self.color_array, self.cv_color_array, cv2.COLOR_RGB2BGR)
        #self.cv_color_array = tuple(np.array([255, 0, 0], np.uint8))
        self.height = None
        self.width = None

    ##
    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

   
    def image_callback (self, image_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return
        #self.height = image_msg.height
        #self.width = image_msg.width
        self.curr_image = self.bridge.imgmsg_to_cv2(image_msg,
                                                    desired_encoding=image_msg.encoding)
        self.image_encoding = image_msg.encoding
    
    def circle_callback (self, empty_msg):
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(self.curr_image,
                                                                self.aruco_dictionary,
                                                                parameters=self.aruco_parameters)
        if marker_ids is not None:
            #self.get_logger().info(f"marker found: {corners[0][0][0]}")
            self.get_logger().info("marker found!")
            self.height = floor((corners[0][0][0][1] + corners[0][0][3][1])/2)
            self.width = floor((corners [0][0][0][0] + corners[0][0][1][0])/2)
        else:
            self.get_logger().info("no marker found")
            return
                    
        #self.color_array = np.array([255, 0, 0], np.int32)
        #image_point = self.marker_point + (self.width, self.height)
        #cv2.circle(self.curr_image, image_point,50, self.cv_color_array)
        #cv2.circle(self.curr_image, (self.width, self.height),50, self.cv_color_array)
        cv2.circle(self.curr_image, (self.width, self.height), 75, (255, 0, 0))
        #self.curr_image.header = Header();
        self.image_to_pub = self.bridge.cv2_to_imgmsg(self.curr_image, self.image_encoding)
        #cv::circle(curr_image->image, marker_center[0][0], 100, CV_RGB(255,0,0));
        self.circled_pub.publish(self.image_to_pub)
        
    
    def marker_pose_callback (self, poses_msg):
        if poses_msg.poses is None:
            return
        self.num_marker = len(poses_msg.poses)
        self.center_idx = int(self.num_marker / 2)
        self.central_marker = poses_msg.poses[self.center_idx]
        self.central_pose = - np.array([self.central_marker.position.x, self.central_marker.position.y, self.central_marker.position.z])
        #cv::projectPoints (center_pose, id_matrix, std::vector<int> ({0, 0, 0}), cam_matrix, cam_dist, marker_center);
        cv2.projectPoints (self.central_pose, self.id_matrix, np.array([0, 0, 0], np.float32), self.intrinsic_mat, self.distortion, self.marker_point)
        




        




def main():
    rclpy.init()
    node = MarkerCircler()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
