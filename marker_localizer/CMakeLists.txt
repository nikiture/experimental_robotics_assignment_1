cmake_minimum_required(VERSION 3.5)
project(marker_localizer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package (rclcpp REQUIRED)
find_package (geometry_msgs REQUIRED)
find_package (std_msgs REQUIRED)
find_package (cv_bridge REQUIRED)
find_package (OpenCV REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (ros2_aruco_interfaces REQUIRED)
find_package (targets_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package (aruco REQUIRED)


#add_executable (Robot_controller robot_control.cpp)
add_executable (aruco_test aruco_detection_test.cpp)

#ament_target_dependencies (Robot_controller rclcpp geometry_msgs)
ament_target_dependencies (aruco_test rclcpp tf2_ros tf2 nav_msgs ros2_aruco_interfaces targets_interface)



install (TARGETS
	aruco_test
	DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
