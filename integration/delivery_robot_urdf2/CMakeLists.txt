cmake_minimum_required(VERSION 2.8.3)

project(delivery_robot_urdf2)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
nav_msgs
sensor_msgs
tf
control_msgs
controller_manager)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	nav_msgs
	sensor_msgs
	control_msgs
	controller_manager)

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(odom src/wheel_ticks.cpp)
target_link_libraries(odom ${catkin_LIBRARIES})
