cmake_minimum_required(VERSION 2.8.3)

project(delivery_robot_urdf)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
nav_msgs
sensor_msgs
tf)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	nav_msgs
	sensor_msgs)

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(odom_simulation src/odometry_publisher_sim.cpp)
target_link_libraries(odom_simulation ${catkin_LIBRARIES})
