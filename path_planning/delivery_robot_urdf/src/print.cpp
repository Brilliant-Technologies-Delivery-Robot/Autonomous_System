#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <cmath>
#include <iostream>
#include "std_msgs/Float64MultiArray.h"
using namespace std;

ros::Publisher pub;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle nh;  
  ros::Rate loop_rate(10);
  
   pub = nh.advertise<std_msgs::Float64MultiArray>("topic_name", 1000);
   std_msgs::Float64MultiArray msg;
  while (ros::ok())
  {
    cout << "data: "  << std::endl;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
