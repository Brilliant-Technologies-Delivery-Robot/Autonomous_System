#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>

using namespace std;



long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
ros::Time current_time_encoder, last_time_encoder;
double DistancePerCount = (2* 3.14159265 * 0.065) / 1320;

const double wheel_base = 0.45; // Distance between the wheels (in meters)
const double wheel_radius = 0.065; // Radius of the wheels (in meters)

ros::Subscriber sub;
ros::Publisher odom_pub;

double x = 0.0;
double y = 0.0;
double th = 0.0;
double _PPSx=0.0;
double _PPSy=0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
double deltaLeft = 0.0;
double deltaRight = 0.0;

void WheelCallback(const geometry_msgs::Vector3::ConstPtr& velocities)
{
  current_time_encoder = ros::Time::now();

  _PPSx = velocities->x;
  _PPSy = velocities->y ;
  vx=_PPSx * DistancePerCount;
  vy=_PPSy * DistancePerCount;
 /* deltaLeft = ticks->x - _PreviousLeftEncoderCounts;
  deltaRight = ticks->y - _PreviousRightEncoderCounts;

  vx = deltaLeft * DistancePerCount; // (current_time_encoder - last_time_encoder).toSec();
  vy = deltaRight * DistancePerCount; // (current_time_encoder - last_time_encoder).toSec();*/  
  //std::cout<< "vx" << endl;
  //std::cout<< vx << endl;
  //std::cout<< "vy" << endl;
  //std::cout<< vy << endl;

 // _PreviousLeftEncoderCounts = ticks->x;
 // _PreviousRightEncoderCounts = ticks->y;
  last_time_encoder = current_time_encoder;
  
  //std::cout<< "wheel_encoders_msg" ;
}

int main(int argc, char **argv)
{

//  cout<< "inside_main" ;
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  sub = n.subscribe("wheel_encoder", 100, WheelCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);   
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(ros::ok()) {
    //cout<<"inside ros ok";
    current_time = ros::Time::now();

    double linear_velocity = (vx + vy) * wheel_radius / 2.0;
    double vth = (vx - vy) * wheel_radius / wheel_base;
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    // std::cout<< "dt" << endl;
    // std::cout<< dt << endl;
    double delta_x = linear_velocity * cos(th) * dt;
    double delta_y = linear_velocity * sin(th) * dt;
    double delta_th = vth * dt;
    // std::cout<< "delta_th" << endl;
    // std::cout<< delta_th << endl;
    x += delta_x;
    y += delta_y;
    th += delta_th;
    // std::cout << "x: " << x << endl ;
    // std::cout << "y: " << y << endl ;
    // std::cout<< "th" << endl;
    // std::cout<< th << endl;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    
    // std::cout << "odom_quat" << endl;
    // std::cout << odom_quat << endl;
   
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    // std::cout<< "vth" << endl;
    // std::cout<< vth << endl;
    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
