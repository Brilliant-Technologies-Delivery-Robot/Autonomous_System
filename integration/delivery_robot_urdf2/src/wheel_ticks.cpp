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
double TicksPerMeter = 3235;
const double wheel_base = 0.45;    // Distance between the wheels (in meters)
const double wheel_radius = 0.065; // Radius of the wheels (in meters)

ros::Subscriber ticks_sub;
ros::Subscriber velocity_sub;
ros::Subscriber odom_sub;
ros::Publisher odom_pub;

double x = 0.0;  // x position (m)
double y = 0.0;  // y position (m)
double th = 0.0; // theta (rad)
double _PPSl=0.0;
double _PPSr=0.0;
double vl = 0.0;  
double vr = 0.0; 
double linear_velocity = 0.0;
double angular_velocity = 0.0;
double linear_distance = 0.0;
double angular_distance = 0.0;
double deltaLeft = 0.0;
double deltaRight = 0.0;
double distance_left = 0.0;
double distance_right = 0.0;
bool use_wheel_speeds = false;

void WheelSpeedCallback(const geometry_msgs::Vector3::ConstPtr& velocities)
{
  current_time_encoder = ros::Time::now();
  _PPSl = velocities->x;
  _PPSr = velocities->y ;
  vl= _PPSl * DistancePerCount; // left wheel linear velocity
  vr= _PPSr * DistancePerCount; // right wheel linear velocity
  last_time_encoder = current_time_encoder;

  // ROS_INFO("WheelSpeedCallback: _PPSl: %f, _PPSr: %f, vl: %f, vr: %f", _PPSl, _PPSr, vl, vr);
}

void WheelTicksCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
{
  current_time_encoder = ros::Time::now();
  deltaLeft = ticks->x - _PreviousLeftEncoderCounts;
  deltaRight = ticks->y - _PreviousRightEncoderCounts;
  distance_left = deltaLeft / TicksPerMeter;
  distance_right = deltaRight / TicksPerMeter;
  _PreviousLeftEncoderCounts = ticks->x;
  _PreviousRightEncoderCounts = ticks->y;
  last_time_encoder = current_time_encoder;

  // ROS_INFO("WheelTicksCallback: deltaLeft: %f, deltaRight: %f, distance_left: %f, distance_right: %f", deltaLeft, deltaRight, distance_left, distance_right);
}

void OdometryCallback(const geometry_msgs::Vector3::ConstPtr& odom_msg)
{
  x = odom_msg->x;
  y = odom_msg->y;
  th = odom_msg->z;

  // ROS_INFO("OdometryCallback: x: %f, y: %f, th: %f", x, y, th);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ticks_sub = n.subscribe("wheel_encoder", 100, WheelTicksCallback);
  velocity_sub = n.subscribe("wheel_speed", 100, WheelSpeedCallback);
  odom_sub = n.subscribe("position", 100, OdometryCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);   
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(10.0);

  while(ros::ok()) {
    current_time = ros::Time::now();

    // if (use_wheel_speeds == true) 
    // {
    //   linear_velocity = (vr + vl) / 2.0;
    //   angular_velocity = (vr - vl) / wheel_base;
    //   double dt = (current_time - last_time).toSec();
    //   double delta_x = linear_velocity * cos(th) * dt;
    //   double delta_y = linear_velocity * sin(th) * dt;
    //   double delta_th = angular_velocity * dt;
    //   x += delta_x;
    //   y += delta_y;
    //   th += delta_th;

    //   ROS_INFO("x: %f, y: %f, th: %f", x, y, th);
    // }

    // else
    // {
    //   linear_distance = (distance_left + distance_right) / 2.0;
    //   angular_distance = (distance_right - distance_left) / wheel_base;
    //   double delta_x = linear_distance * cos(th);
    //   double delta_y = linear_distance * sin(th);
    //   double delta_th = angular_distance;
    //   x += delta_x;
    //   y += delta_y;
    //   th += delta_th;

    //   ROS_INFO("x: %f, y: %f, th: %f", x, y, th);
    // }

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance[0] =  0.2;
    odom.pose.covariance[7] =  0.2;
    odom.pose.covariance[14] = 0.2;
    odom.pose.covariance[21] = 0.2;
    odom.pose.covariance[28] = 0.2;
    odom.pose.covariance[35] = 0.2;
   
    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.angular.z = angular_velocity;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
