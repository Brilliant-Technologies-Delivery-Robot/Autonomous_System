#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>

using namespace std;

ros::Subscriber sub;
ros::Publisher odom_pub;

double x = 0.0;
double y = 0.0;
double th = 0.0;

double right_wheel_velocity = 0.0;
double left_wheel_velocity = 0.0;

const double wheel_base = 0.45; // Distance between the wheels (in meters)
const double wheel_radius = 0.065; // Radius of the wheels (in meters)

void VelocityCallback(const sensor_msgs::JointState::ConstPtr& vel)
{
  // Assuming vel->linear.x and vel->linear.y are the right and left wheel velocities respectively
  right_wheel_velocity = vel.velocity[0];
  left_wheel_velocity = vel.velocity[1];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  sub = n.subscribe("joint_states", 100, VelocityCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);
  while(ros::ok()) {
    ros::spinOnce();
    current_time = ros::Time::now();

    // Compute linear and angular velocities
    double linear_velocity = (right_wheel_velocity + left_wheel_velocity) * wheel_radius / 2.0;
    double angular_velocity = (right_wheel_velocity - left_wheel_velocity) * wheel_radius / wheel_base;

    // Compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = linear_velocity * cos(th) * dt;
    double delta_y = linear_velocity * sin(th) * dt;
    double delta_th = angular_velocity * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    // Since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // First, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // Send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // Next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // Set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // Set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.linear.y = 0; // Assuming no lateral movement
    odom.twist.twist.angular.z = angular_velocity;

    // Publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
  return 0;
}
