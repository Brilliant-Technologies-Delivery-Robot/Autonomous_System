#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <cmath>
#include <iostream>

using namespace std;

// Global variables to store the latest odom and position data
nav_msgs::Odometry latest_odom;
double x = 0.0;  // x position (m)
double y = 0.0;  // y position (m)
double th = 0.0; // theta (rad)

// Function to normalize the quaternion
geometry_msgs::Quaternion normalizeQuaternion(const geometry_msgs::Quaternion& quaternion) {
    geometry_msgs::Quaternion normalized_quaternion;
    double norm = std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y + 
                            quaternion.z * quaternion.z + quaternion.w * quaternion.w);
    if (norm == 0) {
        ROS_WARN("Quaternion norm is zero. Skipping normalization.");
        return quaternion;
    }

    normalized_quaternion.x = quaternion.x / norm;
    normalized_quaternion.y = quaternion.y / norm;
    normalized_quaternion.z = quaternion.z / norm;
    normalized_quaternion.w = quaternion.w / norm;

    return normalized_quaternion;
}

// Function to update the latest odom message
void updateLatestOdom() {
    latest_odom.pose.pose.position.x = x;
    latest_odom.pose.pose.position.y = y;

    // Create a quaternion from the yaw (th)
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    latest_odom.pose.pose.orientation = odom_quat;

    latest_odom.pose.covariance[0] = 0.2;
    latest_odom.pose.covariance[7] = 0.2;
    latest_odom.pose.covariance[14] = 0.2;
    latest_odom.pose.covariance[21] = 0.2;
    latest_odom.pose.covariance[28] = 0.2;
    latest_odom.pose.covariance[35] = 0.2;

    // Update timestamp
    latest_odom.header.stamp = ros::Time::now();
}

// IMU callback function
void imuCallback(const sensor_msgs::Imu::ConstPtr& data, 
                 ros::Publisher& imu_pub, 
                 ros::Publisher& yaw_pub, 
                 ros::Publisher& odom_pub) {
    sensor_msgs::Imu imu_msg = *data;

    // Normalize quaternion
    imu_msg.orientation = normalizeQuaternion(imu_msg.orientation);
    imu_msg.orientation_covariance = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};

    // Publish the normalized IMU data
    imu_pub.publish(imu_msg);

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    tf::Quaternion quat(imu_msg.orientation.x, imu_msg.orientation.y, 
                        imu_msg.orientation.z, imu_msg.orientation.w);
    tf::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // Publish yaw
    std_msgs::Float64 yaw_msg;
    yaw_msg.data = yaw;
    yaw_pub.publish(yaw_msg);

    // Update the latest odom with current x, y, th values
    updateLatestOdom();

    // Publish the updated odom message
    if (latest_odom.header.stamp != ros::Time(0)) {
        odom_pub.publish(latest_odom);
    }
}

// Odom callback function (Position updates)
void odomCallback(const geometry_msgs::Vector3::ConstPtr& odom_msg) {
    x = odom_msg->x;
    y = odom_msg->y;
    th = odom_msg->z;

    // Update latest odom message when position is updated
    updateLatestOdom();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_odom_sync_node");
    ros::NodeHandle n;

    // Publishers
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu_data", 10);
    ros::Publisher yaw_pub = n.advertise<std_msgs::Float64>("/yaw", 10);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);

    // Subscribers
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/imu/data", 10, 
        boost::bind(imuCallback, _1, boost::ref(imu_pub), boost::ref(yaw_pub), 
                    boost::ref(odom_pub)));
    ros::Subscriber position_sub = n.subscribe("position", 100, odomCallback);

    // Initialize latest_odom header
    latest_odom.header.frame_id = "odom";
    latest_odom.child_frame_id = "base_footprint";

    // Main loop with 10Hz rate
    ros::Rate r(10.0);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
