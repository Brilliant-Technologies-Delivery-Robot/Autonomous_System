#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>
#include <geometry_msgs/Vector3.h>

class ROBOTHardwareInterface : public hardware_interface::RobotHW 
{
public:
    ROBOTHardwareInterface(ros::NodeHandle& nh);
    ~ROBOTHardwareInterface();
    void init();
    void update(const ros::TimerEvent& e);
    void read();
    void write(ros::Duration elapsed_time);
    
protected:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
    
    std::string joint_name_[2];  
    double joint_position_[2];
    double joint_velocity_[2];
    double joint_effort_[2];
    double joint_velocity_command_[2];

    ros::NodeHandle nh_;
    ros::Publisher motors_velocity_pub;
    ros::Subscriber sub;
    ros::Timer non_realtime_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

#endif // HARDWARE_INTERFACE_H
