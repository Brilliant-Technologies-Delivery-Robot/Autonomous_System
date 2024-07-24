#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>
#include "hardware_interface.h"

using namespace std;

long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
ros::Time current_time_encoder, last_time_encoder;
double DistancePerCount = (2* 3.14159265) / 1320;

const double wheel_base = 0.45; // Distance between the wheels (in meters)
const double wheel_radius = 0.065; // Radius of the wheels (in meters)

ros::Subscriber sub;
ros::Publisher motors_velocity_pub;

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
int velocity,result;
uint8_t wbuff[2];

void WheelCallback(const geometry_msgs::Vector3::ConstPtr& velocities)
{
  current_time_encoder = ros::Time::now();

  _PPSx = velocities->x;
  _PPSy = velocities->y ;
  vx=_PPSx * DistancePerCount;
  vy=_PPSy * DistancePerCount;
  last_time_encoder = current_time_encoder;
}

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}


void ROBOTHardwareInterface::init() {
	
	for(int i=0; i<2; i++)
	{
	// Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
       
    // Create velocity joint interface
	    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create Joint Limit interface   
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
	    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
	    velocityJointSaturationInterface.registerHandle(jointLimitsHandle);

	}
    
// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


void ROBOTHardwareInterface::read() {
 
    joint_velocity_[0]=vx;
    joint_velocity_[1]=vy;
    cout << "joints velocity: " <<  joint_velocity_[0] << ", " << joint_velocity_[1] << endl; 

}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    velocityJointSaturationInterface.enforceLimits(elapsed_time); 
    cout << "velocity commands: " <<  joint_velocity_command_[0] << ", " << joint_velocity_command_[1] << endl; 

   geometry_msgs::Vector3 vel_msg;
    vel_msg.x = joint_velocity_command_[0]; // Left motor velocity
    vel_msg.y = joint_velocity_command_[1]; // Right motor velocity

    // motors_velocity_pub.publish(vel_msg);		
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_interface");
    ros::NodeHandle nh;
    sub = nh.subscribe("wheel_encoder", 100, WheelCallback);
    motors_velocity_pub = nh.advertise<geometry_msgs::Vector3>("/vel", 1000);  
    ROBOTHardwareInterface ROBOT(nh);
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}