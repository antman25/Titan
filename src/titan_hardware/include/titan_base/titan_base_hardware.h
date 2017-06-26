#ifndef TITAN_BASE_HARDWARE_H
#define TITAN_BASE_HARDWARE_H


#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>



class titan_base_hardware : public hardware_interface::RobotHW
{
	public:
		titan_base_hardware();
		titan_base_hardware(ros::NodeHandle n,ros::NodeHandle p);
	private:
		ros::NodeHandle nh;
		ros::NodeHandle nh_private;
		hardware_interface::JointStateInterface jnt_state_interface;
};
#endif
