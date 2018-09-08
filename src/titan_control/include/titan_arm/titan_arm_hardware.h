#ifndef TITAN_ARM_HARDWARE_H
#define TITAN_ARM_HARDWARE_H


#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <titan_msgs/ArmStatus.h>
#include <titan_msgs/ArmCmd.h>

#define JOINT_ZAXIS		0
#define JOINT_LINK1		1
#define JOINT_LINK2		2
#define JOINT_LINK3		3


#define TOTAL_ARM_JOINTS	4


class titan_arm_hardware : public hardware_interface::RobotHW
{
	public:
		titan_arm_hardware();
		titan_arm_hardware(ros::NodeHandle n,ros::NodeHandle p);
		void registerControlInterfaces();

		void read();
		void write();
		void cbArmStatus(const titan_msgs::ArmStatus::ConstPtr &msg);
	private:
		ros::NodeHandle nh;
		ros::NodeHandle nh_private;
		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;
		double cmd[TOTAL_ARM_JOINTS];
		double pos[TOTAL_ARM_JOINTS];
		double vel[TOTAL_ARM_JOINTS];
		double eff[TOTAL_ARM_JOINTS];
		ros::Subscriber subArmStatus;
		ros::Publisher  pubArmCmd;
};
#endif
