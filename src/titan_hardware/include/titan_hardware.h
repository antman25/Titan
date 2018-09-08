#ifndef TITAN_HARDWARE_H
#define TITAN_HARDWARE_H


#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <titan_msgs/Status.h>
#include <titan_msgs/MotorVelocity.h>
#include <titan_msgs/ArmStatus.h>
#include <titan_msgs/ArmCmd.h>


#define JOINT_LEFT_FRONT	0
#define JOINT_LEFT_MID		1
#define JOINT_LEFT_REAR		2
#define JOINT_RIGHT_FRONT	3
#define JOINT_RIGHT_MID		4
#define JOINT_RIGHT_REAR	5

#define MOTOR_LEFT_FRONT	JOINT_LEFT_FRONT
#define MOTOR_LEFT_MID		JOINT_LEFT_MID			
#define MOTOR_LEFT_REAR		JOINT_LEFT_REAR			
#define MOTOR_RIGHT_FRONT	JOINT_RIGHT_FRONT	
#define MOTOR_RIGHT_MID		JOINT_RIGHT_MID
#define MOTOR_RIGHT_REAR	JOINT_RIGHT_REAR

#define JOINT_ZAXIS             6
#define JOINT_LINK1             7
#define JOINT_LINK2             8
#define JOINT_LINK3             9

#define JOINT_TOTAL		10

#define MOTOR_TOTAL		6

#define LEFT_MASTER		JOINT_LEFT_FRONT
#define RIGHT_MASTER		JOINT_RIGHT_FRONT

#define PI			3.1415926

class titan_hardware : public hardware_interface::RobotHW
{
public:
	titan_hardware();
	titan_hardware(ros::NodeHandle n,ros::NodeHandle p);
	
	void registerControlInterfaces();

	void updateSetpoints();

	void printDebug();
	void read();
	void write();

	void updateMotorData(int motor, long encoderVal, int32_t encoderVel);
	void cbMotorStatus(const titan_msgs::Status::ConstPtr &msg);
	void cbArmStatus(const titan_msgs::ArmStatus::ConstPtr &msg);

private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;

	

	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	double cmd[JOINT_TOTAL];
	double pos[JOINT_TOTAL];
	double vel[JOINT_TOTAL];
	double eff[JOINT_TOTAL];

	long prevEncoderValue[JOINT_TOTAL];
	ros::Time prevEncoderTime[JOINT_TOTAL];
	bool initEncoder[JOINT_TOTAL];


	ros::Subscriber subMotorStatus;
	ros::Publisher pubMotorVelocity;

	ros::Subscriber subArmStatus;
        ros::Publisher  pubArmCmd;

};

#endif // TITAN_HARDWARE_H
