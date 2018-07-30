#ifndef TITAN_HARDWARE_H
#define TITAN_HARDWARE_H


#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <titan_base/Status.h>
#include <titan_base/MotorVelocity.h>

#define MOTOR_LEFT_FRONT	0
#define MOTOR_LEFT_MID		1
#define MOTOR_LEFT_REAR		2
#define MOTOR_RIGHT_FRONT	3
#define MOTOR_RIGHT_MID		4
#define MOTOR_RIGHT_REAR	5
#define MOTOR_TOTAL		6

#define PI			3.1415926

class titan_hardware : public hardware_interface::RobotHW
{
public:
	titan_hardware();
	titan_hardware(ros::NodeHandle n,ros::NodeHandle p);
	
	void registerControlInterfaces();

	void updateSetpoints();

	void printDebug();
	void setTicksPerRev(long val);
	long getTicksPerRev();
	void setWheelDiameter(double val);
	double getWheelDiameter();

	void updateMotorData(int motor, long encoderVal);


	void cbMotorStatus(const titan_base::Status::ConstPtr &msg);

private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;

	double linearToAngular(const double &travel) const;
	double angularToLinear(const double &angle) const;

	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	double cmd[MOTOR_TOTAL];
	double pos[MOTOR_TOTAL];
	double vel[MOTOR_TOTAL];
	double eff[MOTOR_TOTAL];

	long prevEncoderValue[MOTOR_TOTAL];
	ros::Time prevEncoderTime[MOTOR_TOTAL];
	bool initEncoder[MOTOR_TOTAL];

	long TicksPerRev;
	double wheel_diameter;

	ros::Subscriber subMotorStatus;

	ros::Publisher pubMotorVelocity;

};

#endif // TITAN_HARDWARE_H
