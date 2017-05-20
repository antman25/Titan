#ifndef TITAN_HARDWARE_H
#define TITAN_HARDWARE_H


#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

#define MOTOR_FRONT_LEFT	0
#define MOTOR_FRONT_RIGHT	1
#define MOTOR_REAR_LEFT		2
#define MOTOR_REAR_RIGHT	3
#define MOTOR_TOTAL		4

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


	void cbFrontLeftEncoder(const std_msgs::Int64::ConstPtr& msg);
	void cbFrontRightEncoder(const std_msgs::Int64::ConstPtr& msg);
	void cbRearLeftEncoder(const std_msgs::Int64::ConstPtr& msg);
	void cbRearRightEncoder(const std_msgs::Int64::ConstPtr& msg);

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

	ros::Subscriber subFrontLeftEncoder;
	ros::Subscriber subFrontRightEncoder;
	ros::Subscriber subRearLeftEncoder;
	ros::Subscriber subRearRightEncoder;

	ros::Publisher pubFrontLeftSetpoint;
	ros::Publisher pubFrontRightSetpoint;
	ros::Publisher pubRearLeftSetpoint;
	ros::Publisher pubRearRightSetpoint;
};

#endif // TITAN_HARDWARE_H
