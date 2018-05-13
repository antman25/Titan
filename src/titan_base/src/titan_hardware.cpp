#include "titan_base/titan_hardware.h"

#include <boost/assign/list_of.hpp>
#include <sstream>

titan_hardware::titan_hardware()
{
	for (int i=0;i<MOTOR_TOTAL;i++)
	{
		pos[i] = 0;
		vel[i] = 0;
		cmd[i] = 0;
	}

}

titan_hardware::titan_hardware(ros::NodeHandle n, ros::NodeHandle p)
{
	nh = n;
	nh_private = p;

	for (int i=0;i<MOTOR_TOTAL;i++)
	{
		pos[i] = 0;
		vel[i] = 0;
		cmd[i] = 0;

		initEncoder[i] = false;

		prevEncoderTime[i] = ros::Time::now();
	}
	registerControlInterfaces();

	subFrontLeftEncoder = nh.subscribe("encoder_fl_wheel", 10, &titan_hardware::cbFrontLeftEncoder,this);
	subFrontRightEncoder = nh.subscribe("encoder_fr_wheel", 10, &titan_hardware::cbFrontRightEncoder,this);
	subRearLeftEncoder = nh.subscribe("encoder_rl_wheel", 10, &titan_hardware::cbRearLeftEncoder,this);
	subRearRightEncoder = nh.subscribe("encoder_rr_wheel", 10, &titan_hardware::cbRearRightEncoder,this);

	pubFrontLeftSetpoint = nh.advertise<std_msgs::Int64>("vel_sp_fl_wheel", 10);
	pubFrontRightSetpoint = nh.advertise<std_msgs::Int64>("vel_sp_fr_wheel", 10);
	pubRearLeftSetpoint = nh.advertise<std_msgs::Int64>("vel_sp_rl_wheel", 10);
	pubRearRightSetpoint = nh.advertise<std_msgs::Int64>("vel_sp_rr_wheel", 10);

	setWheelDiameter(0.1524);
	setTicksPerRev(1296);
}

void titan_hardware::registerControlInterfaces()
{
	ros::V_string joint_names = boost::assign::list_of("front_left_wheel")("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
	for (unsigned int i = 0; i < joint_names.size(); i++)
	{
		hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(joint_state_handle);
	
		hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
		jnt_vel_interface.registerHandle(joint_handle);
	}
	registerInterface(&jnt_state_interface);
	registerInterface(&jnt_vel_interface);


	/*ros::V_string arm_joint_names = boost::assign::list_of("link1_to_link2")("link2_to_link3");
	for (unsigned int i = 0; i < arm_joint_names.size(); i++)
	{
		hardware_interface::JointStateHandle joint_state_handle(arm_joint_names[i], &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(joint_state_handle);
	
		hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
		jnt_vel_interface.registerHandle(joint_handle);
	}
	registerInterface(&jnt_state_interface);
	registerInterface(&jnt_vel_interface);*/

	
}


double titan_hardware::linearToAngular(const double &travel) const
{
	return travel / wheel_diameter * 2.0;
}

double titan_hardware::angularToLinear(const double &angle) const
{
	return angle * wheel_diameter / 2.0;
}

void titan_hardware::printDebug()
{
	/*ROS_INFO("Front Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%f)", pos[MOTOR_FRONT_LEFT] ,vel[MOTOR_FRONT_LEFT],cmd[MOTOR_FRONT_LEFT], angularToLinear(cmd[MOTOR_FRONT_LEFT]));
	ROS_INFO("Front Right Join	t:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%f)", pos[MOTOR_FRONT_RIGHT] ,vel[MOTOR_FRONT_RIGHT],cmd[MOTOR_FRONT_RIGHT], angularToLinear(cmd[MOTOR_FRONT_RIGHT]));
	ROS_INFO("Rear Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%f)", pos[MOTOR_REAR_LEFT] ,vel[MOTOR_REAR_LEFT],cmd[MOTOR_REAR_LEFT], angularToLinear(cmd[MOTOR_REAR_LEFT]));
	ROS_INFO("Rear Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%f)\n\n", pos[MOTOR_REAR_RIGHT] ,vel[MOTOR_REAR_RIGHT],cmd[MOTOR_REAR_RIGHT], angularToLinear(cmd[MOTOR_REAR_RIGHT]));*/
}

void titan_hardware::updateSetpoints()
{
	int tickVelFrontLeft = cmd[MOTOR_FRONT_LEFT] * ( TicksPerRev / ( 2 * PI) ) ;
	int tickVelFrontRight = cmd[MOTOR_FRONT_RIGHT] * ( TicksPerRev / ( 2 * PI) ) ;
	int tickVelRearLeft = cmd[MOTOR_REAR_LEFT] * ( TicksPerRev / ( 2 * PI) );
	int tickVelRearRight = cmd[MOTOR_REAR_RIGHT] * ( TicksPerRev / ( 2 * PI) );

	/*ROS_INFO("Front Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_FRONT_LEFT] ,vel[MOTOR_FRONT_LEFT], cmd[MOTOR_FRONT_LEFT], tickVelFrontLeft);
	ROS_INFO("Front Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_FRONT_RIGHT] ,vel[MOTOR_FRONT_RIGHT], cmd[MOTOR_FRONT_RIGHT], tickVelFrontRight);
	ROS_INFO("Rear Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_REAR_LEFT] ,vel[MOTOR_REAR_LEFT], cmd[MOTOR_REAR_LEFT], tickVelRearLeft);
	ROS_INFO("Rear Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)\n\n", pos[MOTOR_REAR_RIGHT] ,vel[MOTOR_REAR_RIGHT],cmd[MOTOR_REAR_RIGHT],  tickVelRearRight);*/

	std_msgs::Int64 msgVelFrontLeft;
	std_msgs::Int64 msgVelFrontRight;
	std_msgs::Int64 msgVelRearLeft;
	std_msgs::Int64 msgVelRearRight;

	msgVelFrontLeft.data = tickVelFrontLeft;
	msgVelFrontRight.data = tickVelFrontRight;
	msgVelRearLeft.data = tickVelRearLeft;
	msgVelRearRight.data = tickVelRearRight;

	pubFrontLeftSetpoint.publish(msgVelFrontLeft);
	pubFrontRightSetpoint.publish(msgVelFrontRight);
	pubRearLeftSetpoint.publish(msgVelRearLeft);
	pubRearRightSetpoint.publish(msgVelRearRight);
}


void titan_hardware::setWheelDiameter(double val)
{
    wheel_diameter = val;
}

double titan_hardware::getWheelDiameter()
{
    return wheel_diameter;
}

void titan_hardware::setTicksPerRev(long val)
{
    TicksPerRev = val;
}

long titan_hardware::getTicksPerRev()
{
    return TicksPerRev;
}

void titan_hardware::updateMotorData(int motor, long encoderVal)
{
	ros::Time currentTime=ros::Time::now();
	//ROS_INFO("Updated Motor: %i - Data: %i", motor, (int)encoderVal);
	if (initEncoder[motor] == false)
	{
		initEncoder[motor] = true;
		prevEncoderValue[motor] = encoderVal;
		prevEncoderTime[motor] = currentTime;
		return;
	}

	ros::Duration deltaTime=currentTime-prevEncoderTime[motor];
	long dDist = encoderVal - prevEncoderValue[motor];
	double dAngDist = dDist * ((2.0 * PI) / (double)TicksPerRev);	
	double dT = (double)deltaTime.toSec();
	pos[motor] += dAngDist;
	vel[motor] = dAngDist / dT;
	prevEncoderValue[motor] = encoderVal;
	prevEncoderTime[motor] = currentTime;
}

void titan_hardware::cbFrontLeftEncoder(const std_msgs::Int64::ConstPtr& msg)
{
	updateMotorData(MOTOR_FRONT_LEFT, msg->data);	
}

void titan_hardware::cbFrontRightEncoder(const std_msgs::Int64::ConstPtr& msg)
{
	updateMotorData(MOTOR_FRONT_RIGHT, msg->data);
}

void titan_hardware::cbRearLeftEncoder(const std_msgs::Int64::ConstPtr& msg)
{
	updateMotorData(MOTOR_REAR_LEFT, msg->data);
}

void titan_hardware::cbRearRightEncoder(const std_msgs::Int64::ConstPtr& msg)
{
	updateMotorData(MOTOR_REAR_RIGHT, msg->data);
}

