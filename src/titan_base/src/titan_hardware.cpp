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

	subMotorStatus = nh.subscribe("motor_status", 1000, &titan_hardware::cbMotorStatus,this);
	

	pubMotorVelocity = nh.advertise<titan_base::MotorVelocity>("motor_velocity", 1000);

	setWheelDiameter(0.097155*2.0);
	setTicksPerRev(2048);
}

void titan_hardware::registerControlInterfaces()
{
	ros::V_string joint_names = boost::assign::list_of("left_front_wheel")("left_mid_wheel")("left_rear_wheel")("right_front_wheel")("right_mid_wheel")("right_rear_wheel");
	for (unsigned int i = 0; i < joint_names.size(); i++)
	{
		hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(joint_state_handle);
	
		hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
		jnt_vel_interface.registerHandle(joint_handle);
	}
	registerInterface(&jnt_state_interface);
	registerInterface(&jnt_vel_interface);
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
	ROS_INFO("Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%f)", pos[MOTOR_LEFT_MID] ,vel[MOTOR_LEFT_MID],cmd[MOTOR_LEFT_MID], angularToLinear(cmd[MOTOR_LEFT_MID]));
	ROS_INFO("Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%f)", pos[MOTOR_RIGHT_MID] ,vel[MOTOR_RIGHT_MID],cmd[MOTOR_RIGHT_MID], angularToLinear(cmd[MOTOR_RIGHT_MID]));
	
}

void titan_hardware::updateSetpoints()
{

	/*ROS_INFO("Front Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_FRONT_LEFT] ,vel[MOTOR_FRONT_LEFT], cmd[MOTOR_FRONT_LEFT], tickVelFrontLeft);
	ROS_INFO("Front Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_FRONT_RIGHT] ,vel[MOTOR_FRONT_RIGHT], cmd[MOTOR_FRONT_RIGHT], tickVelFrontRight);
	ROS_INFO("Rear Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_REAR_LEFT] ,vel[MOTOR_REAR_LEFT], cmd[MOTOR_REAR_LEFT], tickVelRearLeft);
	ROS_INFO("Rear Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)\n\n", pos[MOTOR_REAR_RIGHT] ,vel[MOTOR_REAR_RIGHT],cmd[MOTOR_REAR_RIGHT],  tickVelRearRight);*/

	titan_base::MotorVelocity msgMotorVelocity;
	msgMotorVelocity.left_angular_vel = cmd[MOTOR_LEFT_MID];
	msgMotorVelocity.right_angular_vel = cmd[MOTOR_RIGHT_MID];

	pubMotorVelocity.publish(msgMotorVelocity);
	
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
	long dDist = (encoderVal - prevEncoderValue[motor]) * 2.0;
	double dAngDist = dDist * ((2.0 * PI) / (double)2048.0);	
	double dT = (double)deltaTime.toSec();
	pos[motor] += dAngDist;
	vel[motor] = dAngDist / dT;
	prevEncoderValue[motor] = encoderVal;
	prevEncoderTime[motor] = currentTime;
}

void titan_hardware::cbMotorStatus(const titan_base::Status::ConstPtr &msg)
{
	//updateMotorData(MOTOR_FRONT_LEFT, msg->data);	
	//ROS_INFO("Update Motor Data %i", msg->DeviceId);
	int motor = msg->DeviceId - 1;
	long encoderVal = msg->SensorPosition;
	updateMotorData(motor,encoderVal);
	
}

