#include "titan_hardware.h"

#include <boost/assign/list_of.hpp>
#include <sstream>

titan_hardware::titan_hardware()
{
	ROS_INFO("Zeroing out all joint states");
	for (int i=0;i<JOINT_TOTAL;i++)
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
	ROS_INFO("Zeroing out all joint states");
	for (int i=0;i<JOINT_TOTAL;i++)
	{
		pos[i] = 0;
		vel[i] = 0;
		cmd[i] = 0;

		initEncoder[i] = false;

		prevEncoderTime[i] = ros::Time::now();
	}
	registerControlInterfaces();


	ROS_INFO("Setting up publisher for motor commands");
	pubMotorVelocity = nh.advertise<titan_msgs::MotorVelocity>("motor_velocity", 1000);
	ROS_INFO("Subscribing to motor controller status");
	subMotorStatus = nh.subscribe("/motor_status", 1000, &titan_hardware::cbMotorStatus,this);

	ROS_INFO("Subscribing to arm controller status");
	subArmStatus = nh.subscribe("/arm_status", 1000, &titan_hardware::cbArmStatus,this);
	ROS_INFO("Setting up publisher for arm commands");
        pubArmCmd = nh.advertise<titan_msgs::ArmCmd>("/arm_cmd", 1000);

		
	
	
	ROS_INFO("Titan hardware init complete");
	
}

void titan_hardware::registerControlInterfaces()
{
	ROS_INFO("Registering wheel joints");
	ros::V_string vel_joint_names = boost::assign::list_of("left_front_wheel")("left_mid_wheel")("left_rear_wheel")("right_front_wheel")("right_mid_wheel")("right_rear_wheel");
	for (unsigned int i = 0; i < vel_joint_names.size(); i++)
	{
		ROS_INFO("Getting handle for %i", i);
		hardware_interface::JointStateHandle joint_state_handle(vel_joint_names[i], &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(joint_state_handle);
	
		hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
		jnt_vel_interface.registerHandle(joint_handle);
	}

	int index_offset = vel_joint_names.size();
	ros::V_string arm_joint_names = boost::assign::list_of("base_to_zaxis")("link1_to_link2")("link2_to_link3")("link3_to_eef");
	for (unsigned int i = 0; i < arm_joint_names.size(); i++)
	{
		ROS_INFO("Getting handle for %i", index_offset+i);
		hardware_interface::JointStateHandle joint_state_handle(arm_joint_names[i], &pos[index_offset+i], &vel[index_offset+i], &eff[index_offset+i]);
		jnt_state_interface.registerHandle(joint_state_handle);
	
		hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[index_offset+i]);
		jnt_pos_interface.registerHandle(joint_handle);
	}

	ROS_INFO("Registerering interfaces...");

	registerInterface(&jnt_state_interface);
	ROS_INFO("Joint State interface aquired");

	registerInterface(&jnt_pos_interface);
	ROS_INFO("Joint Position interface aquired");	

	registerInterface(&jnt_vel_interface);
	ROS_INFO("Joint Velocity interface aquired");
	
}



void titan_hardware::printDebug()
{
	//ROS_INFO("Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%f)", pos[MOTOR_LEFT_MID] ,vel[MOTOR_LEFT_FRONT],cmd[MOTOR_LEFT_MID], (float)angularToLinear(cmd[MOTOR_LEFT_MID]));
	//ROS_INFO("Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%f)", pos[MOTOR_RIGHT_MID] ,vel[MOTOR_RIGHT_FRONT],cmd[MOTOR_RIGHT_MID], (float)angularToLinear(cmd[MOTOR_RIGHT_MID]));
	
}

void titan_hardware::updateSetpoints()
{

	/*ROS_INFO("Front Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_FRONT_LEFT] ,vel[MOTOR_FRONT_LEFT], cmd[MOTOR_FRONT_LEFT], tickVelFrontLeft);
	ROS_INFO("Front Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_FRONT_RIGHT] ,vel[MOTOR_FRONT_RIGHT], cmd[MOTOR_FRONT_RIGHT], tickVelFrontRight);
	ROS_INFO("Rear Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_REAR_LEFT] ,vel[MOTOR_REAR_LEFT], cmd[MOTOR_REAR_LEFT], tickVelRearLeft);
	ROS_INFO("Rear Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)\n\n", pos[MOTOR_REAR_RIGHT] ,vel[MOTOR_REAR_RIGHT],cmd[MOTOR_REAR_RIGHT],  tickVelRearRight);*/

	/*titan_msgs::MotorVelocity msgMotorVelocity;
	msgMotorVelocity.left_angular_vel = cmd[LEFT_MASTER];
	msgMotorVelocity.right_angular_vel = cmd[RIGHT_MASTER];

	pubMotorVelocity.publish(msgMotorVelocity);*/
	
}

void titan_hardware::read()
{
}

void titan_hardware::write()
{
	titan_msgs::MotorVelocity msgMotorVelocity;
	msgMotorVelocity.left_angular_vel = cmd[LEFT_MASTER];
	msgMotorVelocity.right_angular_vel = cmd[RIGHT_MASTER];

	pubMotorVelocity.publish(msgMotorVelocity);
}

void titan_hardware::updateMotorData(int motor, long encoderVal, int32_t encoderVel)
{
	ros::Time currentTime=ros::Time::now();
	
	

	if (initEncoder[motor] == false)
	{
		initEncoder[motor] = true;
		prevEncoderValue[motor] = encoderVal;
		prevEncoderTime[motor] = currentTime;
		return;
	}
	//ROS_INFO("Updated Motor: %i - Data: %i", motor, (int)encoderVal);
	ros::Duration deltaTime=currentTime-prevEncoderTime[motor];
	long dDist = (encoderVal - prevEncoderValue[motor]);
	double dAngDist = dDist * ((2.0 * PI) / 1000.0);
	double dAngVel = encoderVel * (10.0 / 1000.0) * 2.0 * 3.1415926;
	double dT = (double)deltaTime.toSec();

	if (motor == LEFT_MASTER)
	{
		//ROS_INFO("dDist: %i %f", (int)dDist, dAngDist);
		pos[MOTOR_LEFT_FRONT] = pos[MOTOR_LEFT_MID] = pos[MOTOR_LEFT_REAR] += dAngDist;
		vel[MOTOR_LEFT_FRONT] = vel[MOTOR_LEFT_MID] = vel[MOTOR_LEFT_REAR] = dAngVel;
	}
	if (motor == RIGHT_MASTER)
	{
		pos[MOTOR_RIGHT_FRONT] = pos[MOTOR_RIGHT_MID] = pos[MOTOR_RIGHT_REAR] += dAngDist;
                vel[MOTOR_RIGHT_FRONT] = vel[MOTOR_RIGHT_MID] = vel[MOTOR_RIGHT_REAR] = dAngVel;
	}
	prevEncoderValue[motor] = encoderVal;
	prevEncoderTime[motor] = currentTime;
}

void titan_hardware::cbMotorStatus(const titan_msgs::Status::ConstPtr &msg)
{
	//updateMotorData(MOTOR_FRONT_LEFT, msg->data);	
	//ROS_INFO("Update Motor Data %i", msg->DeviceId);
	int motor = msg->DeviceId - 1;
	long encoderVal = msg->SensorPosition;
	int32_t encoderVel = msg->SensorVelocity;
	updateMotorData(motor,encoderVal,encoderVel);
	
}

void titan_hardware::cbArmStatus(const titan_msgs::ArmStatus::ConstPtr &msg)
{
        //updateMotorData(MOTOR_FRONT_LEFT, msg->data); 
        //ROS_INFO("Status Update");
        pos[JOINT_ZAXIS] = msg->z_axis_pos;
        pos[JOINT_LINK1] = msg->servo1_pos * (-3.1415926 / 180.0);
        pos[JOINT_LINK2] = msg->servo2_pos * (-3.1415926 / 180.0);
        pos[JOINT_LINK3] = msg->servo3_pos * (-3.1415926 / 180.0);

        /*ROS_INFO("Status: Base to Zaxis -- Pos: %f", pos[0]);
        ROS_INFO("Status: Link1 to Link2 -- Pos: %f", pos[1]);
        ROS_INFO("Status: Link2 to Link3 -- Pos: %f", pos[2]);
        ROS_INFO("Status: Link3 to eef -- Pos: %f", pos[3]);*/

}

