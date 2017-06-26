#include "titan_arm/titan_arm_hardware.h"

#include <boost/assign/list_of.hpp>
#include <sstream>

titan_arm_hardware::titan_arm_hardware()
{

}


titan_arm_hardware::titan_arm_hardware(ros::NodeHandle n, ros::NodeHandle p)
{
	nh = n;
	nh_private = p;

	for (int i=0;i<TOTAL_ARM_JOINTS;i++)
	{
		pos[i] = 0;
		vel[i] = 0;
		eff[i] = 0;
		cmd[i] = 0;
	}
	registerControlInterfaces();
}

void titan_arm_hardware::registerControlInterfaces()
{
	ros::V_string arm_joint_names = boost::assign::list_of("base_to_zaxis")("link1_to_link2")("link2_to_link3");
	for (unsigned int i = 0; i < arm_joint_names.size(); i++)
	{
		hardware_interface::JointStateHandle joint_state_handle(arm_joint_names[i], &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(joint_state_handle);
	
		hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
		jnt_pos_interface.registerHandle(joint_handle);
	}
	registerInterface(&jnt_state_interface);
	registerInterface(&jnt_pos_interface);

	
}

void titan_arm_hardware::read()
{
	/*ROS_INFO("Base to Zaxis -- Pos: %f cmd: %f", pos[0], cmd[0]);
	ROS_INFO("Link1 to Link2 -- Pos: %f cmd: %f", pos[1], cmd[1]);
	ROS_INFO("Link2 to Link3 -- Pos: %f cmd: %f\n", pos[2], cmd[2]);*/
	/*ROS_INFO("Front Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_FRONT_LEFT] ,vel[MOTOR_FRONT_LEFT], cmd[MOTOR_FRONT_LEFT], tickVelFrontLeft);
	ROS_INFO("Front Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_FRONT_RIGHT] ,vel[MOTOR_FRONT_RIGHT], cmd[MOTOR_FRONT_RIGHT], tickVelFrontRight);
	ROS_INFO("Rear Left Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)", pos[MOTOR_REAR_LEFT] ,vel[MOTOR_REAR_LEFT], cmd[MOTOR_REAR_LEFT], tickVelRearLeft);
	ROS_INFO("Rear Right Joint:\n\tPos: %f\n\tVel: %f\n\tcmd: %f (%i)\n\n", pos[MOTOR_REAR_RIGHT] ,vel[MOTOR_REAR_RIGHT],cmd[MOTOR_REAR_RIGHT],  tickVelRearRight);*/
}

void titan_arm_hardware::write()
{
	/*pos[0] += cmd[0] + 0.0001;
	pos[1] += cmd[1]+ 0.0001;
	pos[2] = cmd[2];*/
}
