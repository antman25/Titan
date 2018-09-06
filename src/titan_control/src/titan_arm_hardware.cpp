#include <titan_arm/titan_arm_hardware.h>

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
		pos[i] = 0.0;
		vel[i] = 0.0;
		eff[i] = 0.0;
		cmd[i] = 0.0;
	}
	registerControlInterfaces();

	subArmStatus = nh.subscribe("/arm_status", 1000, &titan_arm_hardware::cbArmStatus,this);
	//pubArmCmd = nh.advertise<titan_msgs::ArmCmd>("/arm_cmd", 1000);
}

void titan_arm_hardware::registerControlInterfaces()
{
	ROS_INFO("Registering Control Interfaces");
	ros::V_string arm_joint_names = boost::assign::list_of("base_to_zaxis")("link1_to_link2")("link2_to_link3")("link3_to_eff");
	for (unsigned int i = 0; i < arm_joint_names.size(); i++)
	{
		hardware_interface::JointStateHandle joint_state_handle(arm_joint_names[i], &pos[i], &vel[i], &eff[i]);
		jnt_state_interface.registerHandle(joint_state_handle);
	
		hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd[i]);
		jnt_pos_interface.registerHandle(joint_handle);
	}
	registerInterface(&jnt_state_interface);
	registerInterface(&jnt_pos_interface);
	ROS_INFO("Finished");
	
}

void titan_arm_hardware::read()
{
	/*ROS_INFO("Base to Zaxis -- Pos: %f cmd: %f", pos[0], cmd[0]);
	ROS_INFO("Link1 to Link2 -- Pos: %f cmd: %f", pos[1], cmd[1]);
	ROS_INFO("Link2 to Link3 -- Pos: %f cmd: %f", pos[2], cmd[2]);
	ROS_INFO("Link3 to eef -- Pos: %f cmd: %f\n", pos[3], cmd[3]);*/
	
}

void titan_arm_hardware::write()
{
	/*pos[0] += cmd[0] + 0.0001;
	pos[1] += cmd[1]+ 0.0001;
	pos[2] = cmd[2];*/

	/*titan_msgs::ArmCmd arm_cmd;
	arm_cmd.z_axis_pos = cmd[0];
	arm_cmd.servo1_pos = cmd[1];
	arm_cmd.servo2_pos = cmd[2];
	arm_cmd.servo3_pos = cmd[3];
	pubArmCmd.publish(arm_cmd);*/
}

void titan_arm_hardware::cbArmStatus(const titan_msgs::ArmStatus::ConstPtr &msg)
{
	//updateMotorData(MOTOR_FRONT_LEFT, msg->data);	
	//ROS_INFO("Status Update");
	pos[0] = msg->z_axis_pos;
	pos[1] = msg->servo1_pos * (3.1415926 / 180.0);
	pos[2] = msg->servo2_pos * (3.1415926 / 180.0);
	pos[3] = msg->servo3_pos * (3.1415926 / 180.0);

	/*ROS_INFO("Status: Base to Zaxis -- Pos: %f", pos[0]);
	ROS_INFO("Status: Link1 to Link2 -- Pos: %f", pos[1]);
	ROS_INFO("Status: Link2 to Link3 -- Pos: %f", pos[2]);
	ROS_INFO("Status: Link3 to eef -- Pos: %f", pos[3]);*/
	
}
