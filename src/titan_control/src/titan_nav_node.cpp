#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_srvs/Empty.h"
#include <stdlib.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "titan_nav_node");
	ros::NodeHandle nh;

	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmap");
	std_srvs::Empty srvClearCostMap;



	MoveBaseClient ac("move_base", true);
        while (!ac.waitForServer(ros::Duration(5.0)))
        {
                ROS_INFO("Waiting for movebase server");
        }

	if (client.call(srvClearCostMap))
	{
		ROS_INFO("Cost maps cleared");
	}
	else
	{
		ROS_ERROR("Failed to call service /move_base/clear_costmap");
		return 1;
	}

	

	move_base_msgs::MoveBaseGoal goal;

        /*goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 0;
        goal.target_pose.pose.position.y =  0;
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1;

        ac.sendGoal(goal);
        ac.waitForResult();*/

	return 0;
}
