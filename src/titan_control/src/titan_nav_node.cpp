#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <stdlib.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "titan_nav_node");
	ros::NodeHandle nh;

	ros::Publisher initialpose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
	geometry_msgs::PoseWithCovarianceStamped poseStart;

	//ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("test", 1000);
	//std_msgs::String msg;
	//msg.data = "bae";

	ros::Rate loop_rate(1);

	/*while (ros::ok())
	{
		ROS_INFO("OK!");
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		
	}*/

	while (1)
	{
		int n = initialpose_pub.getNumSubscribers();
		ROS_INFO("Num subs: %i\n", n);
		if (n > 0)
			break;
		ros::spinOnce();
		loop_rate.sleep();
	}


	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	std_srvs::Empty srvClearCostMap;

	char frame_id[] = "map";
	poseStart.header.frame_id = frame_id;
	poseStart.header.stamp=ros::Time::now();
	poseStart.header.seq = 0;

	poseStart.pose.covariance[0] = 0.25;
	poseStart.pose.covariance[7] = 0.25;
	poseStart.pose.covariance[35] = 0.1;

	poseStart.pose.pose.position.x = 0.22;
	poseStart.pose.pose.position.y = -0.21;
	poseStart.pose.pose.position.z = 0.0;

	poseStart.pose.pose.orientation.x = 0.0;
        poseStart.pose.pose.orientation.y = 0.0;
        poseStart.pose.pose.orientation.z = 0.0;
	poseStart.pose.pose.orientation.w = 1.0;

	initialpose_pub.publish(poseStart);

	ROS_INFO("Initial pose published!");
	ros::spinOnce();




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

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 10.2;
        goal.target_pose.pose.position.y =  0.96;
        goal.target_pose.pose.orientation.z = 0.48;
        goal.target_pose.pose.orientation.w = 0.87;

        ac.sendGoal(goal);
        ac.waitForResult();
	ROS_INFO("GOAL 1 reached");


	goal.target_pose.pose.position.x = 11.6;
        goal.target_pose.pose.position.y =  12.5;
        goal.target_pose.pose.orientation.z = 0.72;
        goal.target_pose.pose.orientation.w = 0.68;

        ac.sendGoal(goal);
        ac.waitForResult();  

	ROS_INFO("Goal 2 reached");

 	goal.target_pose.pose.position.x = 11.4;
        goal.target_pose.pose.position.y =  1.94;
        goal.target_pose.pose.orientation.z = -0.937;
        goal.target_pose.pose.orientation.w = 0.34;

        ac.sendGoal(goal);
        ac.waitForResult();  

        ROS_INFO("Goal 3 reached");

	goal.target_pose.pose.position.x = 0.25;
        goal.target_pose.pose.position.y =  0.18;
        goal.target_pose.pose.orientation.z = 0.999;
        goal.target_pose.pose.orientation.w = 0;

        ac.sendGoal(goal);
        ac.waitForResult();  

        ROS_INFO("Goal 4 reached");





	return 0;
}
