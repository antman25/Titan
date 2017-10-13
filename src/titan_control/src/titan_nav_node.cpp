#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "titan_nav_node");
	ros::NodeHandle nh;

	MoveBaseClient ac("move_base", true);
        while (!ac.waitForServer(ros::Duration(5.0)))
        {
                ROS_INFO("Waiting for movebase server");
        }

	move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = 8.508;
        goal.target_pose.pose.position.y =  -6.41;
        goal.target_pose.pose.orientation.z = 0.55457;
        goal.target_pose.pose.orientation.w = 0.838455;

        ac.sendGoal(goal);
        ac.waitForResult();

	return 0;
}
