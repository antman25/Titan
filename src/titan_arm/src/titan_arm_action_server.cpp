//http://clopema.felk.cvut.cz/redmine/projects/clopema/wiki/Sending_trajectory_to_the_controller
//http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
//http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

#include <std_msgs/Float32.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>




#include <controller_manager/controller_manager.h>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include "titan_arm/titan_arm_hardware.h"

typedef boost::chrono::steady_clock time_source;




class RobotTrajectoryFollower
{
protected:

	ros::NodeHandle nh_;
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_; 
	std::string action_name_;
	control_msgs::FollowJointTrajectoryFeedback feedback_;
	control_msgs::FollowJointTrajectoryResult result_;
	ros::Publisher servo1cmd;// = n.advertise<std_msgs::Float32>("/servo1_angle_cmd", 1000);
	ros::Publisher servo2cmd;// = n.advertise<std_msgs::Float32>("/servo2_angle_cmd", 1000);
	ros::Publisher servo3cmd;
	ros::Publisher zaxiscmd;

public:

	RobotTrajectoryFollower(std::string name) : as_(nh_, name, false), action_name_(name)
	{
		ROS_INFO("ROBOT TRAJECTORY FOLLOWER - INIT");
		servo1cmd = nh_.advertise<std_msgs::Float32>("/servo1_angle_cmd", 1000);
		servo2cmd = nh_.advertise<std_msgs::Float32>("/servo2_angle_cmd", 1000);
		servo3cmd = nh_.advertise<std_msgs::Float32>("/servo3_angle_cmd", 1000);
		zaxiscmd = nh_.advertise<std_msgs::Float32>("/z_axis_cmd", 1000);

    		//Register callback functions:
		as_.registerGoalCallback(boost::bind(&RobotTrajectoryFollower::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&RobotTrajectoryFollower::preemptCB, this));
		
		as_.start();
	}

	~RobotTrajectoryFollower(void)//Destructor
	{
	}

	//void goalCB(control_msgs::FollowJointTrajectoryGoal &goal)
	//void goalCB(const control_msgs::FollowJointTrajectoryGoal::ConstPtr &goal)
	void goalCB()
	{

		// accept the new goal
		control_msgs::FollowJointTrajectoryGoalConstPtr goal = as_.acceptNewGoal();
		trajectory_msgs::JointTrajectory trajectory = goal->trajectory;

		std::vector<trajectory_msgs::JointTrajectoryPoint> points = trajectory.points;
		ROS_INFO("GOAL ACCEPTED - %i Total Points", (int)points.size());
		
		for (int i=0; i<points.size(); ++i)
		{
			std::vector<double> positions = points[i].positions;
			ros::Duration time_from_start = points[i].time_from_start;
			std_msgs::Float32 ang1;
			std_msgs::Float32 ang2;
			std_msgs::Float32 ang3;
			std_msgs::Float32 zaxis;

			zaxis.data = positions[0];
			ang1.data = positions[1] * (-180.0/3.14);
			ang2.data = positions[2] * (-180.0/3.14);
			ang3.data = positions[3] * (-180.0/3.14);

			servo1cmd.publish(ang1);
			servo2cmd.publish(ang2);
			servo3cmd.publish(ang3);
			zaxiscmd.publish(zaxis);

			for (int j=0;j<positions.size();j++)
			{
				ROS_INFO("GOAL[%i] - Pos[%i] = %f", i,j,(float)positions[j]);
			}
                        ros::Duration(0.1).sleep();
                        //time_from_start.sleep();
		}
		

		// print all the remaining numbers
		//for(std::vector<int>::const_iterator it = points->data.begin(); it != points->data.end(); ++it)
		//{
		//	Arr[i] = *it;
		//	i++;
		//}
		

		

		
	}

	void preemptCB()
	{
		ROS_INFO("%s: Preempted", action_name_.c_str());
		// set the action state to preempted
		as_.setPreempted();
	}
};

void controlThread(ros::Rate rate, titan_arm_hardware* robot, controller_manager::ControllerManager* cm)
{
	time_source::time_point last_time = time_source::now();

	while (1)
	{
		// Calculate monotonic time elapsed
		time_source::time_point this_time = time_source::now();
		boost::chrono::duration<double> elapsed_duration = this_time - last_time;
		ros::Duration elapsed(elapsed_duration.count());
		last_time = this_time;
		robot->read();
		cm->update(ros::Time::now(), elapsed);
		robot->write();
		rate.sleep();
	}
}



int main(int argc, char** argv)
{
	ROS_INFO("START OF ACTION SERVER CODE");

	ros::init(argc, argv, "action_server");
		

	RobotTrajectoryFollower RobotTrajectoryFollower("/titan_arm_controller/joint_trajectory_action");
		
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	titan_arm_hardware titan_arm(nh,nh_private);

	ros::NodeHandle controller_nh("");
	controller_manager::ControllerManager cm(&titan_arm, controller_nh);
	boost::thread(boost::bind(controlThread, ros::Rate(50), &titan_arm, &cm));
	


  	ROS_INFO("Starting Titan Hardware / Action Server node");
	ROS_INFO("Starting to spin...");


	ros::spin();


	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;

}
