#include "titan_hardware.h"

#include <controller_manager/controller_manager.h>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>

typedef boost::chrono::steady_clock time_source;


void controlThread(ros::Rate rate, titan_hardware* robot, controller_manager::ControllerManager* cm)
{
	time_source::time_point last_time = time_source::now();

	while (1)
	{
		// Calculate monotonic time elapsed
		time_source::time_point this_time = time_source::now();
		boost::chrono::duration<double> elapsed_duration = this_time - last_time;
		ros::Duration elapsed(elapsed_duration.count());
		last_time = this_time;
		//robot->read();
		cm->update(ros::Time::now(), elapsed);
		robot->write();
		rate.sleep();
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "titan_hardware_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	titan_hardware titan(nh,nh_private);

	ros::NodeHandle controller_nh("");
	controller_manager::ControllerManager cm(&titan, controller_nh);
	boost::thread(boost::bind(controlThread, ros::Rate(50), &titan, &cm));

	ROS_INFO("Starting Titan Hardware node");
	ROS_INFO("Starting to spin...");


	//ros::spin();


	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
