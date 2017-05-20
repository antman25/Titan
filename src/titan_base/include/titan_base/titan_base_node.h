#ifndef TITAN_BASE_H
#define TITAN_BASE_H

#include <ros/ros.h>

#include <boost/chrono.hpp>
#include <boost/thread.hpp>


#include <controller_manager/controller_manager.h>

#include "titan_base/titan_hardware.h"

typedef boost::chrono::steady_clock time_source;

class titan_base_node
{
public:
	titan_base_node();
private:
	

};

#endif // TITAN_BASE_H
