#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "turtlesim/Pose.h"
#include <ros/ros.h>

#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

geometry_msgs::Twist get_started(turtlesim::Pose turtle1,turtlesim::Pose turtle2,geometry_msgs::Point goal) ;
void load_params() ;

#endif