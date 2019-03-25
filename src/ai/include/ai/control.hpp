#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "game_engine/RobotDescription.h"
#include "game_engine/RobotDescriptionArray.h"
#include "game_engine/UIState.h"

using namespace std;

geometry_msgs::Twist vel[2];
ros::Publisher vel_pub[2];
ros::Subscriber uistate_sub, robot_sub;
pthread_t t[2];
int id = 0;

#endif
