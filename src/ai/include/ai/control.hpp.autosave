#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "game_engine/RobotDescription.h"
#include "game_engine/RobotDescriptionArray.h"
#include "game_engine/UIState.h"

using namespace std;

#define ROTATE 0
#define TRANSL 1

geometry_msgs::Twist vel[2];
ros::Publisher vel_pub[2];
ros::Subscriber uistate_sub, robot_sub;
pthread_t t[2];
int id = 0;

struct robot_info {
  int teamId;
  int tagId;
  int x;
  int y;
  double height, angle;
  char addr0, addr1;
  int vel1, vel2, previous_vel;
  bool collisionFlag, threadIsRunning;
  int collisionStateVar;
  bool autonomous_drive;
  bool immobilized;
  int damage, kills;
};

struct ui_state {
  bool setupStart, gameStart, teamGame, paused, aiGame;
  int seconds;
};

std::vector<robot_info> robot;
ui_state uistate;

#endif
