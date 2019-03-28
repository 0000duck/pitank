#include "../../include/graphics/dialog.hpp"
#include "ui_dialog.h"

int person_counter = 0;

Dialog::Dialog(int argc, char **argv, QWidget *parent) : QDialog(parent), ui(new Ui::Dialog), qnode(argc, argv) {
  ui->setupUi(this);

  init_argc = argc;
  init_argv = argv;

  auto const desktop(QApplication::desktop());
  setGeometry(desktop->screenGeometry(1));
  this->setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
  this->showFullScreen();

  qnode.init();
  nh = new ros::NodeHandle("~");

  nh->getParam("cam_pos_x", cam_pos.x);
  nh->getParam("cam_pos_y", cam_pos.y);
  nh->getParam("cam_pos_z", cam_pos.z);

  int tempAddr[4];
  nh->getParam("addr1", tempAddr[0]);
  nh->getParam("addr2", tempAddr[1]);
  nh->getParam("addr3", tempAddr[2]);
  nh->getParam("addr4", tempAddr[3]);

  addr[0] = tempAddr[0];
  addr[1] = tempAddr[1];
  addr[2] = tempAddr[2];
  addr[3] = tempAddr[3];

  nh->getParam("tag_id1", tag_id[0]);
  nh->getParam("tag_id2", tag_id[1]);
  nh->getParam("tag_id3", tag_id[2]);
  nh->getParam("tag_id4", tag_id[3]);

  tagAndXbee[tag_id[0]] = addr[0];
  tagAndXbee[tag_id[1]] = addr[1];
  tagAndXbee[tag_id[2]] = addr[2];
  tagAndXbee[tag_id[3]] = addr[3];

  pose_sub = nh->subscribe("/tag_pixel_detections", 1, &Dialog::listener, this);
  thresh_pub = nh->advertise<std_msgs::Int64>("/calibration_thresh", 1);
  gameStarted_pub = nh->advertise<std_msgs::Bool>("/game_state", 1);
  corners_pub = nh->advertise<std_msgs::Float32MultiArray>("/map_corners", 1);
  robotDescription_pub = nh->advertise<game_engine::RobotDescriptionArray>("/robots_description", 1);
  uiState_pub = nh->advertise<game_engine::UIState>("/ui_state", 1);
  controls_sub[0] = nh->subscribe("/cmd_velA", 1, &Dialog::velJoystickA, this);
  controls_sub[1] = nh->subscribe("/cmd_velB", 1, &Dialog::velJoystickB, this);
  controls_sub[2] = nh->subscribe("/cmd_velC", 1, &Dialog::velJoystickC, this);
  controls_sub[3] = nh->subscribe("/cmd_velD", 1, &Dialog::velJoystickD, this);

  ui->graphicsView->setScene(&scene);
  scene.setSceneRect(offset, offset, projectedImageSizeX, projectedImageSizeY);
  setWindowTitle("Game Window");

  M = new map();

  QLineF borderLine[4];
  borderLine[0].setP1(scene.sceneRect().topLeft() + QPointF(borderOffset_x, 0)); borderLine[0].setP2(scene.sceneRect().bottomLeft() + QPointF(borderOffset_x, 0));
  borderLine[1].setP1(scene.sceneRect().topLeft() + QPointF(borderOffset_x, 0)); borderLine[1].setP2(scene.sceneRect().topRight());
  borderLine[2].setP1(scene.sceneRect().bottomLeft() + QPointF(borderOffset_x, 0)); borderLine[2].setP2(scene.sceneRect().bottomRight());
  borderLine[3].setP1(scene.sceneRect().topRight()); borderLine[3].setP2(scene.sceneRect().bottomRight());
  for(int i = 0; i < 4; i++) {
    w = new wall(borderLine[i].p1(), borderLine[i].p2(), true, false, false);
    scene.addItem(w);
  }

  for(int i = 0; i < max_number_robots; i++)
    bullet_interval[i].start();

  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), &scene, SLOT(update()));
  connect(timer, SIGNAL(timeout()), this, SLOT(startSIGNAL()));
  timer->start(50);
}

bool Dialog::reset_game() {
  Q_EMIT reset_buttons();

  tagAndRobot.clear();
  robot.clear();
  team.clear();
  number_robots = 0;
  driving_counter = 0;

  Q_FOREACH(QGraphicsItem *item, scene.items()) {
    wall *wall_item = dynamic_cast<wall *>(item);
    tag *tag_item = dynamic_cast<tag *>(item);
    circle *circle_item = dynamic_cast<circle *>(item);
    bullet *bullet_item = dynamic_cast<bullet *>(item);
    pallet *pallet_item = dynamic_cast<pallet *>(item);
    machine *machine_item = dynamic_cast<machine *>(item);
    person *person_item = dynamic_cast<person *>(item);
    racemap *racemap_item = dynamic_cast<racemap *>(item);
    game_timer *game_timer_item = dynamic_cast<game_timer *>(item);

    if(item->parentItem() == NULL) {
      if(wall_item != NULL) {
        if(wall_item->border == false)
          wall_item->delete_item();
      }

      if(tag_item != NULL)
        tag_item->delete_item();

      if(circle_item != NULL)
        circle_item->delete_item();

      if(bullet_item != NULL)
        bullet_item->delete_item();

      if(pallet_item != NULL)
        pallet_item->delete_item();

      if(machine_item != NULL)
        machine_item->delete_item();

      if(person_item != NULL)
        person_item->delete_item();

      if(racemap_item != NULL)
        racemap_item->delete_item();

      if(game_timer_item != NULL)
        game_timer_item->delete_item();
    }
  }
}

void Dialog::initialize_pitank(const apriltags_ros::AprilTagDetectionArray& msg) {
  int team_id, capacity;

  team_info t1(Qt::green,  cv::Point2f(classNr_x, classNr_yA), 0);
  team_info t2(Qt::blue,   cv::Point2f(classNr_x, classNr_yB), 1);
  team_info t3(Qt::yellow, cv::Point2f(classNr_x, classNr_yC), 2);
  team_info t4(Qt::red,    cv::Point2f(classNr_x, classNr_yD), 3);
  team_info t5(Qt::black,  cv::Point2f(classNr_x, classNr_yE), 4);

  if(teamGame == true)
    capacity = 2;
  else
    capacity = 1;

  game_end = false;

  drawPitankMap();

  clk = new game_timer(cv::Point2f(classNr_x, classNr_yE + 20), game_clock);
  scene.addItem(clk);

  for(size_t i = 0; i < msg.detections.size(); i++) {
    if(tagAndRobot.find(msg.detections[i].id) == tagAndRobot.end() && robot.size() < max_number_robots) {
      id = robot.size();
      tagAndRobot[msg.detections[i].id] = robot.size();

      linearTransformation = M->linearTransf;
      offsetTranformation  = M->offsetTransf;

      for(size_t j = 0; j < team.size(); j++) {
        if(team[j].member_id.size() < capacity) {
          team_id = j;
          team[j].member_id.push_back(id);
          break;
        }
      }

      t = new tag(msg.detections[i], addr[id], robot_z[id], team_id);
      t->game = 0;
      scene.addItem(t);
    }
  }

  for(size_t i = 0; i < team.size(); i++) {
    if(team[i].member_id.size() > 0) {
      c = new circle(i, team[i].team_color, false, 0);
      scene.addItem(c);
    }
  }

  Q_EMIT detected_robots(tagAndRobot);
}


void Dialog::initialize_robotf(const apriltags_ros::AprilTagDetectionArray &msg) {

  int team_id;
  team_info t1(Qt::green,  cv::Point2f(classNr_x, classNr_yA), 0);
  team_info t2(Qt::blue,   cv::Point2f(classNr_x, classNr_yB), 1);

  drawRobotFactoryMap();

  game_end = false;

  clk = new game_timer(cv::Point2f(classNr_x, classNr_yE + 20), game_clock);
  scene.addItem(clk);

  for(size_t i = 0; i < msg.detections.size(); i++) {
    if(tagAndRobot.find(msg.detections[i].id) == tagAndRobot.end() && robot.size() < max_number_robots) {
      id = robot.size();
      tagAndRobot[msg.detections[i].id] = robot.size();

      if(id % 2 == 0) {
        team_id = 0;
        team[0].member_id.push_back(id);
      }
      else {
        team_id = 1;
        team[1].member_id.push_back(id);
      }

      linearTransformation = M->linearTransf;
      offsetTranformation  = M->offsetTransf;

      t = new tag(msg.detections[i], addr[id], robot_z[id], team_id);
      t->game = 1;
      scene.addItem(t);
    }
  }

  for(size_t i = 0; i < team.size(); i++) {
    if(team[i].member_id.size() > 0) {
      c = new circle(i, team[i].team_color, false, 1);
      scene.addItem(c);
    }
  }

  Q_EMIT detected_robots(tagAndRobot);

}

void Dialog::initialize_raceCar(const apriltags_ros::AprilTagDetectionArray &msg) {

  int team_id, capacity = 1;

  team_info t1(Qt::green,  cv::Point2f(classNr_x, classNr_yA), 0);
  team_info t2(Qt::blue,   cv::Point2f(classNr_x, classNr_yB), 1);
  team_info t3(Qt::yellow, cv::Point2f(classNr_x, classNr_yC), 2);
  team_info t4(Qt::cyan,   cv::Point2f(classNr_x, classNr_yD), 3);

  game_end = false;

  clk = new game_timer(cv::Point2f(classNr_x, classNr_yE + 20), game_clock);
  scene.addItem(clk);

  for(size_t i = 0; i < msg.detections.size(); i++) {
    if(tagAndRobot.find(msg.detections[i].id) == tagAndRobot.end() && robot.size() < max_number_robots) {
      id = robot.size();
      tagAndRobot[msg.detections[i].id] = robot.size();

      linearTransformation = M->linearTransf;
      offsetTranformation  = M->offsetTransf;

      for(size_t j = 0; j < team.size(); j++) {
        if(team[j].member_id.size() < capacity) {
          team_id = j;
          team[j].member_id.push_back(id);
          break;
        }
      }

      t = new tag(msg.detections[i], addr[id], robot_z[id], team_id);
      t->game = 2;
      scene.addItem(t);
    }
  }

  Q_EMIT detected_robots(tagAndRobot);

  for(size_t i = 0; i < team.size(); i++) {
    if(team[i].member_id.size() > 0) {
      c = new circle(i, team[i].team_color, false, 2);
      scene.addItem(c);
    }
  }

  rc = new racemap(cv::Point2f(borderOffset_x + wallWidth + 25, wallWidth + 50), true);
  scene.addItem(rc);

  rc = new racemap(cv::Point2f(borderOffset_x + wallWidth + 25, wallWidth + 50), false);
  scene.addItem(rc);
}

void Dialog::paintGoals() {
  Q_FOREACH(QGraphicsItem *item, scene.items()) {
    racemap *racemap_item = dynamic_cast<racemap *>(item);

    if(racemap_item != NULL)
      racemap_item->raceStarted = true;
  }

  raceHasStarted = true;
}

void Dialog::finish_game() {

  Q_FOREACH(QGraphicsItem *item, scene.items()) {
    wall *wall_item = dynamic_cast<wall *>(item);
    pallet *pallet_item = dynamic_cast<pallet *>(item);
    machine *machine_item = dynamic_cast<machine *>(item);
    person *person_item = dynamic_cast<person *>(item);
    game_timer *game_timer_item = dynamic_cast<game_timer *>(item);
    racemap *racemap_item = dynamic_cast<racemap *>(item);

    if(item->parentItem() == NULL) {
      if(wall_item != NULL) {
        if(wall_item->border == false)
          wall_item->delete_item();
      }
    }

    if(pallet_item != NULL)
      pallet_item->delete_item();

    if(machine_item != NULL)
      machine_item->delete_item();

    if(person_item != NULL)
      person_item->delete_item();

    if(game_timer_item != NULL)
      game_timer_item->delete_item();

    if(racemap_item != NULL)
      racemap_item->delete_item();
  }

  int offSetX = scene.width()/2 - 150, offSetY = scene.height()/2 + 50;
  QPointF p1, p2;

  p1.setX(offSetX); p1.setY(offSetY);
  p2.setX(offSetX + 300); p2.setY(offSetY);
  w = new wall(p1, p2, false, true, false);
  scene.addItem(w);

  p1.setX(offSetX); p1.setY(offSetY - 50);
  p2.setX(offSetX + 100); p2.setY(offSetY - 50);
  w = new wall(p1, p2, false, true, false);
  scene.addItem(w);

  p1.setX(offSetX + 100); p1.setY(offSetY - 100);
  p2.setX(offSetX + 200); p2.setY(offSetY - 100);
  w = new wall(p1, p2, false, true, false);
  scene.addItem(w);

  p1.setX(offSetX + 200); p1.setY(offSetY - 70);
  p2.setX(offSetX + 300); p2.setY(offSetY - 70);
  w = new wall(p1, p2, false, true, false);
  scene.addItem(w);



  p1.setX(offSetX); p1.setY(offSetY - 50);
  p2.setX(offSetX); p2.setY(offSetY);
  w = new wall(p1, p2, false, true, false);
  scene.addItem(w);

  p1.setX(offSetX + 100); p1.setY(offSetY - 100);
  p2.setX(offSetX + 100); p2.setY(offSetY - 50);
  w = new wall(p1, p2, false, true, false);
  scene.addItem(w);

  p1.setX(offSetX + 200); p1.setY(offSetY - 100);
  p2.setX(offSetX + 200); p2.setY(offSetY - 70);
  w = new wall(p1, p2, false, true, false);
  scene.addItem(w);

  p1.setX(offSetX + 300); p1.setY(offSetY - 70);
  p2.setX(offSetX + 300); p2.setY(offSetY);
  w = new wall(p1, p2, false, true, false);
  scene.addItem(w);


  for(size_t i = 0; i < team.size(); i++) {
    for(size_t j = 0; j < team[i].member_id.size(); j++) {
      switch(team[i].classification) {
        case 1:
          c = new circle(i, team[i].team_color, true, -1);
          scene.addItem(c);
          robot[team[i].member_id[j]].autonomous_drive = true;
          robot[team[i].member_id[j]].driving_pos = cv::Point2f(podium_first_pos_x, podium_first_pos_y);
        break;

        case 2:
          c = new circle(i, team[i].team_color, true, -1);
          scene.addItem(c);
          robot[team[i].member_id[j]].autonomous_drive = true;
          robot[team[i].member_id[j]].driving_pos = cv::Point2f(podium_second_pos_x, podium_second_pos_y);
        break;

        case 3:
          c = new circle(i, team[i].team_color, true, -1);
          scene.addItem(c);
          robot[team[i].member_id[j]].autonomous_drive = true;
          robot[team[i].member_id[j]].driving_pos = cv::Point2f(podium_third_pos_x, podium_third_pos_y);
        break;
      }
    }
  }
}



void Dialog::calibrateSIGNAL(int64_t threshold) {
  std_msgs::Int64 value;
  value.data = threshold;
  thresh_pub.publish(value);

  input_img = ros::topic::waitForMessage<sensor_msgs::Image>("/map", ros::Duration(10));
  if(input_img == NULL)
    return;

  if(M->performCalibration(input_img) == true) {
    M->cornersToPublish.data.clear();
    M->cornersToPublish.data.push_back(M->corners[0].x);
    M->cornersToPublish.data.push_back(M->corners[0].y);
    M->cornersToPublish.data.push_back(M->corners[1].x);
    M->cornersToPublish.data.push_back(M->corners[1].y);
    M->cornersToPublish.data.push_back(M->corners[2].x);
    M->cornersToPublish.data.push_back(M->corners[2].y);
    M->cornersToPublish.data.push_back(M->corners[3].x);
    M->cornersToPublish.data.push_back(M->corners[3].y);
    Q_EMIT calibDone(true, M->calib_image, M->corners);

  }
  else
    Q_EMIT calibDone(false, cv::Mat::eye(1,1,0), M->corners);

}

void Dialog::manually_calibration(std::vector<cv::Point2f> new_corners) {
  M->new_corners(new_corners);

  M->cornersToPublish.data.clear();
  M->cornersToPublish.data.push_back(M->corners[0].x);
  M->cornersToPublish.data.push_back(M->corners[0].y);
  M->cornersToPublish.data.push_back(M->corners[1].x);
  M->cornersToPublish.data.push_back(M->corners[1].y);
  M->cornersToPublish.data.push_back(M->corners[2].x);
  M->cornersToPublish.data.push_back(M->corners[2].y);
  M->cornersToPublish.data.push_back(M->corners[3].x);
  M->cornersToPublish.data.push_back(M->corners[3].y);
  Q_EMIT calibDone(true, M->calib_image, M->corners);
}

void Dialog::endCalibrationSIGNAL() {
  corners_pub.publish(M->cornersToPublish);
}

void Dialog::startSIGNAL() {
  if(gameStart == true || setupStart == true)
    ros::spinOnce();

  if(gameHasStarted.data != setupStart)
    corners_pub.publish(M->cornersToPublish);

  gameHasStarted.data = setupStart;
  gameStarted_pub.publish(gameHasStarted);
}

void Dialog::listener(const apriltags_ros::AprilTagDetectionArray& msg) {

  if(init_flag == false) {
    if(game == 0)
      initialize_pitank(msg);
    else if(game == 1)
      initialize_robotf(msg);
    else if(game == 2)
      initialize_raceCar(msg);
    init_flag = true;
  }

  if(finish_flag == true) {
    finish_game();
    finish_flag = false;
    game_end = true;
  }

  if(pause_flag == true)
    this->setUpdatesEnabled(false);
  else
    this->setUpdatesEnabled(true);

  for(size_t i = 0; i < msg.detections.size(); i++) {
    it = tagAndRobot.find(msg.detections[i].id);
    if(it != tagAndRobot.end()) {
      id = it->second;
      robot[id].centroid = t->parallax_correction(msg.detections[i].pose.pose.position.x * linearTransformation[0] + offsetTranformation[0],
                                                      msg.detections[i].pose.pose.position.y * linearTransformation[1] + offsetTranformation[1],
                                                      robot[id].height);

      robot[id].angle = 360 - t->normDegrees(atan2(2 * (msg.detections[i].pose.pose.orientation.x * msg.detections[i].pose.pose.orientation.y + msg.detections[i].pose.pose.orientation.z * msg.detections[i].pose.pose.orientation.w),
                                                 1 - 2*(msg.detections[i].pose.pose.orientation.y * msg.detections[i].pose.pose.orientation.y + msg.detections[i].pose.pose.orientation.z * msg.detections[i].pose.pose.orientation.z)) * 180 / CV_PI);
    }
  }

  if(game == 1)
    person_counter++;

  if(person_counter == 200 && game == 1 && game_end == false) {
    person_counter = 0;

    int range_x = (scene.width() - 30 - wallBHeight) - (borderOffset_x + 30 + wallBHeight) + 1;
    int x = rand() % range_x + (borderOffset_x + 30 + wallBHeight);

    h = new person(cv::Point2f(x, wallWidth * 2), false);
    scene.addItem(h);
  }

  this->convertRobotToMessage();
  this->publish_ui_state();
  ros::spinOnce();
}

int Dialog::createTrama(char addr1,  char addr0,  char data1,  char data2,  char options) {
  trama[0] = 0x7E;      //start delimiter
  trama[1] = 0x00;      //length
  trama[2] = 0x07;      //length
  trama[3] = 0x01;      //frame type
  trama[4] = 0x00;      //frame ID   ->0x00 no confirmation
  trama[5] = addr1;
  trama[6] = addr0;
  trama[7] = options;
  trama[8] = data1;
  trama[9] = data2;

  int pos = 8;
  char sum = 0x00;
  for(int i = 3; i < 10; i++)
    sum += trama[i];

  char checksum = (0xFF - sum);

  if((data1 == 0x7E) || (data1 == 0x7D) || (data1 == 0x11) || (data1 == 0x13)) {
    trama[pos] = 0x7D;  //escape byte
    pos++;
    trama[pos] = data1 ^ 0x20;
    pos++;
  }
  else {
    trama[pos] = data1;
    pos++;
  }

  if((data2 == 0x7E) || (data2 == 0x7D) || (data2 == 0x11) || (data2 == 0x13)) {
    trama[pos] = 0x7D; //escape byte
    pos++;
    trama[pos] = data2 ^ 0x20;
    pos++;
  }
  else {
    trama[pos] = data2;
    pos++;
  }

  if((checksum == 0x7E) || (checksum == 0x7D) || (checksum == 0x11) || (checksum == 0x13)) {
    trama[pos] = 0x7D;     //escape byte
    pos++;
    trama[pos] = checksum ^ 0x20;
    pos++;
  }
  else {
    trama[pos] = checksum;
    pos++;
  }

  return pos;
}

void Dialog::velJoystickA(const geometry_msgs::Twist speed) {

  if(gameStart == true && robot.size() > 0) {
    if(robot[0].autonomous_drive == false && robot[0].immobilized == false && pause_flag == false) {
      switch(robot[0].collisionStateVar) {
        case -1:
          robot[0].vel1 = speed.linear.x/6;
          robot[0].vel2 = speed.angular.z/2;

          if(robot[0].previous_vel > 0)
            robot[0].linear_direction = 1;
          else if(robot[0].previous_vel < 0)
            robot[0].linear_direction = -1;
        break;

        case 0:
          robot[0].vel1 = -127/6;
          robot[0].vel2 = 0;
        break;

        case 1:
          robot[0].vel1 = 127/6;
          robot[0].vel2 = 0;
        break;

        case 2:
        robot[0].vel2 = 0;
          if(robot[0].linear_direction == 1 && speed.linear.x >= 0)
            robot[0].vel1 = 0;
          else if(robot[0].linear_direction == 1 && speed.linear.x < 0) {
            robot[0].vel1 = speed.linear.x/6;
            robot[0].collisionStateVar = -1;
          }
          else if(robot[0].linear_direction == -1 && speed.linear.x <= 0)
            robot[0].vel1 = 0;
          else if(robot[0].linear_direction == -1 && speed.linear.x > 0) {
            robot[0].vel1 = speed.linear.x/6;
            robot[0].collisionStateVar = -1;
          }
        break;
      }

      if(speed.angular.z == 2) {
        robot[0].vel1 = robot[0].vel1 * 1.3;
        robot[0].vel2 = robot[0].vel2 * 1.3;
      }

      if(robot[0].collisionStateVar == -1 && speed.angular.z == 1 && bullet_interval[0].nsecsElapsed() > nsecs_between_bullets && game == 0) {
        b = new bullet(robot[0].centroid.x + pi_radius * cos(robot[0].angle * PI / 180), robot[0].centroid.y + pi_radius * sin(robot[0].angle * PI / 180), robot[0].angle);
        b->robot_sender = 0;
        scene.addItem(b);
        bullet_interval[0].restart();
      }

      if(speed.angular.z == 1 && game == 1 && robot[0].catch_pallet == false)
        robot[0].catch_pallet = true;
      else if(speed.angular.z == 1.5 && game == 1 && robot[0].catch_pallet == true)
        robot[0].catch_pallet = false;

      if(robot[0].slow_motion == true)
        robot[0].vel1 /= 2;

      if(game == 2)
        robot[0].vel1 *= 1.3;

      robot[0].previous_vel = speed.linear.x;
    }

    int turn_control_1 = -robot[0].vel2 + robot[0].vel2/2 + robot[0].vel2/3;
    int turn_control_2 = robot[0].vel2 - robot[0].vel2/2 - robot[0].vel2/3;
    int vel_control_1 = robot[0].vel1 - robot[0].vel1/3 - robot[0].vel2 + robot[0].vel2/2 + robot[0].vel2/4 + robot[0].vel2/6;
    int vel_control_2 = robot[0].vel1 - robot[0].vel1/3 + robot[0].vel2 - robot[0].vel2/2 - robot[0].vel2/4 - robot[0].vel2/6;

    int sizeTrama = 0;
    if(robot[0].vel1 < 5 && robot[0].vel1 > -5)
      sizeTrama = createTrama(robot[0].addr0, tagAndXbee[robot[0].tagId], (0x3F + turn_control_1), (0x3F + turn_control_2), 0x01);
    else {
      if((0x3F + vel_control_1) > 0b01111111)
        robot[0].vel1  = 0b01111111;
      else if((0x3F + vel_control_1 ) < 0)
        robot[0].vel1  = 0;
      if((0x3F + vel_control_2) > 0b01111111)
        robot[0].vel2  = 0b01111111;
      else if((0x3F + vel_control_2) < 0)
        robot[0].vel2  = 0;

      sizeTrama = createTrama(robot[0].addr0, tagAndXbee[robot[0].tagId], (0x3F + vel_control_1), (0x3F + vel_control_2), 0x01);
    }

    my_serial_stream.write((char*)trama, sizeTrama);
    this->convertRobotToMessage();
    ros::spinOnce();
  }
}

void Dialog::velJoystickB(const geometry_msgs::Twist speed) {

  if(gameStart == true && robot.size() > 1) {
    if(robot[1].autonomous_drive == false && robot[1].immobilized == false && pause_flag == false) {
      switch(robot[1].collisionStateVar) {
        case -1:
          robot[1].vel1 = speed.linear.x/6;
          robot[1].vel2 = speed.angular.z/2;

          if(robot[1].previous_vel > 0)
            robot[1].linear_direction = 1;
          else if(robot[1].previous_vel < 0)
            robot[1].linear_direction = -1;
        break;

        case 0:
          robot[1].vel1 = -127/6;
          robot[1].vel2 = 0;
        break;

        case 1:
          robot[1].vel1 = 127/6;
          robot[1].vel2 = 0;
        break;

        case 2:
        robot[1].vel2 = 0;
          if(robot[1].linear_direction == 1 && speed.linear.x >= 0)
            robot[1].vel1 = 0;
          else if(robot[1].linear_direction == 1 && speed.linear.x < 0) {
            robot[1].vel1 = speed.linear.x/6;
            robot[1].collisionStateVar = -1;
          }
          else if(robot[1].linear_direction == -1 && speed.linear.x <= 0)
            robot[1].vel1 = 0;
          else if(robot[1].linear_direction == -1 && speed.linear.x > 0) {
            robot[1].vel1 = speed.linear.x/6;
            robot[1].collisionStateVar = -1;
          }
        break;
      }


      if(speed.angular.z == 2) {
        robot[1].vel1 = robot[1].vel1 * 1.3;
        robot[1].vel2 = robot[1].vel2 * 1.3;
      }

      if(robot[1].collisionStateVar == -1 && speed.angular.z == 1 && bullet_interval[1].nsecsElapsed() > nsecs_between_bullets && game == 0) {
        b = new bullet(robot[1].centroid.x + pi_radius * cos(robot[1].angle * PI / 180), robot[1].centroid.y + pi_radius * sin(robot[1].angle * PI / 180), robot[1].angle);
        b->robot_sender = 1;
        scene.addItem(b);
        bullet_interval[1].restart();
      }

      if(speed.angular.z == 1 && game == 1 && robot[1].catch_pallet == false)
        robot[1].catch_pallet = true;
      else if(speed.angular.z == 1.5 && game == 1 && robot[1].catch_pallet == true)
        robot[1].catch_pallet = false;

      if(robot[1].slow_motion == true)
        robot[1].vel1 /= 2;

      if(game == 2)
        robot[1].vel1 *= 1.3;

      robot[1].previous_vel = speed.linear.x;
    }

    int turn_control_1 = -robot[1].vel2 + robot[1].vel2/2 + robot[1].vel2/3;
    int turn_control_2 = robot[1].vel2 - robot[1].vel2/2 - robot[1].vel2/3;
    int vel_control_1 = robot[1].vel1 - robot[1].vel1/3 - robot[1].vel2 + robot[1].vel2/2 + robot[1].vel2/4 + robot[1].vel2/6;
    int vel_control_2 = robot[1].vel1 - robot[1].vel1/3 + robot[1].vel2 - robot[1].vel2/2 - robot[1].vel2/4 - robot[1].vel2/6;

    int sizeTrama = 0;
    if(robot[1].vel1 < 5 && robot[1].vel1 > -5)
      sizeTrama = createTrama(robot[1].addr0, tagAndXbee[robot[1].tagId], (0x3F + turn_control_1), (0x3F + turn_control_2), 0x01);
    else {
      if((0x3F + vel_control_1) > 0b01111111)
        robot[1].vel1  = 0b01111111;
      else if((0x3F + vel_control_1 ) < 0)
        robot[1].vel1  = 0;
      if((0x3F + vel_control_2) > 0b01111111)
        robot[1].vel2  = 0b01111111;
      else if((0x3F + vel_control_2) < 0)
        robot[1].vel2  = 0;

      sizeTrama = createTrama(robot[1].addr0, tagAndXbee[robot[1].tagId], (0x3F + vel_control_1), (0x3F + vel_control_2), 0x01);
    }

    my_serial_stream.write((char*)trama, sizeTrama);
    this->convertRobotToMessage();
    ros::spinOnce();
  }
}

void Dialog::velJoystickC(const geometry_msgs::Twist speed) {

  if(gameStart == true && robot.size() > 2) {
    if(robot[2].autonomous_drive == false && robot[2].immobilized == false && pause_flag == false) {
      switch(robot[2].collisionStateVar) {
        case -1:
          robot[2].vel1 = speed.linear.x/6;
          robot[2].vel2 = speed.angular.z/2;

          if(robot[2].previous_vel > 0)
            robot[2].linear_direction = 1;
          else if(robot[2].previous_vel < 0)
            robot[2].linear_direction = -1;
        break;

        case 0:
          robot[2].vel1 = -127/6;
          robot[2].vel2 = 0;
        break;

        case 1:
          robot[2].vel1 = 127/6;
          robot[2].vel2 = 0;
        break;

        case 2:
        robot[2].vel2 = 0;
          if(robot[2].linear_direction == 1 && speed.linear.x >= 0)
            robot[2].vel1 = 0;
          else if(robot[2].linear_direction == 1 && speed.linear.x < 0) {
            robot[2].vel1 = speed.linear.x/6;
            robot[2].collisionStateVar = -1;
          }
          else if(robot[2].linear_direction == -1 && speed.linear.x <= 0)
            robot[2].vel1 = 0;
          else if(robot[2].linear_direction == -1 && speed.linear.x > 0) {
            robot[2].vel1 = speed.linear.x/6;
            robot[2].collisionStateVar = -1;
          }
        break;
      }


      if(speed.angular.z == 2) {
        robot[2].vel1 = robot[2].vel1 * 1.3;
        robot[2].vel2 = robot[2].vel2 * 1.3;
      }

      if(robot[2].collisionStateVar == -1 && speed.angular.z == 1 && bullet_interval[2].nsecsElapsed() > nsecs_between_bullets && game == 0) {
        b = new bullet(robot[2].centroid.x + pi_radius * cos(robot[2].angle * PI / 180), robot[2].centroid.y + pi_radius * sin(robot[2].angle * PI / 180), robot[2].angle);
        b->robot_sender = 2;
        scene.addItem(b);
        bullet_interval[2].restart();
      }

      if(speed.angular.z == 1 && game == 1 && robot[2].catch_pallet == false)
        robot[2].catch_pallet = true;
      else if(speed.angular.z == 1.5 && game == 1 && robot[2].catch_pallet == true)
        robot[2].catch_pallet = false;

      if(robot[2].slow_motion == true)
        robot[2].vel1 /= 2;

      if(game == 2)
        robot[2].vel1 *= 1.3;

      robot[2].previous_vel = speed.linear.x;
    }

    int turn_control_1 = -robot[2].vel2 + robot[2].vel2/2 + robot[2].vel2/3;
    int turn_control_2 = robot[2].vel2 - robot[2].vel2/2 - robot[2].vel2/3;
    int vel_control_1 = robot[2].vel1 - robot[2].vel1/3 - robot[2].vel2 + robot[2].vel2/2 + robot[2].vel2/4 + robot[2].vel2/6;
    int vel_control_2 = robot[2].vel1 - robot[2].vel1/3 + robot[2].vel2 - robot[2].vel2/2 - robot[2].vel2/4 - robot[2].vel2/6;

    int sizeTrama = 0;
    if(robot[2].vel1 < 5 && robot[2].vel1 > -5)
      sizeTrama = createTrama(robot[2].addr0, tagAndXbee[robot[2].tagId], (0x3F + turn_control_1), (0x3F + turn_control_2), 0x01);
    else {
      if((0x3F + vel_control_1) > 0b01111111)
        robot[2].vel1  = 0b01111111;
      else if((0x3F + vel_control_1 ) < 0)
        robot[2].vel1  = 0;
      if((0x3F + vel_control_2) > 0b01111111)
        robot[2].vel2  = 0b01111111;
      else if((0x3F + vel_control_2) < 0)
        robot[2].vel2  = 0;

      sizeTrama = createTrama(robot[2].addr0, tagAndXbee[robot[2].tagId], (0x3F + vel_control_1), (0x3F + vel_control_2), 0x01);
    }

    my_serial_stream.write((char*)trama, sizeTrama);
    this->convertRobotToMessage();
    ros::spinOnce();
  }
}

void Dialog::velJoystickD(const geometry_msgs::Twist speed) {

  if(gameStart == true && robot.size() > 3) {
    if(robot[3].autonomous_drive == false && robot[3].immobilized == false && pause_flag == false) {
      switch(robot[3].collisionStateVar) {
        case -1:
          robot[3].vel1 = speed.linear.x/6;
          robot[3].vel2 = speed.angular.z/2;

          if(robot[3].previous_vel > 0)
            robot[3].linear_direction = 1;
          else if(robot[3].previous_vel < 0)
            robot[3].linear_direction = -1;
        break;

        case 0:
          robot[3].vel1 = -127/6;
          robot[3].vel2 = 0;
        break;

        case 1:
          robot[3].vel1 = 127/6;
          robot[3].vel2 = 0;
        break;

        case 2:
        robot[3].vel2 = 0;
          if(robot[3].linear_direction == 1 && speed.linear.x >= 0)
            robot[3].vel1 = 0;
          else if(robot[3].linear_direction == 1 && speed.linear.x < 0) {
            robot[3].vel1 = speed.linear.x/6;
            robot[3].collisionStateVar = -1;
          }
          else if(robot[3].linear_direction == -1 && speed.linear.x <= 0)
            robot[3].vel1 = 0;
          else if(robot[3].linear_direction == -1 && speed.linear.x > 0) {
            robot[3].vel1 = speed.linear.x/6;
            robot[3].collisionStateVar = -1;
          }
        break;
      }

      if(speed.angular.z == 2) {
        robot[3].vel1 = robot[3].vel1 * 1.3;
        robot[3].vel2 = robot[3].vel2 * 1.3;

        robot[3].previous_vel = speed.linear.x;
      }

      if(robot[3].collisionStateVar == -1 && speed.angular.z == 1 && bullet_interval[3].nsecsElapsed() > nsecs_between_bullets && game == 0) {
        b = new bullet(robot[3].centroid.x + pi_radius * cos(robot[3].angle * PI / 180), robot[3].centroid.y + pi_radius * sin(robot[3].angle * PI / 180), robot[3].angle);
        b->robot_sender = 3;
        scene.addItem(b);
        bullet_interval[3].restart();
      }

      if(speed.angular.z == 1 && game == 1 && robot[3].catch_pallet == false)
        robot[3].catch_pallet = true;
      else if(speed.angular.z == 1.5 && game == 1 && robot[3].catch_pallet == true)
        robot[3].catch_pallet = false;

      if(robot[3].slow_motion == true)
        robot[3].vel1 /= 2;

      if(game == 2)
        robot[2].vel1 *= 1.3;
    }

    int turn_control_1 = -robot[3].vel2 + robot[3].vel2/2 + robot[3].vel2/3;
    int turn_control_2 = robot[3].vel2 - robot[3].vel2/2 - robot[3].vel2/3;
    int vel_control_1 = robot[3].vel1 - robot[3].vel1/3 - robot[3].vel2 + robot[3].vel2/2 + robot[3].vel2/4 + robot[3].vel2/6;
    int vel_control_2 = robot[3].vel1 - robot[3].vel1/3 + robot[3].vel2 - robot[3].vel2/2 - robot[3].vel2/4 - robot[3].vel2/6;

    int sizeTrama = 0;
    if(robot[3].vel1 < 5 && robot[3].vel1 > -5)
      sizeTrama = createTrama(robot[3].addr0, tagAndXbee[robot[3].tagId], (0x3F + turn_control_1), (0x3F + turn_control_2), 0x01);
    else {
      if((0x3F + vel_control_1) > 0b01111111)
        robot[3].vel1  = 0b01111111;
      else if((0x3F + vel_control_1 ) < 0)
        robot[3].vel1  = 0;
      if((0x3F + vel_control_2) > 0b01111111)
        robot[3].vel2  = 0b01111111;
      else if((0x3F + vel_control_2) < 0)
        robot[3].vel2  = 0;

      sizeTrama = createTrama(robot[3].addr0, tagAndXbee[robot[3].tagId], (0x3F + vel_control_1), (0x3F + vel_control_2), 0x01);
    }

    my_serial_stream.write((char*)trama, sizeTrama);
    this->convertRobotToMessage();
    ros::spinOnce();
  }
}

void Dialog::convertRobotToMessage(){
    r_array.robot.clear();
    for(int i=0;i<robot.size();i++){
        r_description.teamId = robot[i].teamId;
        r_description.tagId = robot[i].tagId;
        r_description.x = robot[i].centroid.x;
        r_description.y = robot[i].centroid.y;
        r_description.height = robot[i].height;
        r_description.angle = robot[i].angle;
        r_description.addr0 = robot[i].addr0;
        r_description.addr1 = robot[i].addr1;
        r_description.vel1 = robot[i].vel1;
        r_description.vel2 = robot[i].vel2;
        r_description.previous_vel = robot[i].previous_vel;
        r_description.collisionFlag = robot[i].collisionFlag;
        r_description.threadIsRunning = robot[i].threadIsRunning;
        r_description.collisionStateVar = robot[i].collisionStateVar;
        r_description.autonomous_drive = robot[i].autonomous_drive;
        r_description.immobilized = robot[i].immobilized;
        r_description.damage = robot[i].damage;
        r_description.kills = robot[i].kills;

        r_array.robot.push_back(r_description);
        i++;
    }
    robotDescription_pub.publish(r_array);
}

void Dialog::publish_ui_state(){
    ui_state.setupStart = setupStart;
    ui_state.gameStart = gameStart;
    ui_state.teamGame = teamGame;
    ui_state.paused = paused;
    ui_state.aiGame = aiGame;
    ui_state.seconds = game_clock.minute() * 60 + game_clock.second();

    uiState_pub.publish(ui_state);
}


Dialog::~Dialog() {
  delete ui;
}
