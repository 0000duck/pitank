#include "../include/ai/control.hpp"

void uiStateReceived(const game_state::UIState& msg){
    uistate.aiGame = msg.aiGame;
    uistate.gameStart = msg.gameStart;
    uistate.paused = msg.paused;
    uistate.seconds = msg.seconds;
    uistate.setupStart = msg.setupStart;
    uistate.teamGame = msg.teamGame;
}

void robotDescriptionReceived(const game_state::RobotDescriptionArray& msg){
    robot_info r;
    for (int i=0;i<msg.robot.size();i++){
        r.addr0 = msg.robot[i].addr0;
        r.addr1 = msg.robot[i].addr1;
        r.angle = msg.robot[i].angle;
        r.autonomous_drive = msg.robot[i].autonomous_drive;
        r.collisionFlag = msg.robot[i].collisionFlag;
        r.collisionStateVar = msg.robot[i].collisionStateVar;
        r.damage = msg.robot[i].damage;
        r.height = msg.robot[i].height;
        r.immobilized = msg.robot[i].immobilized;
        r.kills = msg.robot[i].kills;
        r.tagId = msg.robot[i].tagId;
        r.teamId = msg.robot[i].teamId;
        r.threadIsRunning = msg.robot[i].threadIsRunning;
        r.vel1 = msg.robot[i].vel1;
        r.vel2 = msg.robot[i].vel2;
        r.x = msg.robot[i].x;
        r.y = msg.robot[i].y;

        robot.push_back(r);
    }
}

int main(int argc, char **argv){

        ros::init(argc, argv, "");
        ros::NodeHandle nh;

        vel_pub[0] = nh->advertise<geometry_msgs::Twist>("/cmd_velB", 1);
        uistate_sub = nh->subscribe("/ui_state", 1000, &uiStateReceived);
        robot_sub = nh->subscribe("/robots_description", 1000, &robotDescriptionReceived);

        pthread_create(&t[0], NULL, decision_makingA, &id);
        pthread_detach(t[0]);
}

void move_forward(int id){
    vel[id].linear.x = 127;
}

void move_back(int id){
    vel[id].linear.x = -127;
}

void move_left(int id){
    vel[id].angular.z = 127;
}

void move_right(int id){
    vel[id].angular.z = -127;
}

void shoot(int id){
    vel[id].angular.z = 1;
}

void turbo(int id){
    vel[id].angular.z = 2;
}

void GoToXY(int id, int x, int y){
    int state = 0;

}

void *decision_makingA(void *id){

    while(1){

        GoToXY(id, 10, 10);

    }
    return NULL;
}
