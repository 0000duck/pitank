#include "../include/ai/control.hpp"

void uiStateReceived(const game_state::UIState& msg){

}

void robotDescriptionReceived(const game_state::RobotDescriptionArray& msg){

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

void GoToXY(int id, ){


}

void *decision_makingA(void *id){

    while(1){



    }
    return NULL;
}
