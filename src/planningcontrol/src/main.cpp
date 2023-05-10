#include <ros/ros.h>
#include <iostream>
#include <array>
#include <cmath>
#include "planningcontrol/readPath.h"
#include "planningcontrol/PurePersuit.h"


const float pi{3.141592};
const float L{0.5};


float getDist(std::array<float, 2> point1, std::array<float, 2> point2){
    float dist = sqrt(pow(point2[0] - point1[0],2) + pow(point2[1] - point1[1],2));
    return dist;
}

std::array<float, 2> Drive(std::array<float, 2> curr_pos, int curr_spd, float heading){
    curr_pos[0] += curr_spd * cos(heading);
    curr_pos[1] += curr_spd * sin(heading);
    return curr_pos;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_control_node");
    ros::NodeHandle nh;
   	ros::Publisher pth_point_pub = nh.advertise<std_msgs::Float32MultiArray>("/path_point", 10);

    std::vector<std::array<float, 2>> path = readPath();

    float heading{atan2(1,0)}, steering_angle;
    int curr_spd{1}, lookahead_distance{curr_spd * 2};
    std::array<float, 2> curr_pos = {0,0}, lookahead_point;
    PurePursuit pure_pursuit;

    while (true){
        std::cout<<"current_position : ";
        for(int j=0;j<2;j++){
            std::cout<<curr_pos[j]<<'\t';
        }
        std::cout<<"heading : "<<(heading*180/pi)<<'\t';
        float dist = getDist(curr_pos, path[path.size()]);
        std::cout<<"dist : "<<dist<<std::endl;
        if (dist < 0.5) break;        

        lookahead_point = pure_pursuit.getLookaheadPoint(curr_pos, lookahead_distance);
        steering_angle = pure_pursuit.getSteeringAngle(curr_pos, lookahead_point, heading);
        heading += curr_spd * tan(steering_angle) / L;
        curr_pos = Drive(curr_pos, curr_spd, heading);
    }
    return 0;
}