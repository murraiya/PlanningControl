#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <array>
#include <cmath>
#include "planningcontrol/PurePursuit.h"

const float pi{3.141592};

std::array<float, 2> Drive(std::array<float, 2> curr_pos, int curr_spd, float heading){
    curr_pos[0] += curr_spd * cos(heading);
    curr_pos[1] += curr_spd * sin(heading);
    return curr_pos;
}


void subCallback(const geometry_msgs::Point& point){
    std::cout<<"in subcallback"<<'\n';
    std::cout<<point.x<<point.y<<point.z<<'\n';
    std::cout<<"after msg cout"<<'\n';
}

void sub(){
    ros::NodeHandle n;
    std::cout<<"called sub()"<<'\n';

    ros::Subscriber sub=n.subscribe("/path_point", 1, subCallback);
}
    


int main(int argc, char** argv){
    ros::init(argc, argv, "planning_control_node");
    ros::NodeHandle nh;
   	ros::Publisher path_point_pub = nh.advertise<geometry_msgs::Point[]>("/path_point", 1);
   	ros::Publisher path_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/path_points", 1);
    ros::Publisher curr_point_pub = nh.advertise<geometry_msgs::Point[]>("/curr_point", 1);

    float heading{atan2(1,0)}, steering_angle=0;
    float curr_spd{10.0};
    float lookahead_distance{(float)curr_spd * 0.2 + 3.0};
    std::array<float, 2> curr_pos = {0.0, 5.0}, lookahead_point;

    PurePursuit pure_pursuit;
    pure_pursuit.getPath();
    path_point_pub.publish(pure_pursuit.pubPath());

    sub();


    while (ros::ok()){
        // std::cout<<"current_position : "<<curr_pos[0]<<'\t'<<curr_pos[1]<<'\t';
        // std::cout<<"heading : "<<(heading*180/pi)<<'\t';
        float dist_to_dest = pure_pursuit.getDistToDest(curr_pos);
        // std::cout<<"dist_to_dest : "<<dist_to_dest<<'\n';
        if (dist_to_dest < 0.5) break;

        lookahead_point = pure_pursuit.getLookaheadPoint(curr_pos, lookahead_distance);
        steering_angle = pure_pursuit.getSteeringAngle(curr_pos, lookahead_point, heading);
        heading += curr_spd * tan(steering_angle) / L;
        curr_pos = Drive(curr_pos, curr_spd, heading);
    }
    for(;;);
    return 0;
}