#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include <ros/ros.h>
#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <string.h>
#include <geometry_msgs/Point.h>


const float L{1.04};

class PurePursuit
{
private:
    int lookahead_point_idx=0;
    std::vector<std::array<float, 2>> path;
public:
    void getPath();
    geometry_msgs::Point[] pubPath();
    float getDist(std::array<float, 2> point1, std::array<float, 2> point2);
    float getDistToDest(std::array<float, 2> curr_pos);
    std::array<float, 2> getLookaheadPoint(std::array<float, 2> curr_pos, int lookahead_distance);
    float getSteeringAngle(std::array<float, 2> curr_pos, std::array<float, 2> lookahead_point, float heading);
};

#endif //PUREPURSUIT_H