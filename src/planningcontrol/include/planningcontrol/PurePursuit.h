#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include <ros/ros.h>
#include <iostream>
#include <array>
#include <cmath>
#include <string.h>

float getDist(std::array<float, 2> point1, std::array<float, 2> point2);


class PurePursuit
{
public:
    std::array<float, 2> getLookaheadPoint(std::array<float, 2> curr_pos, int lookahead_distance);
    float getSteeringAngle(std::array<float, 2> curr_pos, std::array<float, 2> lookahead_point, float heading);
};

#endif //PURE_PURSUIT_H