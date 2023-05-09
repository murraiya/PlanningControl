#include <ros/ros.h>
#include <iostream>
#include <array>
#include <cmath>

const float pi{3.141592};
const float L{0.5};
std::array<std::array<float, 2>, 13> path = {{
    {0,0},
    {0,1},
    {0,2},
    {0,3},
    {0,4},
    {0.2,4.98},
    {0.57,5.91},
    {1.07,6.77},
    {1.73,7.52},
    {2.47,8.2},
    {3.23,8.85},
    {4.03,9.45},
    {4.92,9.9}
}};

float getDist(std::array<float, 2> point1, std::array<float, 2> point2){
    float dist = sqrt(pow(point2[0] - point1[0],2) + pow(point2[1] - point1[1],2));
    return dist;
}

std::array<float, 2> Drive(std::array<float, 2> curr_pos, int curr_spd, float heading){
    curr_pos[0] += curr_spd * cos(heading);
    curr_pos[1] += curr_spd * sin(heading);
    return curr_pos;
}

class PurePursuit
{
public:
    std::array<float, 2> getLookaheadPoint(std::array<float, 2> curr_pos, int lookahead_distance){
        float min_dist{99999}, dist;
        int close_index;

        for (int i=0;i<13;i++){
            dist = getDist(path[i], curr_pos);
            if (dist < min_dist){
                min_dist = dist;
                close_index = i;
            }
        }

        std::array<float, 2> lookahead_point = path[close_index+lookahead_distance];
        return lookahead_point;
    }

    float getSteeringAngle(std::array<float, 2> curr_pos, std::array<float, 2> lookahead_point, float heading){
        float Ld, alpha, steering_angle;

        Ld = getDist(lookahead_point, curr_pos);
        alpha = atan2((lookahead_point[1] - curr_pos[1]), (lookahead_point[0] - curr_pos[0])) - heading;
        steering_angle = atan2(2 * L * sin(alpha), Ld);
        return steering_angle;
    }
};

int main(){
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
        float dist = getDist(curr_pos, path[12]);
        std::cout<<"dist : "<<dist<<std::endl;
        if (dist < 0.5) break;        

        lookahead_point = pure_pursuit.getLookaheadPoint(curr_pos, lookahead_distance);
        steering_angle = pure_pursuit.getSteeringAngle(curr_pos, lookahead_point, heading);
        heading += curr_spd * tan(steering_angle) / L;
        curr_pos = Drive(curr_pos, curr_spd, heading);
    }
    return 0;
}