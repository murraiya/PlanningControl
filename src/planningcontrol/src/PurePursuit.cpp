#include "PurePersuit.h"



float getDist(std::array<float, 2> point1, std::array<float, 2> point2){
    float dist = sqrt(pow(point2[0] - point1[0],2) + pow(point2[1] - point1[1],2));
    return dist;
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
