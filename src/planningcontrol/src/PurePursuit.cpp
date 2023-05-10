#include "planningcontrol/PurePursuit.h"


// float getDist(std::array<float, 2> point1, std::array<float, 2> point2){
//     float dist = sqrt(pow(point2[0] - point1[0],2) + pow(point2[1] - point1[1],2));
//     return dist;
// }

void PurePursuit::getPath(){
    std::vector<std::array<float, 2>> path; 
    std::ifstream path_file("/home/murraiya/Downloads/Path.txt"); //original txt file to read

    // if(!path_file.is_open()){
    //     perror("failed file opening\nexit program");
    //     exit(1);
    // }

    while(1){
        if(path_file.eof()){break;} // path_file.close(); }
        std::array<float, 2> point;
        path_file>>point[0]>>point[1];
        // std::cout<<point[0]<<point[1]<<'\n';
        path.push_back(point);
    }
    std::cout<<"reading file success"<<'\n';
    this->path=path;
};

geometry_msgs::Point[] PurePursuit::pubPath(){
  	// ros::Publisher path_point_pub = nh.advertise<geometry_msgs::Point>("/path_point", 1);
    geometry_msgs::Point[] point;
    point[0].x=1;
    point[0].y=2;
    point[0].z=3;
    point[1].x=4;
    point[1].y=5;
    point[1].z=6;
    point[2].x=7;
    point[2].y=8;
    point[2].z=9;

    return point;
}



float PurePursuit::getDist(std::array<float, 2> point1, std::array<float, 2> point2){
    float dist = sqrt(pow(point2[0] - point1[0],2) + pow(point2[1] - point1[1],2));
    return dist;
}

float PurePursuit::getDistToDest(std::array<float, 2> curr_pos){
    std::array<float, 2> dest =path[path.size()-1];
    float dist = sqrt(pow(dest[0] - curr_pos[0],2) + pow(dest[1] - curr_pos[1],2));
    return dist;
}

std::array<float, 2> PurePursuit::getLookaheadPoint(std::array<float, 2> curr_pos, int lookahead_distance){
    float dist;

    for (int i=lookahead_point_idx;i<path.size();i++){
        dist = getDist(path[i], curr_pos);
        if (dist > lookahead_distance){
            lookahead_point_idx=i;
            break;
        }
    }

    std::array<float, 2> lookahead_point = path[lookahead_point_idx];
    return lookahead_point;
}

float PurePursuit::getSteeringAngle(std::array<float, 2> curr_pos, std::array<float, 2> lookahead_point, float heading){
    float Ld, alpha, steering_angle;

    Ld = getDist(lookahead_point, curr_pos);
    alpha = atan2((lookahead_point[1] - curr_pos[1]), (lookahead_point[0] - curr_pos[0])) - heading;
    steering_angle = atan2(2 * L * sin(alpha), Ld);
    return steering_angle;
}
