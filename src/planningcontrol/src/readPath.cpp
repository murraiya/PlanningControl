#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <array>
#include <vector>

std::vector<std::array<float,2>>& readPath(){

    std::vector<std::array<float, 2>> path; //to return
    std::ifstream path_file; //original txt file to read
    path_file.open("/home/murraiya/Downloads/Path.txt");
    if(!path_file.is_open()){
        perror("failed file opening\nexit program");
        exit(1);
    }

    while(!path_file.eof()){
        char buf[200];
        std::array<float, 2> point;
        path_file.getline(buf, 200);
        point.at(0)=atof(strtok(buf, " "));
        point.at(1)=atof(strtok(NULL, " "));
        std::cout<<point.at(0)<<point.at(1)<<'\n';
        path.push_back(point);
    }
    
    return path;
}