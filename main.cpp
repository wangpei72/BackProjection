#include <iostream>
#include "BackProject.h"

int main() {
    Eigen::Matrix3f K;
    K<<520.9, 0, 325.1,
    0, 521.0, 249.7,
    0, 0, 1;
    Eigen::Vector2f pts_pixel;
    pts_pixel<<10,100;

    Eigen::Matrix3f R_c_w;
    R_c_w<<1,0,0,
    0,1,0,
    0,0,1;

    Eigen::Vector3f T_c_w;
    T_c_w<<0,0,22;

    Eigen::Vector3f pts_target;
    BP::BackProject backProject(pts_pixel,K,R_c_w, T_c_w);
    pts_target = backProject.calcuTarget();
    std::cout<<"pts_target is (x,y,z=0):"<<pts_target<<std::endl;

    return 0;
}
