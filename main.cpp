#include <iostream>
#include "BackProject.h"

int main() {
    Eigen::Matrix3d K;
    K<<762.7249337622711, 0.0, 640.5, 0.0, 762.7249337622711, 360.5, 0.0, 0.0, 1.0;
    Eigen::Vector2d pts_pixel;
    pts_pixel<<656,668;
    Eigen::Matrix<double,6,1> pose;
    pose <<4.04 ,2.24 ,1.808 ,0 ,0.25, -2.355;
    Eigen::Quaterniond qt(0.380245018457,0.115155918721,0.0477797397263,-0.916444180948);
//    Eigen::Matrix3d R_c_w;
//    R_c_w<<1,0,0,
//    0,1,0,
//    0,0,1;

//    Eigen::Vector3d T_c_w;
//    T_c_w<<0,0,22;


//    rotation:四元数
//    x: 0.115155918721
//    y: 0.0477797397263
//    z: -0.916444180948
//    w: 0.380245018457

    Eigen::Vector3d pts_target;
//    BP::BackProject backProject(pts_pixel,K,R_c_w, T_c_w);
//    BP::BackProject backProject(pts_pixel,K,pose);
    BP::BackProject backProject(pts_pixel,K,pose,qt);
//    pts_target = backProject.calcuTarget();
    pts_target = backProject.calcuTargetUsingQt();
    std::cout<<"pts_target is (x,y,z=0):"<<pts_target<<std::endl;
    backProject.showTargetInWLD();
    return 0;
}
