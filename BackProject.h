//
// Created by wangpei on 2020/8/12.
//

#ifndef UNTITLED_BACKPROJECT_H
#define UNTITLED_BACKPROJECT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace BP {
    class BackProject {
    private:
        //首先定义我们的二维目标点，也就是像素坐标系下的二位点（u,v）
        //矩阵形式 和 数值形式我们都来一遍 矩阵方便传参 数值方便我计算
        Eigen::Vector2f pt_pixel_;
        float pt_p_u_;
        float pt_p_v_;

        //再定义相机坐标系下的坐标矩阵以及数值
        //以及相机坐标系的原点
        Eigen::Vector3f pt_cam_;
        float pt_c_x_;
        float pt_c_y_;
        float pt_c_z_;
        const Eigen::Vector3f PT_CAM_O_{0, 0, 0};

        //相机内参矩阵
        Eigen::Matrix3f K_;
//        int width_;
//        int height_;
        float fx_;
        float fy_;
        float cx_;
        float cy_;

        //定义从世界到相机的变换矩阵
        Eigen::Matrix3f R_c_w_;
        Eigen::Vector3f T_c_w_;

        //再来定义从相机到世界的变换矩阵
        //要知道他们就是个相反的关系
        Eigen::Matrix3f R_w_c_;
        Eigen::Vector3f T_w_c_;

        //定义世界坐标系下 原点和方向点的转换后坐标
        Eigen::Vector3f pt_wld_;
        float pt_w_x_;
        float pt_w_y_;
        float pt_w_z_;
        Eigen::Vector3f PT_WLD_O_;//传参时候拿到初始化

        //定义世界坐标系下选定的平面
        const float z0_{0};

        //定义我们的世界坐标系下三维目标点坐标
        Eigen::Vector3f pt_target_;

        //定义我们建模数学模型的所有参数
        Eigen::Vector3f VL_;
        const Eigen::Vector3f VP_{0,0,1};
        Eigen::Vector3f m_;
        const Eigen::Vector3f n_{0,0,z0_};
        float m1, m2, m3;
        float v1, v2, v3;
        float n1, n2, n3;
        float vp1, vp2, vp3;
        float t;

    public:
        BackProject() {}

        BackProject(const Eigen::Vector2f& pt_pixel,
                    const Eigen::Matrix3f& K,
                    const Eigen::Matrix3f& R_c_w,
                    const Eigen::Vector3f& T_c_w);

        Eigen::Vector3f pixel2cam();

        bool getTwoPtsInCam();

        bool getTwoPtsInWld();

        Eigen::Vector3f calcuTarget();

    };


#endif //UNTITLED_BACKPROJECT_H
}