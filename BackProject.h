//
// Created by wangpei on 2020/8/12.
//

#ifndef UNTITLED_BACKPROJECT_H
#define UNTITLED_BACKPROJECT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2//highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace BP {
    class BackProject {
    private:
        //首先定义我们的二维目标点，也就是像素坐标系下的二位点（u,v）
        //矩阵形式 和 数值形式我们都来一遍 矩阵方便传参 数值方便我计算
        Eigen::Vector2d pt_pixel_;
        double pt_p_u_;
        double pt_p_v_;
        std::vector<Eigen::Vector2d> lotsof_pts_pixel_;

        //再定义相机坐标系下的坐标矩阵以及数值
        //以及相机坐标系的原点
        Eigen::Vector3d pt_cam_;
        double pt_c_x_;
        double pt_c_y_;
        double pt_c_z_;
        const Eigen::Vector3d PT_CAM_O_{0.0, 0.0, 0.0};
        std::vector<Eigen::Vector3d> lotsof_pts_cam_;

        Eigen::Vector3d pt_mid_;
        double pt_m_x_;
        double pt_m_y_;
        double pt_m_z_;
        const Eigen::Vector3d PT_MID_O_{0.0,0.0,0.0};
        std::vector<Eigen::Vector3d> lotsof_pts_mid_;

        //相机内参矩阵
        Eigen::Matrix3d K_;
//        int width_;
//        int height_;
        double fx_;
        double fy_;
        double cx_;
        double cy_;

        //定义从世界到相机的变换矩阵
        Eigen::Matrix3d R_c_w_;
        Eigen::Vector3d T_c_w_;

        //再来定义从相机到世界的变换矩阵
        //要知道他们就是个相反的关系
        Eigen::Matrix3d R_w_c_;
        Eigen::Vector3d T_w_c_;

        //欧拉角形式
        double roll_;
        double pitch_;
        double yaw_;
        Eigen::Vector3d test_euler;
        Eigen::Vector3d euler_;
        Eigen::Vector3d position_;
        Eigen::Matrix<double,6,1> pose_;

        //四元数形式
        Eigen::Quaterniond qt_;
        Eigen::AngleAxisd rota_vec_;

        //定义世界坐标系下 原点和方向点的转换后坐标
        Eigen::Vector3d pt_wld_;
        double pt_w_x_;
        double pt_w_y_;
        double pt_w_z_;
        Eigen::Vector3d PT_WLD_O_;//传参时候拿到初始化

        //定义世界坐标系下选定的平面
        const double z0_{0};

        //定义我们的世界坐标系下三维目标点坐标
        Eigen::Vector3d pt_target_;

        //定义我们建模数学模型的所有参数
        Eigen::Vector3d VL_;
        const Eigen::Vector3d VP_{0,0,1};
        Eigen::Vector3d m_;
        const Eigen::Vector3d n_{0,0,z0_};
        double m1, m2, m3;
        double v1, v2, v3;
        double mode;
        double n1, n2, n3;
        double vp1, vp2, vp3;
        double t;

        bool use_euler;
        bool use_qt;
        cv::Mat ground_;
    public:
        BackProject(){}

        BackProject(const Eigen::Vector2d& pt_pixel,
                    const Eigen::Matrix3d& K,
                    const Eigen::Matrix3d  R_c_w,
                    const Eigen::Vector3d& T_c_w);
        BackProject(const Eigen::Vector2d& pt_pixel,
                    const Eigen::Matrix3d& K,
                    const Eigen::Matrix<double,6,1>& pose);
        BackProject(const Eigen::Vector2d& pt_pixel,
                    const Eigen::Matrix3d& K,
                    const Eigen::Matrix<double,6,1>& pose,
                    const Eigen::Quaterniond& qt);
        ~BackProject()= default;

        Eigen::Vector3d pixel2cam();
        //double roll-z,double pitch-x,double yaw-y
        Eigen::Matrix3d euler2rota();
        Eigen::Matrix3d qt2rota();
        bool getRotaUsingEuler();
        bool getRotaUsingQt();
        bool getTwoPtsInCam();

        bool getTwoPtsInWld();

        Eigen::Vector3d calcuTarget();
        Eigen::Vector3d calcuTargetUsingQt();
        void showTargetInWLD();
    };


#endif //UNTITLED_BACKPROJECT_H
}