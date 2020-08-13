//
// Created by wangpei on 2020/8/12.
//
#include <iostream>
#include <chrono>
#include "BackProject.h"

namespace BP{
    BackProject::BackProject(const Eigen::Vector2f& pt_pixel,
                             const Eigen::Matrix3f& K,
                             const Eigen::Matrix3f& R_c_w,
                             const Eigen::Vector3f& T_c_w)
            : pt_pixel_(pt_pixel)
            , K_(K)
            , R_c_w_(R_c_w)
            , T_c_w_(T_c_w)
    {
        pt_p_u_ = pt_pixel_(0,0);
        pt_p_v_ = pt_pixel_(1,0);
        //这里本来想让像素坐标也是齐次形式的，【u,v,1】,但是怕到时候读起来难以理解，改成了2行的向量
        fx_ = K_(0,0);
        fy_ = K_(1,1);
        cx_ = K_(0,2);
        cy_ = K_(1,2);

        R_w_c_ = R_c_w_.inverse();
        T_w_c_ = -T_c_w_;

    }

    Eigen::Vector3f BackProject::pixel2cam() {
        this->pt_c_x_ = (pt_p_u_ - cx_)/fx_;
        this->pt_c_y_ = (pt_p_v_ - cy_)/fy_;
        this->pt_c_z_ = 1;
        return Eigen::Vector3f{
                this->pt_c_x_,
                this->pt_c_y_,
                this->pt_c_z_
        };
    }

    bool BackProject::getTwoPtsInCam() {
        this->pt_cam_ = pixel2cam();

        return true;
    }

    bool BackProject::getTwoPtsInWld() {
        pt_wld_ = R_w_c_*(pt_cam_ - T_c_w_);
        PT_WLD_O_ = R_w_c_*(PT_CAM_O_ -T_c_w_);

        return true;
    }

    Eigen::Vector3f BackProject::calcuTarget() {
        if(getTwoPtsInCam()&&getTwoPtsInWld()){
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            std::cout<<K_<<std::endl;
            std::cout<<"pt_cam"<<pt_cam_<<std::endl;
            std::cout<<"PT_CAM_O"<<PT_CAM_O_<<std::endl;
            std::cout<<"pt_wld_:"<<pt_wld_<<std::endl;
            std::cout<<"PT_WLD_O:"<<PT_WLD_O_<<std::endl;
            VL_ = pt_wld_ - PT_WLD_O_;
            v1 = VL_(0,0);
            v2 = VL_(1,0);
            v3 = VL_(2,0);
            std::cout<<"**VL:"<<VL_<<std::endl;

            m_ = PT_WLD_O_;
            m1 = m_(0,0);
            m2 = m_(1,0);
            m3 = m_(2,0);
            std::cout<<"**m_:"<<m_<<std::endl;

            n1 = n_(0,0);
            n2 = n_(1,0);
            n3 = n_(2,0);
            std::cout<<"n_:"<<n_<<std::endl;

            vp1 = VP_(0,0);
            vp2 = VP_(1,0);
            vp3 = VP_(2,0);
            std::cout<<"VP_:"<<VP_<<std::endl;
            t =(n1-m1)*vp1 +(n2 -m2)*vp2 +(n3-m3)*vp3;
            if(vp1*v1 + vp2*v2 + vp3*v3 == 0){
                std::cout<<"divide within o!"<<std::endl;
            }
            t /= (vp1*v1 + vp2*v2 + vp3*v3);
            if(t == ((n3-m3)/v3)){
                std::cout<<"t is as expected:"<<t<<std::endl;
            }

            std::cout<<"x:"<<m1 + v1*t<<" y:"<<m2 + v2*t<<" z:"<<z0_<<std::endl;
            pt_target_ = Eigen::Vector3f{m1 + v1*t, m2 + v2*t ,z0_};
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            std::chrono::duration<double>  time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            std::cout<<"calculate pts target cost time:" << time_used.count()<<"scds."<<std::endl;
            return pt_target_;
        } else if(!getTwoPtsInCam()){
            std::cout<<"cannot get pts in cam correctly!"<<std::endl;
        } else if(!getTwoPtsInWld()){
            std::cout<<"cannot get pts in world correctly!"<<std::endl;
        }
    }

}