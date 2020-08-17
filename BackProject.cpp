//
// Created by wangpei on 2020/8/12.
//
#include <iostream>
#include <chrono>
#include "BackProject.h"

namespace BP{
    BackProject::BackProject(const Eigen::Vector2d & pt_pixel,
                             const Eigen::Matrix3d & K,
                             const Eigen::Matrix3d  R_c_w,
                             const Eigen::Vector3d & T_c_w)
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
        use_euler = false;
    }

    BackProject::BackProject(const Eigen::Vector2d& pt_pixel,
                             const Eigen::Matrix3d& K,
                             const Eigen::Matrix<double,6,1>& pose)
            : pt_pixel_(pt_pixel)
            , K_(K)
            , pose_(pose)
    {
        pt_p_u_ = pt_pixel_(0,0);
        pt_p_v_ = pt_pixel_(1,0);
        //这里本来想让像素坐标也是齐次形式的，【u,v,1】,但是怕到时候读起来难以理解，改成了2行的向量
        fx_ = K_(0,0);
        fy_ = K_(1,1);
        cx_ = K_(0,2);
        cy_ = K_(1,2);

        euler_[0] = pose[5];
        euler_[1] = pose[4];
        euler_[2] = pose[3];
        position_ = pose_.block<3,1>(0,0);
        T_c_w_ = position_;
//        R_w_c_ = R_c_w_.inverse();
        T_w_c_ = -T_c_w_;
        use_euler =true;

    }

    BackProject::BackProject(const Eigen::Vector2d &pt_pixel,
            const Eigen::Matrix3d &K,
            const Eigen::Matrix<double, 6, 1> &pose,
            const Eigen::Quaterniond &qt)
            :pt_pixel_(pt_pixel)
            , K_(K)
            ,pose_(pose)
            , qt_(qt)
            {
                pt_p_u_ = pt_pixel_(0,0);
                pt_p_v_ = pt_pixel_(1,0);
                //这里本来想让像素坐标也是齐次形式的，【u,v,1】,但是怕到时候读起来难以理解，改成了2行的向量
                fx_ = K_(0,0);
                fy_ = K_(1,1);
                cx_ = K_(0,2);
                cy_ = K_(1,2);
                position_ = pose_.block<3,1>(0,0);
                T_w_c_ = position_;
                T_c_w_ = -T_w_c_;
                use_qt = true;
    }

    Eigen::Vector3d BackProject::pixel2cam() {
        this->pt_c_z_ = 1;
        this->pt_c_x_ = pt_c_z_*(pt_p_u_ - cx_)/fx_;
        this->pt_c_y_ = pt_c_z_*(pt_p_v_ - cy_)/fy_;

        pt_m_x_ = pt_c_z_;
        pt_m_y_ = -pt_c_x_;
        pt_m_z_ = -pt_c_y_;
//        return Eigen::Vector3d{
//                this->pt_c_x_,
//                this->pt_c_y_,
//                this->pt_c_z_
//        };
        pt_mid_ <<pt_m_x_,pt_m_y_,pt_m_z_;
        return Eigen::Vector3d{
                this->pt_m_x_,
                this->pt_m_y_,
                this->pt_m_z_
        };
    }

    //roll-x pitch-y yaw-z euler_(yaw,pitch,roll)
    Eigen::Matrix3d BackProject::euler2rota() {
        //注意这里取了负值，直接求得的就是相机到世界的R_w_c,这是为了解决之前矩阵求逆出现nan的问题
        euler_[0] = -euler_[0];
        euler_[1] = -euler_[1];
        euler_[2] = -euler_[2];
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_(2),Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_(1),Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_(0),Eigen::Vector3d::UnitZ()));
//        Eigen::Matrix3d R_z,R_x,R_y ;
//        R_x << 1,       0,              0,
//               0,       cos(euler_[0]),   -sin(euler_[0]),
//               0,       sin(euler_[0]),   cos(euler_[0]);
//        R_y <<cos(euler_[1]),    0,      sin(euler_[1]),
//                0,               1,      0,
//                -sin(euler_[1]),   0,      cos(euler_[1]);
//        R_z <<  cos(euler_[2]),    -sin(euler_[2]),      0,
//                sin(euler_[2]),    cos(euler_[2]),       0,
//                0,               0,                  1;
//        R_c_w_ = R_x*R_y*R_z;
//        R_c_w_ = yawAngle*pitchAngle*rollAngle;
        R_w_c_ =  yawAngle*pitchAngle*rollAngle;
        test_euler = R_w_c_.eulerAngles(2,1,0);
        if(test_euler == euler_)std::cout<<"R_w_c_ to eular is same as origin"<<std::endl;
        return R_w_c_;
    }
    Eigen::Matrix3d BackProject::qt2rota() {
        rota_vec_ =Eigen::AngleAxisd(qt_);
        return rota_vec_.toRotationMatrix();
//        return qt_.toRotationMatrix();

    }
    bool BackProject::getRotaUsingEuler() {
        this->R_w_c_ = euler2rota();
        return true;
    }

    bool BackProject::getRotaUsingQt() {
        this->R_w_c_ = qt2rota();
    }

    bool BackProject::getTwoPtsInCam() {
        this->pt_cam_ = pixel2cam();
        return true;
    }

    bool BackProject::getTwoPtsInWld() {
        pt_wld_ = R_w_c_*pt_cam_ + T_w_c_;
        PT_WLD_O_ = R_w_c_*PT_CAM_O_ + T_w_c_;
        pt_w_x_ = pt_wld_(0);
        pt_w_y_ = pt_wld_(1);
        pt_w_z_ = pt_wld_(2);
        return true;
    }

    Eigen::Vector3d BackProject::calcuTargetUsingQt() {
        if(use_qt){
            std::cout<<"using qt!"<<std::endl;
            if(getRotaUsingQt()&&getTwoPtsInCam()&&getTwoPtsInWld()){
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                std::cout<<K_<<std::endl;
                std::cout<<"pt_cam"<<pt_cam_<<std::endl;
                std::cout<<"PT_CAM_O"<<PT_CAM_O_<<std::endl;
                std::cout<<"pt_wld_:"<<pt_wld_<<std::endl;
                std::cout<<"PT_WLD_O:"<<PT_WLD_O_<<std::endl;
                std::cout<<"euler(ypr)"<<euler_<<std::endl;
//                std::cout<<"R_c_w_"<<R_c_w_<<std::endl;
                std::cout<<"T_c_w_"<<T_c_w_<<std::endl;
                std::cout<<"R_w_c_"<<R_w_c_<<std::endl;
                std::cout<<"T_w_c_"<<T_w_c_<<std::endl;
                VL_ = pt_wld_ - PT_WLD_O_;
                v1 = VL_(0,0);
                v2 = VL_(1,0);
                v3 = VL_(2,0);
                mode = std::sqrt(v1*v1 +v2*v2 +v3*v3);
                v1 /=mode;
                v2 /=mode;
                v3 /=mode;
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
                    std::cout<<"divide within 0!"<<std::endl;
                }
                t /= (vp1*v1 + vp2*v2 + vp3*v3);
                if(t == ((n3-m3)/v3)){
                    std::cout<<"t is as expected:"<<t<<std::endl;
                }

                std::cout<<"x:"<<m1 + v1*t<<" y:"<<m2 + v2*t<<" z:"<<z0_<<std::endl;
                double res3 = m3+v3*t;
                pt_target_ = Eigen::Vector3d{m1 + v1*t, m2 + v2*t ,z0_};
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                std::chrono::duration<double>  time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
                std::cout<<"calculate pts target cost time:" << time_used.count()<<"scds."<<std::endl;
                return pt_target_;
            }else if(!getRotaUsingQt()){
                std::cout<<"cannot get rota mat from QT!"<<std::endl;
            }
            else if(!getTwoPtsInCam()){
                std::cout<<"cannot get pts in cam correctly!"<<std::endl;
            } else if(!getTwoPtsInWld()){
                std::cout<<"cannot get pts in world correctly!"<<std::endl;
            }
        }
    }

    Eigen::Vector3d BackProject::calcuTarget() {
        if(use_euler){
            std::cout<<"using euler"<<std::endl;
            if(getRotaUsingEuler()&&getTwoPtsInCam()&&getTwoPtsInWld()){
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                std::cout<<K_<<std::endl;
                std::cout<<"pt_cam"<<pt_cam_<<std::endl;
                std::cout<<"PT_CAM_O"<<PT_CAM_O_<<std::endl;
                std::cout<<"pt_wld_:"<<pt_wld_<<std::endl;
                std::cout<<"PT_WLD_O:"<<PT_WLD_O_<<std::endl;
                std::cout<<"euler(ypr)"<<euler_<<std::endl;
//                std::cout<<"R_c_w_"<<R_c_w_<<std::endl;
                std::cout<<"T_c_w_"<<T_c_w_<<std::endl;
                std::cout<<"R_w_c_"<<R_w_c_<<std::endl;
                std::cout<<"T_w_c_"<<T_w_c_<<std::endl;
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
                    std::cout<<"divide within 0!"<<std::endl;
                }
                t /= (vp1*v1 + vp2*v2 + vp3*v3);
                if(t == ((n3-m3)/v3)){
                    std::cout<<"t is as expected:"<<t<<std::endl;
                }

                std::cout<<"x:"<<m1 + v1*t<<" y:"<<m2 + v2*t<<" z:"<<z0_<<std::endl;
                double res3 = m3+v3*t;
                pt_target_ = Eigen::Vector3d{m1 + v1*t, m2 + v2*t ,z0_};
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                std::chrono::duration<double>  time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
                std::cout<<"calculate pts target cost time:" << time_used.count()<<"scds."<<std::endl;
                return pt_target_;
            }else if(!getRotaUsingEuler()){
                std::cout<<"cannot get rotation mat from euler!"<<std::endl;
            }
            else if(!getTwoPtsInCam()){
                std::cout<<"cannot get pts in cam correctly!"<<std::endl;
            } else if(!getTwoPtsInWld()){
                std::cout<<"cannot get pts in world correctly!"<<std::endl;
            }
        } else{
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
                pt_target_ = Eigen::Vector3d{m1 + v1*t, m2 + v2*t ,z0_};
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                std::chrono::duration<double>  time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
                std::cout<<"calculate pts target cost time:" << time_used.count()<<"scds."<<std::endl;
                return pt_target_;
            }else if(!getTwoPtsInCam()){
                std::cout<<"cannot get pts in cam correctly!"<<std::endl;
            } else if(!getTwoPtsInWld()){
                std::cout<<"cannot get pts in world correctly!"<<std::endl;
            }
        }
        }

        void BackProject::showTargetInWLD() {

        ground_ = cv::imread("../asserts/ground.png");
//        cv::Mat base = cv::Mat::zeros(1500,1000,CV_8UC3);
//        cv::Mat roi = base(cv::Rect(321,256,ground_.cols/2,ground_.rows/2));
//        ground_.copyTo(roi);
//        roi = ground_.clone();
        double x = pt_target_[0]*100+429;
        double y = 244-pt_target_[1]*100;
        cv::circle(ground_,cv::Point2d(x,y),12.5,cv::Scalar(255, 255, 255), -1, 8, 0);
        cv::imshow("taget_wld",ground_);
        cv::waitKey(0);
    }

}