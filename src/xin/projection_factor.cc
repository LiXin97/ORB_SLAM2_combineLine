//
// Created by lixin04 on 19年12月6日.
//

#include "xin/LineParam.h"
#include "include/xin/projection_factor.hpp"
#include "xin/local_parameterization.hpp"
#include "xin/Ulity.hpp"

Eigen::Matrix2d MonoProjection::sqrt_info;

MonoProjection::MonoProjection(Eigen::Vector2d& obs, double invSigma2)
:point_obs_(obs), invSigma2_(invSigma2){}

Eigen::Vector2d MonoProjection::compute_error(const double *parameters_pose, const double *parameters_point, const Eigen::Vector2d &obs, bool& bad_point)
{
    bad_point = false;
    Eigen::Vector3d tcw(parameters_pose[0], parameters_pose[1], parameters_pose[2]);
    Eigen::Quaterniond qcw(parameters_pose[6], parameters_pose[3], parameters_pose[4], parameters_pose[5]);
    Eigen::Vector3d PointW(parameters_point[0], parameters_point[1], parameters_point[2]);
    Eigen::Vector3d pts_camera = qcw * PointW + tcw;
    if( pts_camera(2) < 0 ) bad_point = true;
    return sqrt_info * Eigen::Vector2d( pts_camera(0)/pts_camera(2) - obs(0), pts_camera(1)/pts_camera(2) - obs(1) );
}

bool MonoProjection::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d tcw(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond qcw(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d PointW(parameters[1][0], parameters[1][1], parameters[1][2]);


    Eigen::Vector3d pts_camera = qcw * PointW + tcw;

    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = pts_camera(0)/pts_camera(2) - point_obs_(0);
    residual(1) = pts_camera(1)/pts_camera(2) - point_obs_(1);

    residual = sqrt_info * invSigma2_ * residual;

    if (jacobians)
    {
        Eigen::Matrix3d Rcw = qcw.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        reduce << 1. / pts_camera(2), 0, -pts_camera(0) / (pts_camera(2) * pts_camera(2)),
                0, 1. / pts_camera(2), -pts_camera(1) / (pts_camera(2) * pts_camera(2));
        reduce = sqrt_info * invSigma2_ * reduce;

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);

            Eigen::Matrix<double, 3, 6> jaco_pose;
            jaco_pose.leftCols<3>() = Eigen::Matrix3d::Identity();
            jaco_pose.rightCols<3>() = - Rcw * Ulity::skewSymmetric(PointW);
            jacobian_pose.leftCols<6>() = reduce * jaco_pose;
            jacobian_pose.rightCols<1>().setZero();
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_point(jacobians[1]);
            jacobian_point =  reduce * Rcw;
        }
    }

    return true;
}


Eigen::Matrix2d MonoLineProjection::sqrt_info;

MonoLineProjection::MonoLineProjection(Eigen::Vector4d &obs): line_obs_(obs)
{}

Eigen::Vector2d MonoLineProjection::compute_error(const double *parameters_pose, const double *parameters_line, const Eigen::Vector4d &obs, bool &bad_line)
{
    bad_line = false;
    Eigen::Vector3d tcw(parameters_pose[0], parameters_pose[1], parameters_pose[2]);
    Eigen::Quaterniond qcw(parameters_pose[6], parameters_pose[3], parameters_pose[4], parameters_pose[5]);
    Eigen::Vector4d line_orth(parameters_line[0], parameters_line[1], parameters_line[2], parameters_line[3]);
    Eigen::Matrix3d Rcw(qcw);
    return compute_error( Rcw, tcw, line_orth, obs, bad_line );
}

Eigen::Vector2d MonoLineProjection::compute_error(const Eigen::Matrix3d &Rcw, const Eigen::Vector3d &tcw,
                                                  const Eigen::Vector4d &Orth, const Eigen::Vector4d &obs,
                                                  bool &bad_line)
{
    auto plucker = ORB_SLAM2::Plucker(Orth);

    // TODO xinli xinli xinli    why different???????????????????????????????????????????????????????????
    auto [norm_c, dir_c] = plucker.Get_nd_transform(Rcw, tcw);

//    auto [norm_w, dir_w] = plucker.Get_nd();
//    auto norm_c = Rcw * norm_w + Ulity::skewSymmetric(tcw) * Rcw * dir_w;
//    auto dir_c = Rcw * dir_w;

//    auto [ norm_w, dir_w ] = ORB_SLAM2::Plucker::Orth2Plucker( Orth );
//        std::cout << "compute_error norm_w = " << norm_w.transpose() << std::endl;
//        std::cout << "compute_error dir_w = " << dir_w.transpose() << std::endl;
//    auto norm_c = Rcw * norm_w + Ulity::skewSymmetric(tcw) * Rcw * dir_w;
//    auto dir_c = Rcw * dir_w;

//    auto [n_c, d_c] = plucker.Get_nd_transform(Rcw, tcw);


//    Eigen::Vector3d norm_w, dirc_w;
//    std::tie( norm_w, dirc_w ) = plucker.Get_nd();
//
////    auto [norm_w, dirc_w] = plucker.Get_nd();
//    //TODO xinli xinli xinli why different???????????????????????????????????????
//    auto norm_c_c = (Rcw * norm_w + Ulity::skewSymmetric(tcw) * Rcw * dirc_w);
//    auto dir_c_c = Rcw * dirc_w;
//    {
//        std::cout <<"=================="<<std::endl;
//        auto norm = Rcw * norm_w + Ulity::skewSymmetric(tcw) * Rcw * dirc_w;
//        auto dirction = Rcw * dirc_w;
//        std::cout << "norm = " << norm.transpose() << std::endl;
//        std::cout << "dirction = " << dirction.transpose() << std::endl;
//    }
////    plucker.plk_transform( Rcw, tcw );
////    auto [norm_c, dir_c] = plucker.Get_nd();
//    std::cout << "n_c = " << n_c.transpose() << std::endl;
//    std::cout << "d_c = " << d_c.transpose() << std::endl;
//    std::cout << "norm_c_c = " << norm_c_c.transpose() << std::endl;
//    std::cout << "dir_c_c = " << dir_c_c.transpose() << std::endl;
//    std::cout <<"=================="<<std::endl;
//    auto norm_c = plucker.GetNorm();



    double l_norm = norm_c(0) * norm_c(0) + norm_c(1) * norm_c(1);
    double l_sqrtnorm = sqrt( l_norm );

    double e1 = obs(0) * norm_c(0) + obs(1) * norm_c(1) + norm_c(2);
    double e2 = obs(2) * norm_c(0) + obs(3) * norm_c(1) + norm_c(2);
    return sqrt_info * Eigen::Vector2d( e1/l_sqrtnorm, e2/l_sqrtnorm );
}

Eigen::Vector2d MonoLineProjection::compute_error(const Eigen::Matrix3d &Rcw, const Eigen::Vector3d &tcw,
                                                  const ORB_SLAM2::Plucker &plucker, const Eigen::Vector4d &obs,
                                                  bool &bad_line)
{
    auto [n_c, d_c] = plucker.Get_nd_transform(Rcw, tcw);


//    Eigen::Vector3d norm_w, dirc_w;
//    std::tie( norm_w, dirc_w ) = plucker.Get_nd();
//
////    auto [norm_w, dirc_w] = plucker.Get_nd();
//    //TODO xinli xinli xinli why different???????????????????????????????????????
//    auto norm_c_c = (Rcw * norm_w + Ulity::skewSymmetric(tcw) * Rcw * dirc_w);
//    auto dir_c_c = Rcw * dirc_w;
//    {
//        std::cout <<"=================="<<std::endl;
//        auto norm = Rcw * norm_w + Ulity::skewSymmetric(tcw) * Rcw * dirc_w;
//        auto dirction = Rcw * dirc_w;
//        std::cout << "norm = " << norm.transpose() << std::endl;
//        std::cout << "dirction = " << dirction.transpose() << std::endl;
//    }
////    plucker.plk_transform( Rcw, tcw );
////    auto [norm_c, dir_c] = plucker.Get_nd();
//    std::cout << "n_c = " << n_c.transpose() << std::endl;
//    std::cout << "d_c = " << d_c.transpose() << std::endl;
//    std::cout << "norm_c_c = " << norm_c_c.transpose() << std::endl;
//    std::cout << "dir_c_c = " << dir_c_c.transpose() << std::endl;
//    std::cout <<"=================="<<std::endl;
//    auto norm_c = plucker.GetNorm();



    double l_norm = n_c(0) * n_c(0) + n_c(1) * n_c(1);
    double l_sqrtnorm = sqrt( l_norm );

    double e1 = obs(0) * n_c(0) + obs(1) * n_c(1) + n_c(2);
    double e2 = obs(2) * n_c(0) + obs(3) * n_c(1) + n_c(2);
    return sqrt_info * Eigen::Vector2d( e1/l_sqrtnorm, e2/l_sqrtnorm );
}

bool MonoLineProjection::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d tcw(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qcw(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector4d line_orth( parameters[1][0],parameters[1][1],parameters[1][2],parameters[1][3] );
    Eigen::Matrix3d Rcw(Qcw);

    auto plucker = ORB_SLAM2::Plucker(line_orth);
    auto [norm_w, dirc_w] = plucker.Get_nd();
//    plucker.plk_transform( Rcw, tcw );
//    auto norm_c = plucker.GetNorm();

    auto [norm_c, dirc_c] = plucker.Get_nd_transform(Rcw, tcw);
//    auto norm_c = Rcw * norm_w + Ulity::skewSymmetric(tcw) * Rcw * dirc_w;
//    auto dirc_c = Rcw * dirc_w;


    double l_norm = norm_c(0) * norm_c(0) + norm_c(1) * norm_c(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = line_obs_(0) * norm_c(0) + line_obs_(1) * norm_c(1) + norm_c(2);
    double e2 = line_obs_(2) * norm_c(0) + line_obs_(3) * norm_c(1) + norm_c(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;
    residual = sqrt_info * residual;

//    std::cout << "line residual = " << residual.transpose() << std::endl;

    if (jacobians)
    {
        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l <<
        (line_obs_(0)/l_sqrtnorm - norm_c(0) * e1 / l_trinorm ), (line_obs_(1)/l_sqrtnorm - norm_c(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
        (line_obs_(2)/l_sqrtnorm - norm_c(0) * e2 / l_trinorm ), (line_obs_(3)/l_sqrtnorm - norm_c(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;
        jaco_e_l = sqrt_info * jaco_e_l;
        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;


        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
            jacobian_pose.setZero();

            Eigen::Matrix<double, 6, 6> jacobian_Lc_pose;
            jacobian_Lc_pose.setZero();
            jacobian_Lc_pose.block<3,3>(0,0) = -Ulity::skewSymmetric(Rcw * dirc_w);
            jacobian_Lc_pose.block<3,3>(0,3) = -Ulity::skewSymmetric(norm_w) - -Ulity::skewSymmetric(tcw) * Rcw * -Ulity::skewSymmetric(dirc_w);
            jacobian_Lc_pose.block<3,3>(3,3) = -Rcw*Ulity::skewSymmetric(dirc_w);

            jacobian_pose.block<2, 6>(0,0) = jaco_e_Lc * jacobian_Lc_pose;
        }

//        {
//            norm_c = Rcw * norm_w + Ulity::skewSymmetric(tcw) * Rcw * dirc_w;
//            dirc_c = Rcw * dirc_w;
//        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_Orth(jacobians[1]);
            jacobian_Orth.setZero();

            Eigen::Matrix<double, 6, 6> jacobian_Lc_Lw;
            jacobian_Lc_Lw.setZero();
            jacobian_Lc_Lw.block<3,3>(0,0) = Rcw;
            jacobian_Lc_Lw.block<3,3>(0,3) = Ulity::skewSymmetric(tcw) * Rcw;
            jacobian_Lc_Lw.block<3,3>(3,3) = Rcw;

            Eigen::Vector3d u1 = norm_w/norm_w.norm();
            Eigen::Vector3d u2 = dirc_w/dirc_w.norm();
            Eigen::Vector3d u3 = u1.cross(u2);
            Eigen::Vector2d w( norm_w.norm(), norm_w.norm() );
            w = w/w.norm();
            Eigen::Matrix<double, 6, 4> jacobian_Lw_Orth;
            jacobian_Lw_Orth.setZero();
            jacobian_Lw_Orth.block(3,0,3,1) = w[1] * u3;
            jacobian_Lw_Orth.block(0,1,3,1) = -w[0] * u3;
            jacobian_Lw_Orth.block(0,2,3,1) = w(0) * u2;
            jacobian_Lw_Orth.block(3,2,3,1) = -w(1) * u1;
            jacobian_Lw_Orth.block(0,3,3,1) = -w(1) * u1;
            jacobian_Lw_Orth.block(3,3,3,1) = w(0) * u2;

            jacobian_Orth = jaco_e_Lc * jacobian_Lc_Lw * jacobian_Lw_Orth;
        }

        if (true)
        {
            // check jacobian
//            std::cout << "ana = " << std::endl;
//            if( jacobians[0] )
//            std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
//                      << std::endl;
//            if( jacobians[1] )
//            std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[1]) << std::endl
//                      << std::endl;

            const auto eps = 1e-6;
            Eigen::Matrix<double, 2, 10> num_jacobian;
            for (int k = 0; k < 10; k++)
            {
                Eigen::Vector3d tcw_ck(parameters[0][0], parameters[0][1], parameters[0][2]);
                Eigen::Quaterniond Qcw_ck(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

                Eigen::Vector4d line_orth_ck( parameters[1][0],parameters[1][1],parameters[1][2],parameters[1][3]);
                ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();

                int a = k / 3, b = k % 3;
                Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

                if (a == 0)
                    tcw_ck += delta;
                else if (a == 1)
                    Qcw_ck = Qcw_ck * Ulity::deltaQ(delta);
                else if (a == 2) {           // line orth的前三个元素
                    Eigen::Vector4d line_new;
                    Eigen::Vector4d delta_l;
                    delta_l<< delta, 0.0;
                    local_parameterization_line->Plus(line_orth_ck.data(),delta_l.data(),line_new.data());
                    line_orth_ck = line_new;
                }
                else if (a == 3) {           // line orth的最后一个元素
                    Eigen::Vector4d line_new;
                    Eigen::Vector4d delta_l;
                    delta_l.setZero();
                    delta_l[3]= delta.x();
                    local_parameterization_line->Plus(line_orth_ck.data(),delta_l.data(),line_new.data());
                    line_orth_ck = line_new;
                }

                Eigen::Matrix3d Rcw_ck(Qcw_ck);
                bool error_flag = false;
                Eigen::Vector2d tmp_residual = compute_error( Rcw_ck, tcw_ck, line_orth_ck, line_obs_, error_flag );
                num_jacobian.col(k) = (tmp_residual - residual) / eps;

            }

            if( jacobians[0] )
            {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
                jacobian_pose.block<2, 6>(0, 0) = num_jacobian.block<2, 6>( 0, 0 );
            }
            if( jacobians[1] )
            {
                Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_Orth(jacobians[1]);
                jacobian_Orth.block<2, 4>(0, 0) = num_jacobian.block<2, 4>( 0, 6 );
            }
//            std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;
        }
    }

    return true;
}