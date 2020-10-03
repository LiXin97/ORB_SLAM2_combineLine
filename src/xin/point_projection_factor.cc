//
// Created by lixin04 on 19年12月6日.
//

#include "include/xin/point_projection_factor.hpp"
#include "xin/Ulity.hpp"

Eigen::Matrix2d MonoProjection::sqrt_info;

MonoProjection::MonoProjection(Eigen::Vector2d& obs) :point_obs(obs){}

bool MonoProjection::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d tcw(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond qcw(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d PointW(parameters[1][0], parameters[1][1], parameters[1][2]);


    Eigen::Vector3d pts_camera = qcw * PointW + tcw;

    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = pts_camera(0)/pts_camera(2) - point_obs(0);
    residual(1) = pts_camera(1)/pts_camera(2) - point_obs(1);

    residual = sqrt_info * residual;

    if (jacobians)
    {
        Eigen::Matrix3d Rcw = qcw.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);
        reduce << 1. / pts_camera(2), 0, -pts_camera(0) / (pts_camera(2) * pts_camera(2)),
                0, 1. / pts_camera(2), -pts_camera(1) / (pts_camera(2) * pts_camera(2));
        reduce = sqrt_info * reduce;

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