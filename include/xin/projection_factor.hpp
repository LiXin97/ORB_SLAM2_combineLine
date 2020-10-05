//
// Created by lixin04 on 19年12月6日.
//
#pragma once
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <ceres/rotation.h>


class MonoProjection : public ceres::SizedCostFunction<2, 7, 3>
{
public:
    MonoProjection(Eigen::Vector2d& obs);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);
    static Eigen::Vector2d compute_error( double const *parameters_pose, double const *parameters_point, const Eigen::Vector2d& obs, bool& bad_point );
    static Eigen::Matrix2d sqrt_info;
    Eigen::Vector2d point_obs_;
};

class MonoLineProjection : public ceres::SizedCostFunction<2, 7, 4>
{
public:
    MonoLineProjection(Eigen::Vector4d& obs);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);
    static Eigen::Vector2d compute_error( double const *parameters_pose, double const *parameters_line, const Eigen::Vector4d& obs, bool& bad_line);;
    static Eigen::Vector2d compute_error( const Eigen::Matrix3d& Rcw, const Eigen::Vector3d& tcw,
                                          const Eigen::Vector4d& Orth, const Eigen::Vector4d& obs, bool& bad_line);
    static Eigen::Matrix2d sqrt_info;
    Eigen::Vector4d line_obs_;
};