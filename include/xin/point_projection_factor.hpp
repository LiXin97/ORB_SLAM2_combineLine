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

    static Eigen::Matrix2d sqrt_info;
    Eigen::Vector2d point_obs;
};

class MonoLineProjection : public ceres::SizedCostFunction<2, 7, 4>
{
public:
    MonoLineProjection(Eigen::Vector4d& obs);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    static Eigen::Matrix2d sqrt_info;
    Eigen::Vector4d point_obs;
};