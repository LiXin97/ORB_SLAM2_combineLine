//
// Created by xin on 2020/10/3.
//
#ifndef ORB_SLAM2_ULITY_H
#define ORB_SLAM2_ULITY_H


#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>

class Ulity
{
public:
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
                q(2), typename Derived::Scalar(0), -q(0),
                -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }
};


#endif