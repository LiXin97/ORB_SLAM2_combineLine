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
//    static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &q)
//    {
//        Eigen::Matrix3d ans;
//        ans << 0., -q(2), q(1),
//                q(2), 0., -q(0),
//                -q(1), q(0), 0.;
//        return ans;
//    }
//
//    static Eigen::Quaterniond deltaQ(const Eigen::Vector3d &theta)
//    {
//        Eigen::Quaterniond dq;
//        Eigen::Vector3d half_theta = theta;
//        half_theta /= 2.;
//        dq.w() = 1.;
//        dq.x() = half_theta.x();
//        dq.y() = half_theta.y();
//        dq.z() = half_theta.z();
//        return dq;
//    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
                q(2), typename Derived::Scalar(0), -q(0),
                -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }
};


#endif