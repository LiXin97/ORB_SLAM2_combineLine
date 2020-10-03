//
// Created by xin on 2020/10/3.
//
#ifndef ORB_SLAM2_LINEGEO_HPP
#define ORB_SLAM2_LINEGEO_HPP

#include <eigen3/Eigen/Dense>
#include <tuple>



class LineGeo
{
public:
    // n*P + d = 0;  pi( n(0), n(1), n(2), d );
    static Eigen::Vector4d pi_from_ppp( const Eigen::Vector3d& p0,
                                 const Eigen::Vector3d& p1,
                                 const Eigen::Vector3d& p2)
    {
        Eigen::Vector4d pi;
        Eigen::Vector3d n = ( p0 - p2 ).cross( p1 - p2 );
        n.normalize();
//        if(n(2) < 0)
//        {
//            n(0) = -n(0);
//            n(1) = -n(1);
//            n(2) = -n(2);
//        }
        pi << n, - p2.dot( n );
        return pi;
    }

    static std::tuple<Eigen::Vector3d, Eigen::Vector3d>
            plk_from_pp( const Eigen::Vector4d& p0, const Eigen::Vector4d& p1)
    {
        Eigen::Matrix4d dp = p0 * p1.transpose() - p1 * p0.transpose();

        Eigen::Vector3d n( dp(0,3), dp(1,3), dp(2,3) );
        Eigen::Vector3d d( - dp(1,2), dp(0,2), - dp(0,1) );

        n /= d.norm();
        d /= d.norm();
        return std::make_tuple(n, d);
    }
};


#endif