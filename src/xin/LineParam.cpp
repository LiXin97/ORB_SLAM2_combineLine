//
// Created by xin on 2020/10/5.
//

#include <iostream>
#include "include/xin/LineParam.h"

namespace ORB_SLAM2
{
    Eigen::Vector4d Plucker::GetOrth()
    {
        return Plucker2Orth( norm_, dirction_ );
    }

    Eigen::Vector4d Plucker::Plucker2Orth(const Eigen::Vector3d &norm, const Eigen::Vector3d &dirction)
    {
        Eigen::Vector3d u1 = norm/norm.norm();
        Eigen::Vector3d u2 = dirction/dirction.norm();
        Eigen::Vector3d u3 = u1.cross(u2);
        Eigen::Vector2d w( norm.norm(), dirction.norm() );
        w = w/w.norm();
        return Eigen::Vector4d(
                atan2( u2(2),u3(2) ),
                asin( -u1(2) ),
                atan2( u1(1),u1(0) ),
                asin( w(1) ) );
    }

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> Plucker::Orth2Plucker(const Eigen::Vector4d &Orth)
    {
        Eigen::Vector3d theta = Orth.head(3);
        double phi = Orth[3];
        double s1 = sin(theta[0]);
        double c1 = cos(theta[0]);
        double s2 = sin(theta[1]);
        double c2 = cos(theta[1]);
        double s3 = sin(theta[2]);
        double c3 = cos(theta[2]);
        Eigen::Matrix3d R;
        R <<
          c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
                c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
                -s2,                  s1 * c2,                  c1 * c2;
        double w1 = cos(phi);
        double w2 = sin(phi);
        double distance = w1/w2;      // 原点到直线的距离

        Eigen::Vector3d u1 = R.col(0);
        Eigen::Vector3d u2 = R.col(1);

        Eigen::Vector3d norm = w1 * u1;
        Eigen::Vector3d direction = w2 * u2;

        return std::make_tuple( norm, direction );
    }

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> Plucker::Get3D(const Eigen::Vector3d &ob0, const Eigen::Vector3d &ob1)
    {
        auto [nc, vc] = Get_nd();

        Eigen::Matrix4d Lc;
        Lc << Ulity::skewSymmetric(nc), vc, -vc.transpose(), 0;

        Eigen::Vector2d ln = ( ob0.cross(ob1) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Eigen::Vector3d p12 = Eigen::Vector3d(ob0(0) + ln(0), ob0(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Eigen::Vector3d p22 = Eigen::Vector3d(ob1(0) + ln(0), ob1(1) + ln(1), 1.0);
        Eigen::Vector3d cam = Eigen::Vector3d( 0, 0, 0 );

        Eigen::Vector4d pi1 = LineGeo::pi_from_ppp(cam, ob0, p12);
        Eigen::Vector4d pi2 = LineGeo::pi_from_ppp(cam, ob1, p22);

        Eigen::Vector4d e1 = Lc * pi1;
        Eigen::Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);


//            double length = (e1-e2).norm();
        // TODO xinli check
//            if(length > 5 || e1(2) < 0 || e2(2) < 0)
//            {
//                Vector6d ZeroLine;
//                ZeroLine << 0,0,0,0,0,0;
//                return ZeroLine;
////            std::cerr << " length = " << length << std::endl;
//            }

        //std::cout << e1 <<"\n\n";
        Eigen::Vector3d pts_1(e1(0),e1(1),e1(2));
        Eigen::Vector3d pts_2(e2(0),e2(1),e2(2));

        return std::make_tuple( pts_1, pts_2 );


        Eigen::Vector3d dirction_p = ( pts_2 - pts_1 );
        dirction_p.normalize();
        double equ_dir = dirction_p.dot( vc );
        if( equ_dir > -.998 && equ_dir < .998 )
        {
            std::cerr << dirction_p.dot( vc ) << " " <<  equ_dir << std::endl;
        }
//            std::cout << (equ_dir == 1 || equ_dir == -1) << std::endl;
        assert( !( equ_dir > -.998 && equ_dir < .998 ) );
        if( equ_dir < -.998 )
        {
            Eigen::Vector3d tmp;
            tmp = pts_1;
            pts_1 = pts_2;
            pts_2 = tmp;
        }

        return std::make_tuple( pts_1, pts_2 );
    }

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> Plucker::Get3D(
            const Eigen::Matrix3d &Rwc,
            const Eigen::Vector3d &twc,
            const Eigen::Vector3d &ob0,
            const Eigen::Vector3d &ob1)
    {
        Eigen::Matrix3d Rcw = Rwc.transpose();
        Eigen::Vector3d tcw = -Rcw*twc;
        auto plucker_cam = Get_plk_transform(Rcw, tcw);
        auto[ pts_0, pts_1 ] = plucker_cam.Get3D( ob0, ob1 );


        Eigen::Vector3d w_pts_0 =  Rwc * pts_0 + twc;
        Eigen::Vector3d w_pts_1 =  Rwc * pts_1 + twc;

        return std::make_tuple( w_pts_0, w_pts_1 );
    }

}