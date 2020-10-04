//
// Created by xin on 2020/10/2.
//
#ifndef ORB_SLAM2_MAPLINE_H
#define ORB_SLAM2_MAPLINE_H

#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/line_descriptor.hpp>
#include <mutex>

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "Converter.h"
#include "xin/LineGeometry.hpp"
#include "xin/Ulity.hpp"
#include "ORBmatcher.h"

namespace ORB_SLAM2
{
    class Map;

    class Plucker
    {
    public:
        Plucker( const Eigen::Vector4d& p0, const Eigen::Vector4d& p1 )
        {
            std::tie( norm_, dirction_ ) = LineGeo::plk_from_pp( p0, p1 );
        }

        Plucker(const Eigen::Vector3d& norm, const Eigen::Vector3d& dirction)
        :norm_(norm),dirction_(dirction)
        {}

        Plucker() = default;

        Plucker(const Plucker& _plucker) = default;

        // transform from ori to new
        void plk_transform( const Eigen::Matrix3d& Rno, const Eigen::Vector3d& tno ) {
            norm_ = Rno * norm_ + Ulity::skewSymmetric(tno) * Rno * dirction_;
            dirction_ = Rno * dirction_;
//            Eigen::Vector3d nc = Rcw * nw + skew_symmetric(tcw) * Rcw * vw;
//            Eigen::Vector3d vc = Rcw * vw;
        }

        Plucker Get_plk_transform( const Eigen::Matrix3d& Rno, const Eigen::Vector3d& tno ) {
            Eigen::Vector3d norm = Rno * norm_ + Ulity::skewSymmetric(tno) * Rno * dirction_;
            Eigen::Vector3d dirction = Rno * dirction_;
            return Plucker( norm, dirction );
        }

        std::tuple<Eigen::Vector3d, Eigen::Vector3d>
                Get_nd_transform( const Eigen::Matrix3d& Rno, const Eigen::Vector3d& tno ) {
            Eigen::Vector3d norm = Rno * norm_ + Ulity::skewSymmetric(tno) * Rno * dirction_;
            Eigen::Vector3d dirction = Rno * dirction_;
            return std::make_tuple( norm, dirction );
        }

        std::tuple<Eigen::Vector3d, Eigen::Vector3d>
        Get3D( const Eigen::Vector3d& ob0,
               const Eigen::Vector3d& ob1)
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

        std::tuple<Eigen::Vector3d, Eigen::Vector3d>
                Get3D( const Eigen::Matrix3d& Rwc,
                       const Eigen::Vector3d& twc,
                       const Eigen::Vector3d& ob0,
                       const Eigen::Vector3d& ob1)
        {
            Eigen::Matrix3d Rcw = Rwc.transpose();
            Eigen::Vector3d tcw = -Rcw*twc;
            auto plucker_cam = Get_plk_transform(Rcw, tcw);
            auto[ pts_0, pts_1 ] = plucker_cam.Get3D( ob0, ob1 );


            Eigen::Vector3d w_pts_0 =  Rwc * pts_0 + twc;
            Eigen::Vector3d w_pts_1 =  Rwc * pts_1 + twc;

            return std::make_tuple( w_pts_0, w_pts_1 );
        }

        std::tuple<Eigen::Vector3d, Eigen::Vector3d>
        Get_nd(  ) {
            return std::make_tuple( norm_, dirction_ );
        }

        [[nodiscard]] Eigen::Vector3d GetNorm( ) const {return norm_;}
        [[nodiscard]] Eigen::Vector3d GetDir( ) const {return dirction_;}

    private:
        Eigen::Vector3d norm_;
        Eigen::Vector3d dirction_;
    };

    class LineOrth
    {
    public:

    private:
        Eigen::Vector4d orth_;
    };

    class MapLine
    {
    public:
        MapLine( const Plucker &plucker, KeyFrame* pRefKF, Map* pMap );


        void AddObservation(KeyFrame* pKF,size_t idx);
        void EraseObservation(KeyFrame* pKF);

        void Update3D();

        void IncreaseVisible(int n=1);
        void IncreaseFound(int n=1);

        void SetBadFlag();
        bool isBad();
        bool IsInKeyFrame(KeyFrame* pKF);

        double GetFoundRatio();

        std::map<KeyFrame*,size_t> GetObservations();
        int Observations();
        inline int GetFound(){
            return mnFound;
        }
        cv::Mat GetDescriptor();
        void ComputeDistinctiveDescriptors();
        int GetIndexInKeyFrame(KeyFrame *pKF);

        std::tuple<Eigen::Vector3d, Eigen::Vector3d>
        GetStarEndPoints(  ) const {
            return std::make_tuple( mstartPoint3d_, mendPoint3d_ );
        }

        Plucker GetPlucker() const {return plucker_;}


        long unsigned int mnId;
        static long unsigned int nNextId;
        long int mnFirstKFid;
        long int mnFirstFrame;
        int nObs;


        bool mbTrackInView;
        long unsigned int mnTrackReferenceForFrame;
        long unsigned int mnLastFrameSeen;

        static std::mutex mGlobalMutex;
    private:
        Plucker plucker_;

        Eigen::Vector3d mstartPoint3d_, mendPoint3d_;

        // Keyframes observing the point and associated index in keyframe
        std::map<KeyFrame*,size_t> mObservations;

        // Mean viewing direction
        cv::Mat mNormalVector;

        // Best descriptor to fast matching
        cv::Mat mDescriptor;

        // Reference KeyFrame
        KeyFrame* mpRefKF;

        // Tracking counters
        int mnVisible;
        int mnFound;

        // Bad flag (we do not currently erase MapPoint from memory)
        bool mbBad;
        MapLine* mpReplaced;

        Map* mpMap;

        std::mutex mMutexPos;
        std::mutex mMutexFeatures;
    };
}

#endif