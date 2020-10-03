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

        Plucker() = default;

        Plucker(const Plucker& _plucker) = default;

        // transform from ori to new
        void plk_transform( const Eigen::Matrix3d& Rno, const Eigen::Vector3d& tno ) {
            norm_ = Rno * norm_ + Ulity::skewSymmetric(tno) * Rno * dirction_;
            dirction_ = Rno * dirction_;
//            Eigen::Vector3d nc = Rcw * nw + skew_symmetric(tcw) * Rcw * vw;
//            Eigen::Vector3d vc = Rcw * vw;
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


        void SetBadFlag();
        bool isBad();
        std::map<KeyFrame*,size_t> GetObservations();
        int Observations();
        inline int GetFound(){
            return mnFound;
        }
        cv::Mat GetDescriptor();
        void ComputeDistinctiveDescriptors();

        int GetIndexInKeyFrame(KeyFrame *pKF);


        long unsigned int mnId;
        static long unsigned int nNextId;
        long int mnFirstKFid;
        long int mnFirstFrame;
        int nObs;

        static std::mutex mGlobalMutex;
    private:
        Plucker plucker_;

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