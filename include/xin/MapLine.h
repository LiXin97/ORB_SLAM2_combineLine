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
#include "LineParam.h"

namespace ORB_SLAM2
{
    class Map;

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

        void SetOrth( const Eigen::Vector4d& Orth );

        long unsigned int mnId;
        static long unsigned int nNextId;
        long int mnFirstKFid;
        long int mnFirstFrame;
        int nObs;

        // Variables used by local mapping
        long unsigned int mnBALocalForKF;
        long unsigned int mnFuseCandidateForKF;


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