//
// Created by xin on 2020/10/2.
//

#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/line_descriptor.hpp>
#include <mutex>

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

namespace ORB_SLAM2
{
    class Map;

    class Plucker
    {
    public:


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

        void SetBadFlag();
        bool isBad();

    private:
        Plucker plucker_;

        // Bad flag
        bool mbBad;

        // Tracking counters
        int mnVisible;
        int mnFound;


        std::mutex mMutexPos;
        std::mutex mMutexFeatures;
    };
}