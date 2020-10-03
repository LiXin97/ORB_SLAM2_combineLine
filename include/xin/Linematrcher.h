//
// Created by xin on 2020/10/3.
//

#pragma once
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "MapLine.h"
#include "KeyFrame.h"
#include "Frame.h"


namespace ORB_SLAM2
{
    class LineMatcher
    {
    public:
        explicit LineMatcher( float nnratio = 0.6, bool checkOri = true );

        int SearchForTriangulation(
                KeyFrame *pKF1, KeyFrame* pKF2,
                std::vector<pair<size_t, size_t> > &vMatchedPairs
                ) const;

        int SearchByProjection(
                Frame &CurrentFrame, const Frame &LastFrame, const float th
                ) const;

        static const int TH_LOW;
        static const int TH_HIGH;

    private:
        const float mnnratio;
        const bool mcheckOri;

    };
}
