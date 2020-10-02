//
// Created by xin on 2020/10/3.
//

#include "include/xin/Linematrcher.h"

namespace ORB_SLAM2
{

    const int LineMatcher::TH_HIGH = 100;
    const int LineMatcher::TH_LOW = 50;

    LineMatcher::LineMatcher(float nnratio, bool checkOri)
    :mnnratio(nnratio), mcheckOri(checkOri)
    {

    }

    int LineMatcher::SearchForTriangulation(
            KeyFrame *pKF1, KeyFrame *pKF2,
            std::vector<pair<size_t, size_t>> &vMatchedPairs) const
    {
        vMatchedPairs.clear();
        int nmatches = 0;
        auto bm = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
        cv::Mat ldesc1, ldesc2;
        vector<vector<cv::DMatch>> lmatches;
        ldesc1 = pKF1->mDescriptorLine;
        ldesc2 = pKF2->mDescriptorLine;

        if( ldesc1.empty() || ldesc2.empty() ) return nmatches;

        bm->knnMatch(ldesc1, ldesc2, lmatches, 2);

        /* select best matches */
        std::vector<cv::DMatch> good_matches;
        for (auto & lmatche : lmatches)
        {
            if( lmatche[0].distance < TH_LOW && lmatche[0].distance < lmatche[1].distance * mnnratio  ){

                vMatchedPairs.emplace_back(lmatche[0].queryIdx, lmatche[0].trainIdx );
                nmatches++;
            }

        }

        return nmatches;
    }
}