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
        ldesc1 = pKF1->mDescriptorLine;
        ldesc2 = pKF2->mDescriptorLine;

        cv::Mat ldesc1_no, ldesc2_no;
        std::vector< size_t > ori_idex1, ori_index2;
        {
            auto MapLine1 = pKF1->GetMapLineMatches();
            auto MapLine2 = pKF2->GetMapLineMatches();
            for( int index = 0;index < pKF1->NL;++index )
            {
                if( MapLine1[index] ) continue;
                ldesc1_no.push_back( ldesc1.row(index) );
                ori_idex1.push_back( index );
            }
            for( int index = 0;index < pKF2->NL;++index )
            {
                if( MapLine2[index] ) continue;
                ldesc2_no.push_back( ldesc2.row(index) );
                ori_index2.push_back( index );
            }
        }

        vector<vector<cv::DMatch>> lmatches;

        if( ldesc1_no.empty() || ldesc2_no.empty() ) return nmatches;

        bm->knnMatch(ldesc1_no, ldesc2_no, lmatches, 2);

        /* select best matches */
        std::vector<cv::DMatch> good_matches;
        for (auto & lmatche : lmatches)
        {
            if( lmatche[0].distance < TH_LOW && lmatche[0].distance < lmatche[1].distance * mnnratio  ){

                vMatchedPairs.emplace_back(
                        ori_idex1[ lmatche[0].queryIdx ],
                        ori_index2[ lmatche[0].trainIdx ]);
                nmatches++;
            }

        }

        return nmatches;
    }
}