//
// Created by xin on 2020/10/3.
//

#include "include/xin/Linematrcher.h"
#include "xin/projection_factor.hpp"

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

    int LineMatcher::SearchByProjection(Frame &F, const std::vector<MapLine *> &vpLocalMapLines) const
    {
        int nmatches = 0;

        for( auto &pML:vpLocalMapLines )
        {
            if( !pML->mbTrackInView ) continue;
            if( pML->isBad() ) continue;

            const auto MLdescri = pML->GetDescriptor();

            const float &fx = ORB_SLAM2::Frame::fx;
            const float &cx = ORB_SLAM2::Frame::cx;
            const float &cy = ORB_SLAM2::Frame::cy;
            const float &invfx = ORB_SLAM2::Frame::invfx;
            const float &invfy = ORB_SLAM2::Frame::invfy;
            cv::Mat Tcw_mat = F.mTcw;
            Eigen::Matrix<double,3,3> Rcw;
            Rcw << Tcw_mat.at<float>(0,0), Tcw_mat.at<float>(0,1), Tcw_mat.at<float>(0,2),
                    Tcw_mat.at<float>(1,0), Tcw_mat.at<float>(1,1), Tcw_mat.at<float>(1,2),
                    Tcw_mat.at<float>(2,0), Tcw_mat.at<float>(2,1), Tcw_mat.at<float>(2,2);
            Eigen::Matrix<double,3,1> tcw(Tcw_mat.at<float>(0,3), Tcw_mat.at<float>(1,3), Tcw_mat.at<float>(2,3));

            auto plucker = pML->GetPlucker();
            plucker.plk_transform( Rcw, tcw );
            auto norm_c = plucker.GetNorm();
            double l_norm = norm_c(0) * norm_c(0) + norm_c(1) * norm_c(1);
            double l_sqrtnorm = sqrt( l_norm );

            int bestDist=256;
            int bestDist2=256;
            int bestIdx =-1 ;
            for( int i = 0; i<F.NL;++i )
            {
                auto line0 = F.mvKeyLinesUn[i];
                cv::Point2f startPoint = line0.getStartPoint();
                cv::Point2f endPoint = line0.getEndPoint();
                Eigen::Vector4d line_ob = Eigen::Vector4d(
                        (startPoint.x - cx) * invfx,
                        (startPoint.y - cy) * invfy,
                        (endPoint.x - cx) * invfx,
                        (endPoint.y - cy) * invfy);

                double e1 = line_ob(0) * norm_c(0) + line_ob(1) * norm_c(1) + norm_c(2);
                double e2 = line_ob(2) * norm_c(0) + line_ob(3) * norm_c(1) + norm_c(2);
                e1 /= l_sqrtnorm;
                e2 /= l_sqrtnorm;

                if( e1*fx < 5. && e2*fx < 5. )
                {
                    const cv::Mat &d = F.mDescriptorLine.row(i);
                    const int dist = ORBmatcher::DescriptorDistance(MLdescri,d);

                    if(dist<bestDist)
                    {
                        bestDist2=bestDist;
                        bestDist=dist;
                        bestIdx=i;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }
            }

            if( bestDist < TH_LOW )
            {
                if( bestDist > mnnratio * bestDist2 ) continue;

                F.mvpMapLines[bestIdx] = pML;
                nmatches++;
            }
        }

//        std::cout << "LocalMapLine nmatches = " << nmatches << std::endl;

        return nmatches;
    }

    int LineMatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th) const
    {
        int nmatches = 0;

        const cv::Mat cvRcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat cvtcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
        Eigen::Matrix3d Rcw = Converter::toMatrix3d(cvRcw);
        Eigen::Vector3d tcw = Converter::toVector3d(cvtcw);

        const float &cx = ORB_SLAM2::Frame::cx;
        const float &cy = ORB_SLAM2::Frame::cy;
        const float &invfx = ORB_SLAM2::Frame::invfx;
        const float &invfy = ORB_SLAM2::Frame::invfy;
        //  a-----b
        //  |     |
        //  d-----c

        double minX = ORB_SLAM2::Frame::mnMinX;
        double minY = ORB_SLAM2::Frame::mnMinY;
        double maxX = ORB_SLAM2::Frame::mnMaxX;
        double maxY = ORB_SLAM2::Frame::mnMaxY;

        Eigen::Vector3d corna( (minX-cx) * invfx, (minY-cy) * invfy, 1. );
        Eigen::Vector3d cornb( (maxX-cx) * invfx, (minY-cy) * invfy, 1. );
        Eigen::Vector3d cornc( (maxX-cx) * invfx, (maxY-cy) * invfy, 1. );
        Eigen::Vector3d cornd( (minX-cx) * invfx, (maxY-cy) * invfy, 1. );

        cv::Mat descr_last;
        std::vector< size_t > ori_idex;
        std::vector< MapLine* > vpMapLine;
        for( int i = 0; i < LastFrame.NL; ++ i )
        {
            //TODO xinli there are some problems
            auto pML = LastFrame.mvpMapLines[i];
            if( pML )
            {
                std::cout << "pML not nullptr" << std::endl;
            }
            if( LastFrame.mvbLineOutlier[i] )
            {
                std::cout << "LastFrame.mvbLineOutlier[i]  = true" << std::endl;
            }
            if(!pML || LastFrame.mvbLineOutlier[i]) continue;

            vpMapLine.push_back(pML);

            auto plucker = pML->GetPlucker();
            plucker.plk_transform( Rcw, tcw );
            auto nc = plucker.GetNorm();

            double error_a = nc.dot( corna );
            double error_b = nc.dot( cornb );
            double error_c = nc.dot( cornc );
            double error_d = nc.dot( cornd );


            std::cout << "error_a = " << error_a << std::endl;
            std::cout << "error_b = " << error_b << std::endl;
            std::cout << "error_c = " << error_c << std::endl;
            std::cout << "error_d = " << error_d << std::endl;
            if( error_a > 0 && error_b > 0 && error_c > 0 && error_d > 0 )
                continue;
            if( error_a < 0 && error_b < 0 && error_c < 0 && error_d < 0 )
                continue;

            descr_last.push_back( LastFrame.mDescriptorLine.row(i) );
            ori_idex.push_back(i);
        }

//        std::cout << "LastFrame.mDescriptorLine = " << LastFrame.mDescriptorLine.rows << std::endl;
        auto descr_cur = CurrentFrame.mDescriptorLine;
        std::cout << "descr_last = " << descr_last.rows << std::endl;
        if( descr_last.empty() || descr_cur.empty() ) return nmatches;

        vector<vector<cv::DMatch>> lmatches;
        auto bm = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
        bm->knnMatch(descr_cur, descr_last, lmatches, 2);

        /* select best matches */
        std::vector<cv::DMatch> good_matches;
        for (auto & lmatche : lmatches)
        {
            if( lmatche[0].distance < TH_LOW && lmatche[0].distance < lmatche[1].distance * mnnratio  ){

                CurrentFrame.mvpMapLines[ lmatche[0].queryIdx ] = vpMapLine[ lmatche[0].trainIdx ];
                nmatches++;
            }
        }

        std::cout << "LastFrame nmatches = " << nmatches << std::endl;

        return nmatches;
    }


    int LineMatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame* RefKeyFrame, const float th) const
    {
        int nmatches = 0;

//        std::cout << CurrentFrame.mTcw << std::endl;
        return nmatches;

        const cv::Mat cvRcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat cvtcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
        Eigen::Matrix3d Rcw = Converter::toMatrix3d(cvRcw);
        Eigen::Vector3d tcw = Converter::toVector3d(cvtcw);

        const float &cx = ORB_SLAM2::Frame::cx;
        const float &cy = ORB_SLAM2::Frame::cy;
        const float &invfx = ORB_SLAM2::Frame::invfx;
        const float &invfy = ORB_SLAM2::Frame::invfy;
        //  a-----b
        //  |     |
        //  d-----c

        double minX = ORB_SLAM2::Frame::mnMinX;
        double minY = ORB_SLAM2::Frame::mnMinY;
        double maxX = ORB_SLAM2::Frame::mnMaxX;
        double maxY = ORB_SLAM2::Frame::mnMaxY;

        Eigen::Vector3d corna( (minX-cx) * invfx, (minY-cy) * invfy, 1. );
        Eigen::Vector3d cornb( (maxX-cx) * invfx, (minY-cy) * invfy, 1. );
        Eigen::Vector3d cornc( (maxX-cx) * invfx, (maxY-cy) * invfy, 1. );
        Eigen::Vector3d cornd( (minX-cx) * invfx, (maxY-cy) * invfy, 1. );


        cv::Mat descr_last;
        const vector<MapLine*> vpMapLinesKF = RefKeyFrame->GetMapLineMatches();
        for(auto map_line : vpMapLinesKF)
        {
            auto plucker = map_line->GetPlucker();
            plucker.plk_transform( Rcw, tcw );
            auto nc = plucker.GetNorm();

            double error_a = nc.dot( corna );
            double error_b = nc.dot( cornb );
            double error_c = nc.dot( cornc );
            double error_d = nc.dot( cornd );


            std::cout << "error_a = " << error_a << std::endl;
            std::cout << "error_b = " << error_b << std::endl;
            std::cout << "error_c = " << error_c << std::endl;
            std::cout << "error_d = " << error_d << std::endl;
            if( error_a > 0 && error_b > 0 && error_c > 0 && error_d > 0 )
                continue;
            if( error_a < 0 && error_b < 0 && error_c < 0 && error_d < 0 )
                continue;

            descr_last.push_back( map_line->GetDescriptor() );

//            ori_idex.push_back(i);
        }


//        std::cout << "LastFrame.mDescriptorLine = " << LastFrame.mDescriptorLine.rows << std::endl;
        auto descr_cur = CurrentFrame.mDescriptorLine;
        std::cout << "descr_last = " << descr_last.rows << std::endl;
        if( descr_last.empty() || descr_cur.empty() ) return nmatches;

        return 0;

        vector<vector<cv::DMatch>> lmatches;
        auto bm = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
        bm->knnMatch(descr_cur, descr_last, lmatches, 2);

        /* select best matches */
        std::vector<cv::DMatch> good_matches;
        for (auto & lmatche : lmatches)
        {
            if( lmatche[0].distance < TH_LOW && lmatche[0].distance < lmatche[1].distance * mnnratio  ){

//                CurrentFrame.mvpMapLines[ lmatche[0].queryIdx ] = vpMapLine[ lmatche[0].trainIdx ];
                nmatches++;
            }
        }

        std::cout << "LastFrame nmatches = " << nmatches << std::endl;

        return nmatches;
    }
}