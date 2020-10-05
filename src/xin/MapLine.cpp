//
// Created by xin on 2020/10/2.
//

#include "include/xin/MapLine.h"

namespace ORB_SLAM2
{
    long unsigned int MapLine::nNextId=0;
    mutex MapLine::mGlobalMutex;
    MapLine::MapLine(const Plucker &plucker, KeyFrame *pRefKF, Map *pMap)
    :plucker_(plucker),
     mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0),
     mnTrackReferenceForFrame(0), mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0),
//     mnLoopPointForKF(0), mnCorrectedByKF(0),
//     mnCorrectedReference(0), mnBAGlobalForKF(0),
     mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
//     mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0),
     mpMap(pMap)

    {
        mNormalVector = cv::Mat::zeros(3,1,CV_32F);

        // Maplines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        unique_lock<mutex> lock(mpMap->mMutexPointCreation);
        mnId=nNextId++;
    }

    void MapLine::AddObservation(KeyFrame *pKF, size_t idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return;
        mObservations[pKF]=idx;

        // TODO stereo line remain
//        if(pKF->mvuRight[idx]>=0)
//            nObs+=2;
//        else
            nObs++;
    }

    void MapLine::EraseObservation(KeyFrame *pKF)
    {
        bool bBad=false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if(mObservations.count(pKF))
            {
                // TODO stereo line remain
//                int idx = mObservations[pKF];
//                if(pKF->mvuRight[idx]>=0)
//                    nObs-=2;
//                else
                    nObs--;

                mObservations.erase(pKF);

                if(mpRefKF==pKF)
                    mpRefKF=mObservations.begin()->first;

                // If only 2 observations or less, discard point
                if(nObs<=2)
                    bBad=true;
            }
        }

        if(bBad)
            SetBadFlag();
    }

    cv::Mat MapLine::GetDescriptor()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }

    void MapLine::ComputeDistinctiveDescriptors()
    {
        // Retrieve all observed descriptors
        std::vector<cv::Mat> vDescriptors;

        map<KeyFrame*,size_t> observations;

        {
            unique_lock<mutex> lock1(mMutexFeatures);
            if(mbBad)
                return;
            observations=mObservations;
        }

        if(observations.empty())
            return;

        vDescriptors.reserve(observations.size());

        for(auto & observation : observations)
        {
            KeyFrame* pKF = observation.first;

            if(!pKF->isBad())
                vDescriptors.push_back(pKF->mDescriptorLine.row(observation.second));
        }

        if(vDescriptors.empty())
            return;

        // Compute distances between them
        const size_t N = vDescriptors.size();

        float Distances[N][N];
        for(size_t i=0;i<N;i++)
        {
            Distances[i][i]=0;
            for(size_t j=i+1;j<N;j++)
            {
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
                Distances[i][j]=distij;
                Distances[j][i]=distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for(size_t i=0;i<N;i++)
        {
            vector<int> vDists(Distances[i],Distances[i]+N);
            sort(vDists.begin(),vDists.end());
            int median = vDists[0.5*(N-1)];

            if(median<BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }

        {
            unique_lock<mutex> lock(mMutexFeatures);
            mDescriptor = vDescriptors[BestIdx].clone();
        }
    }

    void MapLine::SetBadFlag()
    {
        map<KeyFrame*,size_t> obs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad=true;
            obs = mObservations;
            mObservations.clear();
        }
        for(auto & ob : obs)
        {
            KeyFrame* pKF = ob.first;
            pKF->EraseMapLineMatch(ob.second);
        }

        mpMap->EraseMapLine(this);
    }

    int MapLine::GetIndexInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    bool MapLine::isBad()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mbBad;
    }

    void MapLine::SetOrth(const Eigen::Vector4d &Orth)
    {
        auto [norm, dir] = Plucker::Orth2Plucker(Orth);
        double dir_n = dir.norm();
        norm /= dir_n;
        dir /= dir_n;
        plucker_ = Plucker( norm, dir );
    }

    bool MapLine::IsInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    double MapLine::GetFoundRatio()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<double>(mnFound)/static_cast<double>(mnVisible);
    }

    void MapLine::Update3D()
    {
        Eigen::Vector3d startPoint3dSum(0.,0.,0.);
        Eigen::Vector3d endPoint3dSum(0.,0.,0.);

        for( auto& obser:mObservations )
        {
            auto index = obser.second;
            auto KF = obser.first;

            Eigen::Vector3d twc = Converter::toVector3d( KF->GetCameraCenter() );
            Eigen::Matrix3d Rcw = Converter::toMatrix3d( KF->GetRotation() );
            Eigen::Vector3d tcw = Converter::toVector3d( KF->GetTranslation() );
            Eigen::Matrix3d Rwc = Rcw.transpose();

            const float &cx = KF->cx;
            const float &cy = KF->cy;
            const float &invfx = KF->invfx;
            const float &invfy = KF->invfy;

            Eigen::Vector3d ob0, ob1;
            {
                auto line1 = KF->mvKeyLinesUn[index];
                cv::Point2f startPoint = line1.getStartPoint();
                cv::Point2f endPoint = line1.getEndPoint();
                ob0 = Eigen::Vector3d( startPoint.x, startPoint.y, 1. );
                ob1 = Eigen::Vector3d( endPoint.x, endPoint.y, 1. );
                ob0(0) = (ob0(0) - cx) * invfx;
                ob0(1) = (ob0(1) - cy) * invfy;
                ob1(0) = (ob1(0) - cx) * invfx;
                ob1(1) = (ob1(1) - cy) * invfy;
            }

            auto plucker_cam = plucker_.Get_plk_transform( Rcw, tcw );
            auto [starP3d, endP3d] = plucker_cam.Get3D( ob0, ob1 );
            starP3d =  Rwc * starP3d + twc;
            endP3d =  Rwc * endP3d + twc;

//            std::cout << "starP3d = " << starP3d.transpose() << std::endl;
//            std::cout << "endP3d = " << endP3d.transpose() << std::endl;
//            std::cout << "dir = " << (starP3d - endP3d).normalized().transpose() << std::endl;
//            std::cout << "len = " << (starP3d - endP3d).norm() << std::endl;

            Eigen::Vector3d dir_3d = (starP3d - endP3d).normalized();
            auto dir_plk = plucker_.GetDir();
            double equ_dir = dir_3d.dot( dir_plk );
            const double thred = .998;
            if( equ_dir > -thred && equ_dir < thred )
            {
                std::cerr << "dir_3d.dot( dir ) = " <<  equ_dir << std::endl;
                std::cerr << "theta = " <<  std::acos(equ_dir) / M_PI * 180. << std::endl;
            }
            assert( !( equ_dir > -thred && equ_dir < thred ) );

            if( equ_dir < -thred )
            {
                startPoint3dSum += endP3d;
                endPoint3dSum += starP3d;
            }
            else
            {
                startPoint3dSum += starP3d;
                endPoint3dSum += endP3d;
            }
        }

//        std::cout << "len = " << (mstartPoint3d_ - mendPoint3d_).norm() << std::endl;
//        std::cout << "=-=-=-=-=-=-=-" << std::endl;
        mstartPoint3d_ = startPoint3dSum / mObservations.size();
        mendPoint3d_ = endPoint3dSum / mObservations.size();
    }

    void MapLine::IncreaseVisible(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible+=n;
    }

    void MapLine::IncreaseFound(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound+=n;
    }

    int MapLine::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    std::map<KeyFrame *, size_t> MapLine::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }
}