//
// Created by xin on 2020/10/2.
//

#include "include/xin/MapLine.h"

namespace ORB_SLAM2
{

    bool MapLine::isBad()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mbBad;
    }
}