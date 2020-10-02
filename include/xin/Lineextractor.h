//
// Created by xin on 2020/10/2.
//

#ifndef ORB_SLAM2_LINEEXTRACTOR_H
#define ORB_SLAM2_LINEEXTRACTOR_H

#include <vector>

#include <opencv2/line_descriptor.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

namespace ORB_SLAM2
{
    struct sort_flines_by_length
    {
        inline bool operator()(const cv::Vec4f& a, const cv::Vec4f& b){
            return ( sqrt(pow(a(0)-a(2),2.0)+pow(a(1)-a(3),2.0)) > sqrt(pow(b(0)-b(2),2.0)+pow(b(1)-b(3),2.0)) );
        }
    };

    enum class LineExtractType
    {
        LSD = 0,
        FLD,
        TP_LSD
    };

    class LineExtractor
    {
    public:
        ~LineExtractor() = default;

        virtual void operator()
        (const cv::Mat& im, std::vector<cv::line_descriptor::KeyLine> &line, cv::Mat& lbd_descr)
        = 0;


    private:

    };

    class FLDExtractor : public LineExtractor
    {
    public:
        FLDExtractor( int nLineFeatures, int nminLinelength );
        void operator()
                (const cv::Mat& im, std::vector<cv::line_descriptor::KeyLine> &line, cv::Mat& lbd_descr) override;

    private:
        cv::Ptr<cv::ximgproc::FastLineDetector> fld_;
        cv::Ptr<cv::line_descriptor::BinaryDescriptor> lbd_;

        int nLineFeatures_;
        int nminLineLength_;
    };

    class LSDExtractor : public LineExtractor
    {
    public:
        LSDExtractor( int nLineFeatures, int nminLinelength );
        void operator()
                (const cv::Mat& im, std::vector<cv::line_descriptor::KeyLine> &line, cv::Mat& lbd_descr) override;

    private:
//        cv::Ptr<cv::line_descriptor::LSDDetectorC> lsd_;
//        cv::Ptr<cv::line_descriptor::BinaryDescriptor> lbd_;

        int nLineFeatures_;
        int nminLineLength_;
    };
}


#endif //ORB_SLAM2_LINEEXTRACTOR_H
