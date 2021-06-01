//
// Created by lixin04 on 19年12月6日.
//
#pragma once
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <ceres/rotation.h>


//struct GetPixelGrayValue {
//    GetPixelGrayValue(const float pixel_gray_val_in[16],
//                      const int rows,
//                      const int cols,
//                      const std::vector<float>& vec_pixel_gray_values)
//    {
//        for (int i = 0; i < 16; i++)
//            pixel_gray_val_in_[i] = pixel_gray_val_in[i];
//        rows_ = rows;
//        cols_ = cols;                // 初始化grid2d
//         grid2d.reset(
//                 new ceres::Grid2D<float>(
//                         &vec_pixel_gray_values[0], 0, rows_, 0, cols_));                //双三次插值
//         get_pixel_gray_val.reset(
//                 new ceres::BiCubicInterpolator<ceres::Grid2D<float> >(*grid2d));
//    }
//    template <typename T>bool operator()(
//            const T* const so3t,              //模型参数，位姿，6维
//            const T* const xyz,             //模型参数，3D点，3维
//            T* residual ) const              //残差，4*4图像块，16维
//    {
//        // 计算变换后的u和v
//        T u_out, v_out, pt[3], r[3];
//        r[0] = so3t[0];
//        r[1] = so3t[1];
//        r[2] = so3t[2];
//        ceres::AngleAxisRotatePoint(r, xyz, pt);
//        pt[0] += so3t[3];
//        pt[1] += so3t[4];
//        pt[2] += so3t[5];
//        u_out = pt[0] * T(fx) / pt[2] + T(cx);
//        v_out = pt[1] * T(fy) / pt[2] + T(cy);
//        for (int i = 0; i < 16; i++)
//        {
//            int m = i / 4;
//            int n = i % 4;
//            T u, v, pixel_gray_val_out;
//            u = u_out + T(m - 2);
//            v = v_out + T(n - 2);
//            get_pixel_gray_val->Evaluate(u, v, &pixel_gray_val_out);
//            residual[i] = T(pixel_gray_val_in_[i]) - pixel_gray_val_out;
//        }
//        return true;
//    }
//
//    float pixel_gray_val_in_[16];
//    int rows_,cols_;
//    std::unique_ptr<ceres::Grid2D<float> > grid2d;
//    std::unique_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<float> > > get_pixel_gray_val;
//};

struct MonoProjectionAuto {
    MonoProjectionAuto(double observed_x, double observed_y)
            : observed_x(observed_x), observed_y(observed_y) {}

    template <typename T>
    bool operator()(const T* const Tcw,
                    const T* const point,
                    T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];

        ceres::QuaternionRotatePoint(Tcw, point, p);

//        ceres::AngleAxisRotatePoint(camera, point, p);
        // camera[3,4,5] are the translation.
        p[0] += Tcw[3]; p[1] += Tcw[4]; p[2] += Tcw[5];

        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T predicted_x = p[0] / p[2];
        T predicted_y = p[1] / p[2];

        // Apply second and fourth order radial distortion.
//        const T& l1 = camera[7];
//        const T& l2 = camera[8];
//        T r2 = xp*xp + yp*yp;
//        T distortion = 1.0 + r2  * (l1 + l2  * r2);

        // Compute final projected point position.
//        const T& focal = camera[6];
//        T predicted_x = focal * distortion * xp;
//        T predicted_y = focal * distortion * yp;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y) {
        return (new ceres::AutoDiffCostFunction<MonoProjectionAuto, 2, 7, 3>(
                new MonoProjectionAuto(observed_x, observed_y)));
    }

    double observed_x;
    double observed_y;
};

class MonoProjection : public ceres::SizedCostFunction<2, 7, 3>
{
public:
    MonoProjection(Eigen::Vector2d& obs);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);
    static Eigen::Vector2d compute_error( double const *parameters_pose, double const *parameters_point, const Eigen::Vector2d& obs, bool& bad_point );
    static Eigen::Matrix2d sqrt_info;
    Eigen::Vector2d point_obs_;
};

class MonoLineProjection : public ceres::SizedCostFunction<2, 7, 4>
{
public:
    MonoLineProjection(Eigen::Vector4d& obs);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);
    static Eigen::Vector2d compute_error( double const *parameters_pose, double const *parameters_line, const Eigen::Vector4d& obs, bool& bad_line);;
    static Eigen::Vector2d compute_error( const Eigen::Matrix3d& Rcw, const Eigen::Vector3d& tcw,
                                          const Eigen::Vector4d& Orth, const Eigen::Vector4d& obs, bool& bad_line);
    static Eigen::Vector2d compute_error( const Eigen::Matrix3d& Rcw, const Eigen::Vector3d& tcw,
                                          const ORB_SLAM2::Plucker& plucker, const Eigen::Vector4d& obs, bool& bad_line);
    static Eigen::Matrix2d sqrt_info;
    Eigen::Vector4d line_obs_;
};