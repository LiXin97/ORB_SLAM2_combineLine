//
// Created by xin on 2020/10/5.
//

#ifndef ORB_SLAM2_COMBIANLINE_LINEPARAM_H
#define ORB_SLAM2_COMBIANLINE_LINEPARAM_H

#include <eigen3/Eigen/Dense>
#include "xin/LineGeometry.hpp"
#include "xin/Ulity.hpp"

namespace ORB_SLAM2
{
    class Plucker
    {
    public:
        Plucker( const Eigen::Vector4d& p0, const Eigen::Vector4d& p1 )
        {
            std::tie( norm_, dirction_ ) = LineGeo::plk_from_pp( p0, p1 );
        }
        Plucker(const Eigen::Vector3d& norm, const Eigen::Vector3d& dirction)
                :norm_(norm),dirction_(dirction)
        {}
        explicit Plucker( const Eigen::Vector4d& orth )
        {
            auto [norm, dir] = Orth2Plucker(orth);
//            norm_ = norm;
//            dirction_ = dir;
            new (this)Plucker( norm, dir );
        }
        Plucker() = default;
        Plucker(const Plucker& _plucker) = default;

        Eigen::Vector4d GetOrth();

        static Eigen::Vector4d Plucker2Orth( const Eigen::Vector3d& norm, const Eigen::Vector3d& dirction );

        static std::tuple<Eigen::Vector3d, Eigen::Vector3d>
                Orth2Plucker( const Eigen::Vector4d& Orth );

        // transform from ori to new
        void plk_transform( const Eigen::Matrix3d& Rno, const Eigen::Vector3d& tno ) {
            norm_ = Rno * norm_ + Ulity::skewSymmetric(tno) * Rno * dirction_;
            dirction_ = Rno * dirction_;
        }

        Plucker Get_plk_transform( const Eigen::Matrix3d& Rno, const Eigen::Vector3d& tno ) {
            Eigen::Vector3d norm = Rno * norm_ + Ulity::skewSymmetric(tno) * Rno * dirction_;
            Eigen::Vector3d dirction = Rno * dirction_;
            return Plucker( norm, dirction );
        }

        [[nodiscard]] std::tuple<Eigen::Vector3d, Eigen::Vector3d>
        Get_nd_transform( const Eigen::Matrix3d& Rno, const Eigen::Vector3d& tno ) const {
            Eigen::Vector3d norm = Rno * norm_ + Ulity::skewSymmetric(tno) * Rno * dirction_;
            Eigen::Vector3d dirction = Rno * dirction_;
            return std::make_tuple( norm, dirction );
        }

        std::tuple<Eigen::Vector3d, Eigen::Vector3d>
        Get3D( const Eigen::Vector3d& ob0,
               const Eigen::Vector3d& ob1);

        std::tuple<Eigen::Vector3d, Eigen::Vector3d>
        Get3D( const Eigen::Matrix3d& Rwc,
               const Eigen::Vector3d& twc,
               const Eigen::Vector3d& ob0,
               const Eigen::Vector3d& ob1);

        [[nodiscard]] std::tuple<Eigen::Vector3d, Eigen::Vector3d>
        Get_nd(  ) {
            return std::make_tuple( norm_, dirction_ );
        }

        [[nodiscard]] Eigen::Vector3d GetNorm( ) const {return norm_;}
        [[nodiscard]] Eigen::Vector3d GetDir( ) const {return dirction_;}

    private:
        Eigen::Vector3d norm_;
        Eigen::Vector3d dirction_;
    };
}


#endif //ORB_SLAM2_COMBIANLINE_LINEPARAM_H
