#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

#include <iostream>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chain.hpp>
#include "maths/commonmathssolver.hpp"
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std;

namespace Conversions
{
    const double rad_to_deg = 180.0 / M_PI;
    const double deg_to_rad = M_PI / 180.0;
    const double mm_to_meter = 0.001;
    const double meter_to_mm = 1000.0;

    inline KDL::Frame Transform_2KDL(const Eigen::Matrix4d &T)
    {
        KDL::Frame kdl_frame;
        kdl_frame.p = KDL::Vector(T(0, 3), T(1, 3), T(2, 3));
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                kdl_frame.M(i, j) = T(i, j);
        return kdl_frame;
    }

    inline Eigen::Matrix4d KDL_2Transform(const KDL::Frame &kdl_frame)
    {
        Eigen::Matrix4d T;
        T << kdl_frame.M(0, 0), kdl_frame.M(0, 1), kdl_frame.M(0, 2), kdl_frame.p.x(),
            kdl_frame.M(1, 0), kdl_frame.M(1, 1), kdl_frame.M(1, 2), kdl_frame.p.y(),
            kdl_frame.M(2, 0), kdl_frame.M(2, 1), kdl_frame.M(2, 2), kdl_frame.p.z(),
            0, 0, 0, 1;
        return T;
    }

    inline Eigen::Matrix4d Pair_2Transform(const pair<Eigen::Vector4d, Eigen::Vector3d> &pair)
    {
        Eigen::Matrix4d T;
        T.block<3, 3>(0, 0) = CommonMathsSolver::OrientationNTransformaton::ComputeR(pair.second);
        T.block<4, 1>(0, 3) = pair.first;
        T.block<1, 3>(3, 0).setZero();
        return T;
    }

    inline Eigen::Matrix4d TransformStamped_2Eigen(const geometry_msgs::msg::TransformStamped &transform_stamped)
    {
        Eigen::Matrix4d T;
        Eigen::Quaterniond Q(transform_stamped.transform.rotation.w,
                             transform_stamped.transform.rotation.x,
                             transform_stamped.transform.rotation.y,
                             transform_stamped.transform.rotation.z);
        T.block<3, 3>(0, 0) = Q.toRotationMatrix();
        T(0, 3) = transform_stamped.transform.translation.x;
        T(1, 3) = transform_stamped.transform.translation.y;
        T(2, 3) = transform_stamped.transform.translation.z;
        T(3, 0) = T(3, 1) = T(3, 2) = 0.0;
        T(3, 3) = 1.0;
        return T;
    }

    inline Eigen::Matrix4d Pose_2Eigen(const geometry_msgs::msg::Pose &pose)
    {
        Eigen::Matrix4d T;
        Eigen::Quaterniond Q(pose.orientation.w,
                             pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z);
        T.block<3, 3>(0, 0) = Q.toRotationMatrix();
        T(0, 3) = pose.position.x;
        T(1, 3) = pose.position.y;
        T(2, 3) = pose.position.z;
        T(3, 0) = T(3, 1) = T(3, 2) = 0.0;
        T(3, 3) = 1.0;
        return T;
    }

    inline Eigen::Matrix3f KDL_2Rot(const KDL::Frame &kdl_frame)
    {
        Eigen::Matrix3f mat;
        mat(0, 0) = static_cast<float>(kdl_frame.M(0, 0));
        mat(0, 1) = static_cast<float>(kdl_frame.M(0, 1));
        mat(0, 2) = static_cast<float>(kdl_frame.M(0, 2));

        mat(1, 0) = static_cast<float>(kdl_frame.M(1, 0));
        mat(1, 1) = static_cast<float>(kdl_frame.M(1, 1));
        mat(1, 2) = static_cast<float>(kdl_frame.M(1, 2));

        mat(2, 0) = static_cast<float>(kdl_frame.M(2, 0));
        mat(2, 1) = static_cast<float>(kdl_frame.M(2, 1));
        mat(2, 2) = static_cast<float>(kdl_frame.M(2, 2));
        return mat;
    }
}

#endif