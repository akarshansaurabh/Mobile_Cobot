#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Dense>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chain.hpp>

#include <array>

using namespace std;

namespace cMRKinematics
{
    struct StateInformation
    {
        std::array<double, 6> joint_angles, xyz_rpy;
        double t;
    };

    extern StateInformation state_info_;

    class ArmKinematicsSolver : public rclcpp::Node
    {
    private:
        int dof_num;
        KDL::Tree robot_tree_;
        KDL::Chain chain_;
        std::string root_link_;
        std::string tip_link_;
        KDL::Jacobian jacobian;

        double tolerance;
        Eigen::Matrix<double, 6, 6> Jm;

        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
        std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;

        bool SolveIKIntermediate(const KDL::Frame &end_effector_pose);

    public:
        KDL::Frame sixDpose;
        KDL::JntArray initial_guess, q, q_home;
        Eigen::Matrix4d pose_6_wrt_0;
        ArmKinematicsSolver(const std::string &urdf_param, const std::string &root_link, const std::string &tip_link);
        int GetDOF();
        void SolveFK(const KDL::JntArray &joint_positions);
        bool SolveIK(const KDL::Frame &target_pose);
        bool SingularityExists(const KDL::JntArray &joint_positions);
    };
}

#endif