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
        std::array<double, 7> joint_states;
        double t;
    };

    extern StateInformation state_info_;

    class ArmKinematicsSolver
    {
    private:
        int dof_num;
        KDL::Tree robot_tree_;
        KDL::Chain chain_;
        std::string root_link_;
        std::string tip_link_;
        rclcpp::Node::SharedPtr node_;
        // KDL::Jacobian jacobian;

        // double tolerance;
        // Eigen::Matrix<double, 6, 6> Jm;

        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
        // std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;

        bool SolveIKIntermediate(const KDL::Frame &end_effector_pose);

    public:
        KDL::Frame sixDpose;
        KDL::JntArray initial_guess, q;
        // Eigen::Matrix4d pose_6_wrt_0;
        ArmKinematicsSolver(const rclcpp::Node::SharedPtr &node, const std::string &urdf_param,
                            const std::string &root_link, const std::string &tip_link);
        int GetDOF();
        void SolveFK(const KDL::JntArray &joint_positions);
        bool SolveIK(const KDL::Frame &target_pose);
        void SolveFKAllLinks(const KDL::JntArray &joint_positions, std::vector<KDL::Frame> &all_link_poses);

        // bool SingularityExists(const KDL::JntArray &joint_positions);
    };
}

#endif