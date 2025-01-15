#ifndef KINEMATICS_TPP
#define KINEMATICS_TPP

#include <iostream>
#include "maths/curves.hpp"
#include "miscellaneous/conversion.hpp"

namespace cMRKinematics
{
    template <int DOF>
    ArmKinematicsSolver<DOF>::ArmKinematicsSolver(const rclcpp::Node::SharedPtr &node,
                                                  const std::string &urdf_param,
                                                  const std::string &root_link,
                                                  const std::string &tip_link, bool flag)
        : node_(node), root_link_(root_link), tip_link_(tip_link)
    {
        std::string robot_desc_string;
        if (flag)
            node_->declare_parameter(urdf_param, std::string());
        node_->get_parameter(urdf_param, robot_desc_string);

        if (!kdl_parser::treeFromString(robot_desc_string, robot_tree_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to construct KDL Tree from param: %s",
                         urdf_param.c_str());
        }

        if (!robot_tree_.getChain(root_link_, tip_link_, chain_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get KDL Chain from %s to %s",
                         root_link_.c_str(), tip_link_.c_str());
        }

        dof_num_ = chain_.getNrOfJoints();
        if (dof_num_ != DOF)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Chain has %d joints, but template DOF=%d. This might be inconsistent.",
                        dof_num_, DOF);
        }

        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);

        q.resize(dof_num_);
        initial_guess.resize(dof_num_);

        if (DOF == 7)
        {
            state_info_.joint_states[0] = initial_guess(0) = q(0) = 0.5;
            state_info_.joint_states[1] = initial_guess(1) = q(1) = 1.396;
            state_info_.joint_states[2] = initial_guess(2) = q(2) = -1.15;
            state_info_.joint_states[3] = initial_guess(3) = q(3) = -0.65;
            state_info_.joint_states[4] = initial_guess(4) = q(4) = 1.5;
            state_info_.joint_states[5] = initial_guess(5) = q(5) = 0.5;
            state_info_.joint_states[6] = initial_guess(6) = q(6) = -1.5;
        }
        else if (DOF == 6)
        {
            initial_guess(0) = q(0) = 1.396;
            initial_guess(1) = q(1) = -1.15;
            initial_guess(2) = q(2) = -0.65;
            initial_guess(3) = q(3) = 1.5;
            initial_guess(4) = q(4) = 0.5;
            initial_guess(5) = q(5) = -1.5;
        }

        SolveFK(initial_guess);

        RCLCPP_INFO(node_->get_logger(),
                    "ArmKinematicsSolver<%d> constructed with dof_num=%d (root=%s, tip=%s)",
                    DOF, dof_num_, root_link_.c_str(), tip_link_.c_str());
    }

    template <int DOF>
    int ArmKinematicsSolver<DOF>::GetDOF()
    {
        return dof_num_;
    }

    template <int DOF>
    void ArmKinematicsSolver<DOF>::SolveFK(const KDL::JntArray &joint_positions)
    {
        if ((int)joint_positions.rows() != dof_num_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[ArmKinematicsSolver<%d>] SolveFK got a mismatch in JntArray size.",
                         DOF);
            return;
        }

        if (fk_solver_->JntToCart(joint_positions, sixDpose) < 0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[ArmKinematicsSolver<%d>] Failed to solve forward kinematics.",
                         DOF);
        }
    }

    template <int DOF>
    void ArmKinematicsSolver<DOF>::SolveFKAllLinks(const KDL::JntArray &joint_positions,
                                                   std::vector<KDL::Frame> &all_link_poses)
    {
        if ((int)joint_positions.rows() != dof_num_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[ArmKinematicsSolver<%d>] SolveFKAllLinks got mismatch in JntArray size.",
                         DOF);
            return;
        }

        const int num_segments = chain_.getNrOfSegments();
        all_link_poses.resize(num_segments);

        KDL::Frame tempFrame;
        for (int i = 0; i < num_segments; ++i)
        {
            int ret = fk_solver_->JntToCart(joint_positions, tempFrame, i + 1);
            if (ret < 0)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[ArmKinematicsSolver<%d>] Failed to compute FK for segment %d",
                             DOF, i + 1);
                continue;
            }
            all_link_poses[i] = tempFrame;
        }
    }

    template <int DOF>
    bool ArmKinematicsSolver<DOF>::SolveIKIntermediate(const KDL::Frame &end_effector_pose)
    {
        int result = ik_solver_->CartToJnt(initial_guess, end_effector_pose, q);
        if (result != 0)
        {
            return false;
        }
        // On success, update the initial guess
        initial_guess = q;
        return true;
    }

    template <int DOF>
    bool ArmKinematicsSolver<DOF>::SolveIK(const KDL::Frame &target_pose)
    {
        // Start by solving FK for the initial guess to get the current end-effector pose
        SolveFK(initial_guess);
        Eigen::Matrix4d T1 = Conversions::KDL_2Transform(this->sixDpose);
        Eigen::Matrix4d T2 = Conversions::KDL_2Transform(target_pose);

        // We'll do a simple line interpolation
        Curves::Line L(T1.block<4, 1>(0, 3), T2.block<4, 1>(0, 3));
        bool success = true;
        if (L.path_lenght > 0.01)
        {
            double precision = 0.01;
            auto pose_vector = L.Generate6DPoints(T1, T2, precision);

            for (const auto &pv : pose_vector)
            {
                KDL::Frame intermediate_frame;
                intermediate_frame.p = KDL::Vector(pv.first(0), pv.first(1), pv.first(2));

                Eigen::Matrix3d rot_matrix = pv.second.toRotationMatrix();
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        intermediate_frame.M(i, j) = rot_matrix(i, j);

                // Attempt IK
                if (!SolveIKIntermediate(intermediate_frame))
                {
                    success = false;
                    break;
                }
            }
        }
        else
        {
            // If distance is small, just do direct
            success = SolveIKIntermediate(target_pose);
        }

        if (!success)
        {
            // RCLCPP_WARN(node_->get_logger(),
            //             "[ArmKinematicsSolver<%d>] SolveIK path had an IK failure mid-way.",
            //             DOF);
            return false;
        }

        // If we made it, we set initial_guess to the final q:
        initial_guess = q;
        return true;
    }

}

#endif
