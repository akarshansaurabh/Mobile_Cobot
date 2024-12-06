
#include "planner_module/kinematics.hpp"
#include "maths/curves.hpp"
#include "miscellaneous/conversion.hpp"

namespace cMRKinematics
{
    StateInformation state_info_;

    ArmKinematicsSolver::ArmKinematicsSolver(const std::string &urdf_param, const std::string &root_link, const std::string &tip_link)
        : Node("arm_kinematics_solver"), root_link_(root_link), tip_link_(tip_link)
    {
        std::string robot_desc_string;
        this->declare_parameter(urdf_param, std::string());
        this->get_parameter(urdf_param, robot_desc_string);

        if (!kdl_parser::treeFromString(robot_desc_string, robot_tree_))
            RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL Tree");

        if (!robot_tree_.getChain(root_link_, tip_link_, chain_))
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL Chain from tree");

        dof_num = chain_.getNrOfJoints();
        q.resize(dof_num);
        jacobian.resize(dof_num);
        initial_guess.resize(dof_num);
        tolerance = 0.001;

        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
        jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);

        state_info_.joint_angles[0] = initial_guess(0) = q(0) = 1.396;
        state_info_.joint_angles[1] = initial_guess(1) = q(1) = -1.15;
        state_info_.joint_angles[2] = initial_guess(2) = q(2) = -0.65;
        state_info_.joint_angles[3] = initial_guess(3) = q(3) = 1.5;
        state_info_.joint_angles[4] = initial_guess(4) = q(4) = 0.5;
        state_info_.joint_angles[5] = initial_guess(5) = q(5) = -1.5;

        q_home = initial_guess;
        // fk for home position -> rot_6_wrt_0 at home position
        SolveFK(initial_guess);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                pose_6_wrt_0(i, j) = sixDpose.M(i, j);
        state_info_.xyz_rpy[0] = pose_6_wrt_0(0, 3) = sixDpose.p.x();
        state_info_.xyz_rpy[1] = pose_6_wrt_0(1, 3) = sixDpose.p.y();
        state_info_.xyz_rpy[2] = pose_6_wrt_0(2, 3) = sixDpose.p.z();
        pose_6_wrt_0(3, 3) = 1.0;

        double roll, pitch, yaw;
        sixDpose.M.GetRPY(roll, pitch, yaw);
        state_info_.xyz_rpy[3] = roll;
        state_info_.xyz_rpy[4] = pitch;
        state_info_.xyz_rpy[5] = yaw;
    }

    bool ArmKinematicsSolver::SolveIKIntermediate(const KDL::Frame &end_effector_pose)
    {
        int result = ik_solver_->CartToJnt(initial_guess, end_effector_pose, q);
        if (result != 0)
            return false;

        initial_guess = q;
        return true;
    }

    int ArmKinematicsSolver::GetDOF()
    {
        return dof_num;
    }

    void ArmKinematicsSolver::SolveFK(const KDL::JntArray &joint_positions)
    {
        if (fk_solver_->JntToCart(joint_positions, sixDpose) < 0)
            cout << "Failed to solve forward kinematics" << endl;
    }

    bool ArmKinematicsSolver::SolveIK(const KDL::Frame &target_pose)
    {
        SolveFK(initial_guess);
        // KDL::Frame current_kdl_frame = sixDpose;
        // KDL::Frame target_kdl_frame = target_pose;
        Eigen::Matrix4d T1 = Conversions::KDL_2Transform(this->sixDpose);
        Eigen::Matrix4d T2 = Conversions::KDL_2Transform(target_pose);

        bool flag;
        Curves::Line L(T1.block<4, 1>(0, 3), T2.block<4, 1>(0, 3));
        if (L.path_lenght > 0.01)
        {
            double precision = 0.01;
            vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> pose_vector = L.Generate6DPoints(T1, T2, precision);

            for (int i = 0; i < pose_vector.size(); i++)
            {
                KDL::Frame intermediate_frame;
                intermediate_frame.p = KDL::Vector(pose_vector[i].first(0), pose_vector[i].first(1), pose_vector[i].first(2));
                Eigen::Matrix3d rot_matrix = pose_vector[i].second.toRotationMatrix();
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        intermediate_frame.M(i, j) = rot_matrix(i, j);
                flag = SolveIKIntermediate(intermediate_frame);
            }

            if (flag == false)
                return false;
            initial_guess = q;
            return true;
        }
        else
        {
            flag = SolveIKIntermediate(target_pose);
            if (flag == false)
                return false;
            initial_guess = q;
            return true;
        }
    }

    bool ArmKinematicsSolver::SingularityExists(const KDL::JntArray &joint_positions)
    {
        KDL::Jacobian J(6);
        int result = jacobian_solver_->JntToJac(joint_positions, J);
        if (result < 0)
        {
            cout << "J doest not exist" << endl;
            return true;
        }

        Eigen::Matrix<double, 6, 6> jacobian_matrix = J.data;

        if (jacobian_matrix.determinant() < tolerance)
            return true;

        return false;
    }
}
