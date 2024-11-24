#ifndef PLANNER_MODULE_ARM_CONTROLLER_HPP
#define PLANNER_MODULE_ARM_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace arm_planner
{
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    class ArmController : public rclcpp::Node
    {
    public:
        ArmController();
        ~ArmController() = default;

    private:
        GoalHandleFollowJointTrajectory::SharedPtr arm_goal_hangle_;
        std::string arm_goal_pose_name_;

        // Action Client
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint_trajectory_action_client_;

        // Parameters

        // Methods
        void SendJointTrajectoryGoal(const trajectory_msgs::msg::JointTrajectory &trajectory);
        trajectory_msgs::msg::JointTrajectory CreateJointTrajectory(const std::vector<double> &positions, double execution_time);

        // Action Callbacks
        void JointTrajectoryGoalResponseCallback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle);
        void JointTrajectoryFeedbackCallback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle,
                                             const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback);
        void JointTrajectoryResultCallback(const GoalHandleFollowJointTrajectory::WrappedResult &result);

        // ros2 parameter callbacks
        rcl_interfaces::msg::SetParametersResult ArmGoalUpdatedCallback(const std::vector<rclcpp::Parameter> &parameters);
        std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> parameter_callback_handle_; // used for parameter registration
        vector<double> GetArmGoalPose(const std::string &filename, const std::string &pose_);
    };
}

#endif
