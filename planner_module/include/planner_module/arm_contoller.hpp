#ifndef PLANNER_MODULE_ARM_CONTROLLER_HPP
#define PLANNER_MODULE_ARM_CONTROLLER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace arm_planner
{
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    class ArmController
    {
    public:
        ArmController(const rclcpp::Node::SharedPtr &node);
        ~ArmController() = default;

        // Flag that is set by external code (e.g. nav2) to start the snapshot process
        std::atomic<bool> start_taking_snapshots_;

    private:
        rclcpp::Node::SharedPtr node_;
        GoalHandleFollowJointTrajectory::SharedPtr arm_goal_hangle_;
        std::string arm_goal_pose_name_, yaml_file_;

        // Action Client
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint_trajectory_action_client_;

        // To track the sequence of viewpoints c1, c2, c3
        std::vector<std::string> viewpoint_sequence_;
        std::size_t current_viewpoint_index_;
        bool sequence_in_progress_;

        void proceedToNextViewpoint(std::string str);
        void triggerSnapshotForCurrentViewpoint();
        // void finishSnapshotSequence();

        // Methods
        void SendJointTrajectoryGoal(const trajectory_msgs::msg::JointTrajectory &trajectory);
        trajectory_msgs::msg::JointTrajectory CreateJointTrajectory(const std::vector<double> &positions, double execution_time);
        vector<double> GetArmGoalPose(const std::string &filename, const std::string &pose_);

        // Action Callbacks
        void JointTrajectoryGoalResponseCallback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle);
        void JointTrajectoryFeedbackCallback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle,
                                             const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback);
        void JointTrajectoryResultCallback(const GoalHandleFollowJointTrajectory::WrappedResult &result);

        // ros2 parameter callbacks
        rcl_interfaces::msg::SetParametersResult ArmGoalUpdatedCallback(const std::vector<rclcpp::Parameter> &parameters);
        std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> parameter_callback_handle_; // used for parameter registration
    };
}

#endif
