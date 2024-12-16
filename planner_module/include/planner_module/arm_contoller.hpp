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

#include "pc_processing/octomap_generator.hpp"
#include "custom_interfaces/msg/table_vertices.hpp"

using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace arm_planner
{
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    extern std::vector<geometry_msgs::msg::Point> table_vertices_;

    class ArmController
    {
    public:
        ArmController(const rclcpp::Node::SharedPtr &node);
        ~ArmController() = default;

    private:
        rclcpp::Node::SharedPtr node_;
        GoalHandleFollowJointTrajectory::SharedPtr arm_goal_hangle_;
        std::string arm_goal_pose_name_, yaml_file_;

        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint_trajectory_action_client_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub_;
        rclcpp::Subscription<custom_interfaces::msg::TableVertices>::SharedPtr table_vertices_sub_;

        void proceedToNextViewpoint(std::string str);
        void triggerSnapshotForCurrentViewpoint(bool stitch);
        void TableVerticesCallback(const custom_interfaces::msg::TableVertices::ConstSharedPtr &table_msg);

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

        static std::mutex m;
        static std::condition_variable cv;
        static bool callback_triggered;

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
    };
}

#endif
