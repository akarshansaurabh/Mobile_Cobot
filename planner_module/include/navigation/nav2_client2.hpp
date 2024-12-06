#ifndef NAV2_ACTION_CLIENT_HPP_
#define NAV2_ACTION_CLIENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <yaml-cpp/yaml.h>

namespace custom_nav2_action_client
{
    // Base class for shared functionality
    class Nav2ActionClientBase
    {
    public:
        explicit Nav2ActionClientBase(rclcpp::Node::SharedPtr node);
        virtual ~Nav2ActionClientBase() = default;

    protected:
        // Shared node pointer
        rclcpp::Node::SharedPtr node_;
    };

    // Class for handling NavigateToPose action
    class NavigateToPoseClient : public Nav2ActionClientBase
    {
    public:
        using ActionType = nav2_msgs::action::NavigateToPose;
        using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

        explicit NavigateToPoseClient(rclcpp::Node::SharedPtr node);

        // Action methods with action-specific parameters
        void SendGoal(const ActionType::Goal &goal_msg);
        void CancelGoal();
        void GoalResponseCallback(const GoalHandle::SharedPtr &goal_handle);
        void FeedbackCallback(const GoalHandle::SharedPtr &goal_handle,
                              const std::shared_ptr<const ActionType::Feedback> feedback);
        void ResultCallback(const GoalHandle::WrappedResult &result);

    private:
        rclcpp_action::Client<ActionType>::SharedPtr action_client_;
        GoalHandle::SharedPtr goal_handle_;
        bool goal_active_;
    };

    // Class for handling ComputePathToPose action
    class ComputePathToPoseClient : public Nav2ActionClientBase
    {
    public:
        using ActionType = nav2_msgs::action::ComputePathToPose;
        using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

        explicit ComputePathToPoseClient(rclcpp::Node::SharedPtr node);

        // Action methods with action-specific parameters
        void SendGoal(const ActionType::Goal &goal_msg);
        void CancelGoal();
        void GoalResponseCallback(const GoalHandle::SharedPtr &goal_handle);
        void FeedbackCallback(const GoalHandle::SharedPtr &goal_handle,
                              const std::shared_ptr<const ActionType::Feedback> feedback);
        void ResultCallback(const GoalHandle::WrappedResult &result);

    private:
        rclcpp_action::Client<ActionType>::SharedPtr action_client_;
        GoalHandle::SharedPtr goal_handle_;
        bool goal_active_;
    };

    // Class for handling NavigateThroughPoses action
    class NavigateThroughPosesClient : public Nav2ActionClientBase
    {
    public:
        using ActionType = nav2_msgs::action::NavigateThroughPoses;
        using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

        explicit NavigateThroughPosesClient(rclcpp::Node::SharedPtr node);

        // Action methods with action-specific parameters
        void SendGoal(const ActionType::Goal &goal_msg);
        void CancelGoal();
        void GoalResponseCallback(const GoalHandle::SharedPtr &goal_handle);
        void FeedbackCallback(const GoalHandle::SharedPtr &goal_handle,
                              const std::shared_ptr<const ActionType::Feedback> feedback);
        void ResultCallback(const GoalHandle::WrappedResult &result);

    private:
        rclcpp_action::Client<ActionType>::SharedPtr action_client_;
        GoalHandle::SharedPtr goal_handle_;
        bool goal_active_;
    };

    // Separate class for non-action functionalities
    class Nav2Utilities
    {
    public:
        explicit Nav2Utilities(rclcpp::Node::SharedPtr node,
                               NavigateToPoseClient &client_,
                               ComputePathToPoseClient &path_client_,
                               NavigateThroughPosesClient &waypoint_client);

        // Parameter management methods
        void ActivateCostmapParameters(bool activate);
        void SetNav2Parameters();
        void ActivatePCProcessingParameters(const std::string &param);

        // Pose handling methods
        void SetInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &initial_pose);
        geometry_msgs::msg::PoseStamped GetCurrentPose();

        // Goal handling methods
        nav2_msgs::action::NavigateToPose::Goal GetGoalPose(const std::string &filename,
                                                            const std::string &destination);

        rcl_interfaces::msg::SetParametersResult DestinationUpdatedCallback(const std::vector<rclcpp::Parameter> &parameters);

    private:
        rclcpp::Node::SharedPtr node_;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

        std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> parameter_callback_handle_;

        // Parameter clients
        rclcpp::AsyncParametersClient::SharedPtr global_costmap_param_client_;
        rclcpp::AsyncParametersClient::SharedPtr local_costmap_param_client_;
        rclcpp::AsyncParametersClient::SharedPtr controller_server_param_client_;
        rclcpp::AsyncParametersClient::SharedPtr velocity_smoother_param_client_;
        rclcpp::AsyncParametersClient::SharedPtr activate_costmap_comparison_param_client_;
        rclcpp::AsyncParametersClient::SharedPtr table_detection_param_client_;

        // TF
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Parameters
        std::string destination_;
        std::string default_destination_;

        NavigateToPoseClient nav_to_pose;
        ComputePathToPoseClient path_to_pose;
        NavigateThroughPosesClient nav_to_poses;
    };

} 

#endif  
