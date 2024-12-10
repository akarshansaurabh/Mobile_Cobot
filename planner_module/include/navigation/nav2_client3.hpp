#ifndef NAV2_ACTION_CLIENT_HPP_
#define NAV2_ACTION_CLIENT_HPP_

#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <cmath>
#include <limits>
#include <algorithm>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <yaml-cpp/yaml.h>

#include "planner_module/waypoint_gen.hpp"
#include "planner_module/planner_correction.hpp"
// #include "pc_processing/octomap_generator.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace custom_nav2_action_client2
{

    // Forward declaration of Nav2Utilities, so we can reference it in clients
    class Nav2Utilities;

    // Base class for shared functionality
    class Nav2ActionClientBase
    {
    public:
        explicit Nav2ActionClientBase(rclcpp::Node::SharedPtr node);
        virtual ~Nav2ActionClientBase() = default;

    protected:
        rclcpp::Node::SharedPtr node_;
    };

    // Class for handling NavigateToPose action
    class NavigateToPoseClient : public Nav2ActionClientBase
    {
    public:
        using ActionType = nav2_msgs::action::NavigateToPose;
        using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

        explicit NavigateToPoseClient(rclcpp::Node::SharedPtr node, std::shared_ptr<planner_correction::DetectionTracker> detection_tracker__);
        void initialize();

        void SendGoal(const ActionType::Goal &goal_msg);
        void CancelGoal();
        void GoalResponseCallback(const GoalHandle::SharedPtr &goal_handle);
        void FeedbackCallback(const GoalHandle::SharedPtr &goal_handle,
                              const std::shared_ptr<const ActionType::Feedback> feedback);
        void ResultCallback(const GoalHandle::WrappedResult &result);
        bool is_table_destination_; // to indicate if current destination is a table or a door/home

    private:
        rclcpp_action::Client<ActionType>::SharedPtr action_client_;
        std::shared_ptr<planner_correction::DetectionTracker> detection_tracker_;
        GoalHandle::SharedPtr goal_handle_;
        bool goal_active_;
        ActionType::Goal nav_to_pose_desired_pose_;
        double desired_yaw;
        std::shared_ptr<planner_correction::AMRCorrection> amr_correction_;
        geometry_msgs::msg::PoseStamped nav_to_pose_actual_pose_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr move_amr_sub_;
        void MoveAMRCallBack(const geometry_msgs::msg::Vector3::ConstSharedPtr &msg);
    };

    // Class for handling ComputePathToPose action
    // We add logic to differentiate between P1 and P2, and to call back into Nav2Utilities.
    class ComputePathToPoseClient : public Nav2ActionClientBase
    {
    public:
        using ActionType = nav2_msgs::action::ComputePathToPose;
        using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;

        explicit ComputePathToPoseClient(rclcpp::Node::SharedPtr node);

        // Action methods with action-specific parameters
        // We add a boolean to indicate if this is P1 or P2.
        void SendGoal(const ActionType::Goal &goal_msg, bool is_p1);
        void CancelGoal();
        void GoalResponseCallback(const GoalHandle::SharedPtr &goal_handle);
        void FeedbackCallback(const GoalHandle::SharedPtr &goal_handle,
                              const std::shared_ptr<const ActionType::Feedback> feedback);
        void ResultCallback(const GoalHandle::WrappedResult &result);

        // Set utilities pointer so we can inform Nav2Utilities of path results
        void SetUtilities(Nav2Utilities *utilities) { utilities_ = utilities; }

    private:
        rclcpp_action::Client<ActionType>::SharedPtr action_client_;
        GoalHandle::SharedPtr goal_handle_;
        bool goal_active_;
        bool current_goal_is_p1_;
        Nav2Utilities *utilities_{nullptr};
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
    // We will add logic for handling the two-path computation scenario for tables.
    class Nav2Utilities
    {
    public:
        explicit Nav2Utilities(rclcpp::Node::SharedPtr node,
                               NavigateToPoseClient &client_,
                               ComputePathToPoseClient &path_client_,
                               NavigateThroughPosesClient &waypoint_client);

        void ActivateCostmapParameters(bool activate);
        void SetNav2Parameters();
        void ActivatePCProcessingParameters(const std::string &param);

        void SetInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &initial_pose);
        geometry_msgs::msg::PoseStamped GetCurrentPose();

        nav2_msgs::action::NavigateToPose::Goal GetGoalPose(const std::string &filename,
                                                            const std::string &destination);

        rcl_interfaces::msg::SetParametersResult DestinationUpdatedCallback(const std::vector<rclcpp::Parameter> &parameters);

        // Called by ComputePathToPoseClient after receiving a path result
        // This allows us to handle P1/P2 logic after results are received.
        void OnPathComputed(const nav_msgs::msg::Path &path, bool is_p1);

        std::unique_ptr<waypointGen::TablePathAnalyzer> table_analyser;

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

        std::string destination_;
        std::string default_destination_;

        NavigateToPoseClient &nav_to_pose;
        ComputePathToPoseClient &path_to_pose;
        NavigateThroughPosesClient &nav_to_poses;

        // For the table scenario:
        // We'll store states to handle P1 and P2 logic.
        bool waiting_for_paths_; // Indicates we are in the table scenario waiting for P1/P2 paths
        bool p1_received_;
        bool p2_received_;
        nav_msgs::msg::Path p1_path_;
        nav_msgs::msg::Path p2_path_;
        geometry_msgs::msg::PoseStamped current_pose_;
        nav2_msgs::action::NavigateToPose::Goal destination_goal_; // final goal from YAML

        // After receiving both paths, decide which is shorter and send final nav_to_pose
        void DecideAndSendFinalGoal();
        // pc processing activation
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pc_processing_sub_;
        void pointCloudActivationCallback(const std_msgs::msg::String::ConstSharedPtr &msg);
    };

}

#endif
