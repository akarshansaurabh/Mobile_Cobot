#ifndef NAV2_ACTION_CLIENT_HPP_
#define NAV2_ACTION_CLIENT_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <unordered_map>
#include <tuple>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "std_msgs/msg/string.hpp"

#include "nav_msgs/msg/odometry.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using ComputePathToPoseGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

using FollowPath = nav2_msgs::action::FollowPath;
using FollowPathGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

namespace custom_Nav2ActionClient
{
  class Nav2ActionClient : public rclcpp::Node
  {
  public:
    Nav2ActionClient();
    void SetCostmapParameters();
    void SetNav2Parameters();
    void SetInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &initial_pose);

    void SendGoal(const NavigateToPose::Goal &goal_msg);
    void CancelGoal();

    void SendPlannerGoal(const geometry_msgs::msg::PoseStamped &goal_pose_stamped);
    geometry_msgs::msg::PoseStamped GetCurrentPose();

    NavigateToPose::Goal GetGoalPose(const std::string &filename, const std::string &destination);

  private:
    void GoalResponseCallBack(const NavigateToPoseGoalHandle::SharedPtr &goal_handle);
    void FeedbackCallBack(const NavigateToPoseGoalHandle::SharedPtr &goal_handle,
                          const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void ResultCallBack(const NavigateToPoseGoalHandle::WrappedResult &result);

    void PlannerGoalResponseCallback(const ComputePathToPoseGoalHandle::SharedPtr &goal_handle);
    void PlannerResultCallBack(const ComputePathToPoseGoalHandle::WrappedResult &result);
    rcl_interfaces::msg::SetParametersResult DestinationUpdatedCallback(const std::vector<rclcpp::Parameter> &parameters);

    // void OdometryCallBack(const nav_msgs::msg::Odometry &msg);

    rclcpp_action::Client<NavigateToPose>::SharedPtr single_goal_action_client_;
    rclcpp_action::Client<ComputePathToPose>::SharedPtr global_planner_action_client_;
    rclcpp_action::Client<FollowPath>::SharedPtr controller_action_client_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> parameter_callback_handle_;

    bool goal_active_;
    NavigateToPoseGoalHandle::SharedPtr goal_handle_;
    ComputePathToPoseGoalHandle::SharedPtr path_goal_handle_;
    // Parameter clients 
    rclcpp::AsyncParametersClient::SharedPtr global_costmap_param_client_;
    rclcpp::AsyncParametersClient::SharedPtr local_costmap_param_client_;
    rclcpp::AsyncParametersClient::SharedPtr controller_server_param_client_;
    rclcpp::AsyncParametersClient::SharedPtr velocity_smoother_param_client_;

    // ros2 params
    std::string destination_, default_destination_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
  };
}

#endif