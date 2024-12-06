#include "navigation/nav2_client2.hpp"

#include <iostream>
#include <fstream>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace custom_nav2_action_client
{

    /** Nav2ActionClientBase Implementation **/

    Nav2ActionClientBase::Nav2ActionClientBase(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        // Common initialization if needed
    }

    /** Nav2Utilities Implementation **/

    Nav2Utilities::Nav2Utilities(rclcpp::Node::SharedPtr node,
                                 NavigateToPoseClient &client_,
                                 ComputePathToPoseClient &path_client_,
                                 NavigateThroughPosesClient &waypoint_client) : node_(node),
                                                                                tf_buffer_(node_->get_clock()),
                                                                                tf_listener_(tf_buffer_),
                                                                                nav_to_pose(client_),
                                                                                path_to_pose(path_client_),
                                                                                nav_to_poses(waypoint_client)
    {
        // Initialize publishers
        initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

        // Declare parameters
        default_destination_ = "door_B";
        node_->declare_parameter<std::string>("destination", default_destination_);

        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = node_->now();
        initial_pose.header.frame_id = "map";
        initial_pose.pose.pose.position.x = 0.0;
        initial_pose.pose.pose.position.y = 0.0;
        initial_pose.pose.pose.position.z = 0.0;
        initial_pose.pose.pose.orientation.x = 0.0;
        initial_pose.pose.pose.orientation.y = 0.0;
        initial_pose.pose.pose.orientation.z = 0.0;
        initial_pose.pose.pose.orientation.w = 1.0;
        for (int i = 0; i < 36; ++i)
            initial_pose.pose.covariance[i] = 0.0;
        initial_pose.pose.covariance[0] = 0.25;                 // X position covariance
        initial_pose.pose.covariance[7] = 0.25;                 // Y position covariance
        initial_pose.pose.covariance[35] = 0.06853891945200942; // Yaw angle covariance
        this->SetInitialPose(initial_pose);

        // Initialize parameter clients
        global_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/global_costmap/global_costmap");
        local_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/local_costmap/local_costmap");
        controller_server_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/controller_server");
        velocity_smoother_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/velocity_smoother");
        activate_costmap_comparison_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/obstacle_monitor_node");
        table_detection_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/pointcloud_processor_node");

        // Add parameter callback
        parameter_callback_handle_ = node_->add_on_set_parameters_callback(
            std::bind(&Nav2Utilities::DestinationUpdatedCallback, this, std::placeholders::_1));
    }

    void Nav2Utilities::ActivateCostmapParameters(bool activate)
    {
        RCLCPP_INFO(node_->get_logger(), "ActivateCostmapParameters called with activate: %s", activate ? "true" : "false");

        if (!activate_costmap_comparison_param_client_->wait_for_service(10s))
        {
            RCLCPP_ERROR(node_->get_logger(), "Costmap parameter service not available.");
            return;
        }

        std::vector<rclcpp::Parameter> params = {rclcpp::Parameter("activate_costmap_comparison", activate)};
        auto future = activate_costmap_comparison_param_client_->set_parameters(params);

        std::thread([this, future = std::move(future), params]() mutable
                    {
        try
        {
            auto results = future.get();
            for (size_t i = 0; i < results.size(); ++i)
            {
                if (!results[i].successful)
                {
                RCLCPP_ERROR(node_->get_logger(), "Failed to set parameter '%s': %s",
                            params[i].get_name().c_str(), results[i].reason.c_str());
                }
                else
                {
                RCLCPP_INFO(node_->get_logger(), "Successfully set parameter '%s'",
                            params[i].get_name().c_str());
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Exception while setting parameters: %s", e.what());
        } })
            .detach();
    }

    void Nav2Utilities::SetNav2Parameters()
    {
        // Wait for parameter services
        if (!global_costmap_param_client_->wait_for_service(10s) ||
            !local_costmap_param_client_->wait_for_service(10s) ||
            !controller_server_param_client_->wait_for_service(10s) ||
            !velocity_smoother_param_client_->wait_for_service(10s))
        {
            RCLCPP_ERROR(node_->get_logger(), "One or more parameter services are not available.");
            return;
        }

        // Set parameters
        std::vector<rclcpp::Parameter> global_params = {
            rclcpp::Parameter("robot_radius", 0.3),
            rclcpp::Parameter("inflation_layer.inflation_radius", 0.6)};

        std::vector<rclcpp::Parameter> local_params = {
            rclcpp::Parameter("robot_radius", 0.3),
            rclcpp::Parameter("inflation_layer.inflation_radius", 0.6)};

        auto global_future = global_costmap_param_client_->set_parameters(global_params);
        auto local_future = local_costmap_param_client_->set_parameters(local_params);

        try
        {
            auto global_results = global_future.get();
            for (size_t i = 0; i < global_results.size(); ++i)
            {
                if (!global_results[i].successful)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to set global parameter '%s': %s",
                                 global_params[i].get_name().c_str(), global_results[i].reason.c_str());
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "Successfully set global parameter '%s'",
                                global_params[i].get_name().c_str());
                }
            }

            auto local_results = local_future.get();
            for (size_t i = 0; i < local_results.size(); ++i)
            {
                if (!local_results[i].successful)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to set local parameter '%s': %s",
                                 local_params[i].get_name().c_str(), local_results[i].reason.c_str());
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "Successfully set local parameter '%s'",
                                local_params[i].get_name().c_str());
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Exception while setting parameters: %s", e.what());
        }
    }

    void Nav2Utilities::ActivatePCProcessingParameters(const std::string &param)
    {
        if (!table_detection_param_client_->wait_for_service(10s))
        {
            RCLCPP_ERROR(node_->get_logger(), "PC Processing parameter service not available.");
            return;
        }

        std::vector<rclcpp::Parameter> params = {rclcpp::Parameter("pc_processing_param", param)};
        auto future = table_detection_param_client_->set_parameters(params);

        std::thread([this, future = std::move(future), params]() mutable
                    {
    try
    {
      auto results = future.get();
      for (size_t i = 0; i < results.size(); ++i)
      {
        if (!results[i].successful)
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to set parameter '%s': %s",
                       params[i].get_name().c_str(), results[i].reason.c_str());
        }
        else
        {
          RCLCPP_INFO(node_->get_logger(), "Successfully set parameter '%s'",
                      params[i].get_name().c_str());
        }
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Exception while setting parameters: %s", e.what());
    } })
            .detach();
    }

    void Nav2Utilities::SetInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &initial_pose)
    {
        RCLCPP_INFO(node_->get_logger(), "Setting initial pose");

        for (int i = 0; i < 100; ++i)
        {
            initial_pose_pub_->publish(initial_pose);
            rclcpp::sleep_for(10ms);
        }
    }

    geometry_msgs::msg::PoseStamped Nav2Utilities::GetCurrentPose()
    {
        geometry_msgs::msg::PoseStamped current_pose;

        try
        {
            auto transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));
            current_pose.header = transform_stamped.header;
            current_pose.pose.position.x = transform_stamped.transform.translation.x;
            current_pose.pose.position.y = transform_stamped.transform.translation.y;
            current_pose.pose.position.z = transform_stamped.transform.translation.z;
            current_pose.pose.orientation = transform_stamped.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not get current pose: %s", ex.what());
        }

        return current_pose;
    }

    nav2_msgs::action::NavigateToPose::Goal Nav2Utilities::GetGoalPose(const std::string &filename,
                                                                       const std::string &destination)
    {
        nav2_msgs::action::NavigateToPose::Goal goal;

        try
        {
            YAML::Node config = YAML::LoadFile(filename);
            if (!config["locations"])
            {
                RCLCPP_ERROR(node_->get_logger(), "'locations' key not found in the YAML file.");
                return goal;
            }

            for (const auto &item : config["locations"])
            {
                std::string key = item.first.as<std::string>();
                if (key == destination)
                {
                    auto position = item.second["position"];
                    auto orientation = item.second["orientation"];

                    goal.pose.pose.position.x = position["x"].as<double>();
                    goal.pose.pose.position.y = position["y"].as<double>();
                    goal.pose.pose.position.z = position["z"].as<double>();

                    goal.pose.pose.orientation.x = orientation["x"].as<double>();
                    goal.pose.pose.orientation.y = orientation["y"].as<double>();
                    goal.pose.pose.orientation.z = orientation["z"].as<double>();
                    goal.pose.pose.orientation.w = orientation["w"].as<double>();

                    goal.pose.header.frame_id = "map";
                    RCLCPP_INFO(node_->get_logger(), "Loaded goal for destination '%s'", destination.c_str());
                    return goal;
                }
            }

            RCLCPP_ERROR(node_->get_logger(), "Destination '%s' not found in the YAML file.", destination.c_str());
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "YAML Exception: %s", e.what());
        }

        return goal;
    }

    rcl_interfaces::msg::SetParametersResult Nav2Utilities::DestinationUpdatedCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Parameters set successfully.";
        std::cout << "nav2 param callback called" << std::endl;
        for (const auto &param : parameters)
        {
            if (param.get_name() == "destination")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
                {
                    destination_ = param.as_string();
                    RCLCPP_INFO(node_->get_logger(), "Destination parameter updated to '%s'", destination_.c_str());
                }
                else
                {
                    result.successful = false;
                    result.reason = "Invalid parameter type for 'destination'";
                    RCLCPP_ERROR(node_->get_logger(), "Invalid parameter type for 'destination'");
                }
                break;
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid parameter nams'";
                RCLCPP_ERROR(node_->get_logger(), "Invalid parameter name");
            }
        }

        std::string yaml_file = "/home/akarshan/mobile_cobot_ws/src/r1d1_description/config/locations.yaml";
        auto destination_goal = GetGoalPose(yaml_file, destination_);
        destination_goal.pose.header.frame_id = "map";
        destination_goal.pose.header.stamp = node_->now();
        // if doors, use nav_to_pose
        // if table, use path_to_pose
        nav_to_pose.SendGoal(destination_goal);
        return result;
    }

    /** NavigateToPoseClient Implementation **/

    NavigateToPoseClient::NavigateToPoseClient(rclcpp::Node::SharedPtr node)
        : Nav2ActionClientBase(node), goal_active_(false)
    {
        action_client_ = rclcpp_action::create_client<ActionType>(node_, "/navigate_to_pose");
    }

    void NavigateToPoseClient::SendGoal(const ActionType::Goal &goal_msg)
    {
        if (!action_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available.");
            return;
        }

        auto goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
        goal_options.goal_response_callback = std::bind(&NavigateToPoseClient::GoalResponseCallback, this, std::placeholders::_1);
        goal_options.feedback_callback = std::bind(&NavigateToPoseClient::FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options.result_callback = std::bind(&NavigateToPoseClient::ResultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, goal_options);
    }

    void NavigateToPoseClient::CancelGoal()
    {
        if (goal_handle_)
        {
            action_client_->async_cancel_goal(goal_handle_);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "No active goal to cancel.");
        }
    }

    void NavigateToPoseClient::GoalResponseCallback(const GoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server.");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by the server.");
            this->goal_handle_ = goal_handle;
        }
    }

    void NavigateToPoseClient::FeedbackCallback(const GoalHandle::SharedPtr &goal_handle,
                                                const std::shared_ptr<const ActionType::Feedback> feedback)
    {
        RCLCPP_INFO(node_->get_logger(), "Distance remaining: %f", feedback->distance_remaining);

        // Implement specific feedback logic here, possibly interacting with Nav2Utilities
    }

    void NavigateToPoseClient::ResultCallback(const GoalHandle::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded.");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "Goal was canceled.");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
            break;
        }
    }

    /** ComputePathToPoseClient Implementation **/

    ComputePathToPoseClient::ComputePathToPoseClient(rclcpp::Node::SharedPtr node)
        : Nav2ActionClientBase(node), goal_active_(false)
    {
        action_client_ = rclcpp_action::create_client<ActionType>(node_, "/compute_path_to_pose");
    }

    void ComputePathToPoseClient::SendGoal(const ActionType::Goal &goal_msg)
    {
        if (!action_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(node_->get_logger(), "ComputePathToPose action server not available.");
            return;
        }

        auto goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
        goal_options.goal_response_callback = std::bind(&ComputePathToPoseClient::GoalResponseCallback, this, std::placeholders::_1);
        goal_options.result_callback = std::bind(&ComputePathToPoseClient::ResultCallback, this, std::placeholders::_1);
        goal_options.feedback_callback = std::bind(&ComputePathToPoseClient::FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

        action_client_->async_send_goal(goal_msg, goal_options);
    }

    void ComputePathToPoseClient::CancelGoal()
    {
        if (goal_handle_)
        {
            action_client_->async_cancel_goal(goal_handle_);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "No active goal to cancel.");
        }
    }

    void ComputePathToPoseClient::GoalResponseCallback(const GoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server.");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by the server.");
            goal_handle_ = goal_handle;
        }
    }

    void ComputePathToPoseClient::FeedbackCallback(const GoalHandle::SharedPtr &goal_handle,
                                                   const std::shared_ptr<const ActionType::Feedback> feedback)
    {
        // Implement feedback handling if needed
    }

    void ComputePathToPoseClient::ResultCallback(const GoalHandle::WrappedResult &result)
    {

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Received path from planner.");
            // Process the path if needed
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "Goal was canceled.");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
            break;
        }
    }

    /** NavigateThroughPosesClient Implementation **/

    NavigateThroughPosesClient::NavigateThroughPosesClient(rclcpp::Node::SharedPtr node)
        : Nav2ActionClientBase(node), goal_active_(false)
    {
        action_client_ = rclcpp_action::create_client<ActionType>(node_, "/navigate_through_poses");
    }

    void NavigateThroughPosesClient::SendGoal(const ActionType::Goal &goal_msg)
    {
        if (!action_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(node_->get_logger(), "NavigateThroughPoses action server not available.");
            return;
        }

        auto goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
        goal_options.goal_response_callback = std::bind(&NavigateThroughPosesClient::GoalResponseCallback, this, std::placeholders::_1);
        goal_options.feedback_callback = std::bind(&NavigateThroughPosesClient::FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options.result_callback = std::bind(&NavigateThroughPosesClient::ResultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, goal_options);
    }

    void NavigateThroughPosesClient::CancelGoal()
    {
        if (goal_handle_)
        {
            action_client_->async_cancel_goal(goal_handle_);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "No active goal to cancel.");
        }
    }

    void NavigateThroughPosesClient::GoalResponseCallback(const GoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server.");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by the server.");
            goal_handle_ = goal_handle;
        }
    }

    void NavigateThroughPosesClient::FeedbackCallback(const GoalHandle::SharedPtr &goal_handle,
                                                      const std::shared_ptr<const ActionType::Feedback> feedback)
    {
        // Implement feedback handling if needed
    }

    void NavigateThroughPosesClient::ResultCallback(const GoalHandle::WrappedResult &result)
    {

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Successfully navigated through poses.");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "Goal was canceled.");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
            break;
        }
    }

} // namespace custom_nav2_action_client
