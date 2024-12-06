#include "navigation/nav2_client3.hpp" // The header above (you would include the correct path)
#include <iostream>
#include <fstream>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace custom_nav2_action_client2
{

    /** Nav2ActionClientBase Implementation **/

    Nav2ActionClientBase::Nav2ActionClientBase(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
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
        (void)goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Distance remaining: %f", feedback->distance_remaining);
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
        : Nav2ActionClientBase(node), goal_active_(false), next_goal_is_p1_(false), utilities_(nullptr)
    {
        action_client_ = rclcpp_action::create_client<ActionType>(node_, "/compute_path_to_pose");
    }

    void ComputePathToPoseClient::SendGoal(const ActionType::Goal &goal_msg, bool is_p1)
    {
        next_goal_is_p1_ = is_p1;

        if (!action_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(node_->get_logger(), "ComputePathToPose action server not available.");
            return;
        }

        auto goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
        goal_options.goal_response_callback = std::bind(&ComputePathToPoseClient::GoalResponseCallback, this, std::placeholders::_1);
        goal_options.feedback_callback = std::bind(&ComputePathToPoseClient::FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options.result_callback = std::bind(&ComputePathToPoseClient::ResultCallback, this, std::placeholders::_1);

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
        (void)goal_handle;
        (void)feedback;
        // Implement feedback handling if needed
    }

    void ComputePathToPoseClient::ResultCallback(const GoalHandle::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Received path from planner.");
            if (utilities_)
            {
                // Call back to utilities with the computed path
                utilities_->OnPathComputed(result.result->path, next_goal_is_p1_);
            }
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
        : Nav2ActionClientBase(node)
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
        (void)goal_handle;
        (void)feedback;
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

    /** Nav2Utilities Implementation **/

    Nav2Utilities::Nav2Utilities(rclcpp::Node::SharedPtr node,
                                 NavigateToPoseClient &client_,
                                 ComputePathToPoseClient &path_client_,
                                 NavigateThroughPosesClient &waypoint_client) : node_(node),
                                                                                tf_buffer_(node_->get_clock()),
                                                                                tf_listener_(tf_buffer_),
                                                                                nav_to_pose(client_),
                                                                                path_to_pose(path_client_),
                                                                                nav_to_poses(waypoint_client),
                                                                                waiting_for_paths_(false),
                                                                                p1_received_(false),
                                                                                p2_received_(false),
                                                                                is_table_destination_(false)
    {
        // Initialize publishers
        initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

        // Set default destination
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
        initial_pose.pose.covariance[0] = 0.25;
        initial_pose.pose.covariance[7] = 0.25;
        initial_pose.pose.covariance[35] = 0.06853891945200942;
        this->SetInitialPose(initial_pose);

        // Initialize parameter clients
        global_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/global_costmap/global_costmap");
        local_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/local_costmap/local_costmap");
        controller_server_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/controller_server");
        velocity_smoother_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/velocity_smoother");
        activate_costmap_comparison_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/obstacle_monitor_node");
        table_detection_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/pointcloud_processor_node");

        parameter_callback_handle_ = node_->add_on_set_parameters_callback(
            std::bind(&Nav2Utilities::DestinationUpdatedCallback, this, std::placeholders::_1));

        // Set utilities pointer in path_to_pose so we can get callbacks
        path_to_pose.SetUtilities(this);
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
        if (!global_costmap_param_client_->wait_for_service(10s) ||
            !local_costmap_param_client_->wait_for_service(10s) ||
            !controller_server_param_client_->wait_for_service(10s) ||
            !velocity_smoother_param_client_->wait_for_service(10s))
        {
            RCLCPP_ERROR(node_->get_logger(), "One or more parameter services are not available.");
            return;
        }

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
                result.reason = "Invalid parameter name";
                RCLCPP_ERROR(node_->get_logger(), "Invalid parameter name");
            }
        }

        std::string yaml_file = "/home/akarshan/mobile_cobot_ws/src/r1d1_description/config/locations.yaml";
        destination_goal_ = GetGoalPose(yaml_file, destination_);
        destination_goal_.pose.header.frame_id = "map";
        destination_goal_.pose.header.stamp = node_->now();

        // Determine if destination is door/home or table
        // Doors and home: {door_A, door_B, door_C, door_D, door_E, home}
        // Tables: {table_A, table_B, table_C, table_D}
        std::vector<std::string> doors_or_home = {"door_A", "door_B", "door_C", "door_D", "door_E", "home"};
        std::vector<std::string> tables = {"table_A", "table_B", "table_C", "table_D"};

        if (std::find(doors_or_home.begin(), doors_or_home.end(), destination_) != doors_or_home.end())
        {
            // Just send the goal to nav_to_pose
            is_table_destination_ = false;
            nav_to_pose.SendGoal(destination_goal_);
        }
        else if (std::find(tables.begin(), tables.end(), destination_) != tables.end())
        {
            // Table scenario:
            // 1) Get current pose
            is_table_destination_ = true;
            current_pose_ = GetCurrentPose();

            // 2) We want to send two goals P1 and P2 to compute_path_to_pose
            // For demonstration, let's assume P1 is just a direct path to destination, and
            // P2 is also direct but we imagine a slight orientation difference for testing.
            //
            // In a real scenario, you'd have logic to produce different goals. Here we produce two distinct goals:
            // P1: current_pose -> destination_goal_.pose
            // P2: current_pose -> destination_goal_.pose (maybe rotate orientation by some factor)
            // We'll just use the same position but different orientation for P2 for demonstration.

            geometry_msgs::msg::PoseStamped p1_start = current_pose_;
            geometry_msgs::msg::PoseStamped p1_goal = destination_goal_.pose;

            geometry_msgs::msg::PoseStamped p2_start = current_pose_;
            geometry_msgs::msg::PoseStamped p2_goal = destination_goal_.pose;
            // Slight modification in orientation for P2 to differentiate:
            p2_goal.pose.orientation.z += 0.01; // Just a tiny tweak for demonstration
            if (p2_goal.pose.orientation.z > 1.0)
                p2_goal.pose.orientation.z = 1.0;

            // We'll store them so that after we get both paths we can decide.
            p1_received_ = false;
            p2_received_ = false;
            waiting_for_paths_ = true;

            // Send P1 as compute_path_to_pose goal
            nav2_msgs::action::ComputePathToPose::Goal p1_compute_goal;
            p1_compute_goal.goal = p1_goal;
            p1_compute_goal.start = p1_start;
            p1_compute_goal.use_start = true;
            path_to_pose.SendGoal(p1_compute_goal, true); // This is P1
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Destination '%s' is neither door/home nor table.", destination_.c_str());
            result.successful = false;
            result.reason = "Unknown destination";
        }

        return result;
    }

    void Nav2Utilities::OnPathComputed(const nav_msgs::msg::Path &path, bool is_p1)
    {
        // This is called after a ComputePathToPose result is received
        if (!waiting_for_paths_)
        {
            // Not expecting paths, ignore
            return;
        }

        if (is_p1)
        {
            p1_path_ = path;
            p1_received_ = true;
            RCLCPP_INFO(node_->get_logger(), "P1 path received. Length: %zu", p1_path_.poses.size());

            // Now send P2 goal
            geometry_msgs::msg::PoseStamped p2_start = current_pose_;
            geometry_msgs::msg::PoseStamped p2_goal = destination_goal_.pose;
            p2_goal.pose.orientation.z += 0.01; // same tweak as above
            if (p2_goal.pose.orientation.z > 1.0)
                p2_goal.pose.orientation.z = 1.0;

            nav2_msgs::action::ComputePathToPose::Goal p2_compute_goal;
            p2_compute_goal.goal = p2_goal;
            p2_compute_goal.start = p2_start;
            p2_compute_goal.use_start = true;
            path_to_pose.SendGoal(p2_compute_goal, false); // This is P2
        }
        else
        {
            p2_path_ = path;
            p2_received_ = true;
            RCLCPP_INFO(node_->get_logger(), "P2 path received. Length: %zu", p2_path_.poses.size());

            // Now we have both P1 and P2
            DecideAndSendFinalGoal();
        }
    }

    void Nav2Utilities::DecideAndSendFinalGoal()
    {
        waiting_for_paths_ = false;

        // Compare lengths of p1_path_ and p2_path_
        double p1_length = 0.0;
        for (size_t i = 0; i + 1 < p1_path_.poses.size(); i++)
        {
            double dx = p1_path_.poses[i + 1].pose.position.x - p1_path_.poses[i].pose.position.x;
            double dy = p1_path_.poses[i + 1].pose.position.y - p1_path_.poses[i].pose.position.y;
            double dz = p1_path_.poses[i + 1].pose.position.z - p1_path_.poses[i].pose.position.z;
            p1_length += std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        double p2_length = 0.0;
        for (size_t i = 0; i + 1 < p2_path_.poses.size(); i++)
        {
            double dx = p2_path_.poses[i + 1].pose.position.x - p2_path_.poses[i].pose.position.x;
            double dy = p2_path_.poses[i + 1].pose.position.y - p2_path_.poses[i].pose.position.y;
            double dz = p2_path_.poses[i + 1].pose.position.z - p2_path_.poses[i].pose.position.z;
            p2_length += std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        RCLCPP_INFO(node_->get_logger(), "P1 length: %f, P2 length: %f", p1_length, p2_length);

        // If P1 is shorter, send P1 as final goal, else P2
        // P1 and P2 differ only slightly. We must reconstruct the final navigate_to_pose goal.
        // We'll just use the same final destination_goal_ because P1 and P2 differ only in approach.
        // In a real scenario, you would store P1/P2 final poses.

        if (p1_length < p2_length)
        {
            RCLCPP_INFO(node_->get_logger(), "P1 is shorter. Sending P1 as navigate_to_pose goal.");
            nav_to_pose.SendGoal(destination_goal_);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "P2 is shorter or equal. Sending P2 as navigate_to_pose goal.");
            // In a real scenario, we might have stored a slightly different final pose for P2.
            // Here we just reuse destination_goal_ for demonstration.
            nav_to_pose.SendGoal(destination_goal_);
        }
    }

} // namespace custom_nav2_action_client
