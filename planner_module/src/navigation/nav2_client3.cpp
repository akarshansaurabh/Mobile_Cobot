#include "navigation/nav2_client3.hpp" // The header above (you would include the correct path)
#include <iostream>
#include <fstream>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using namespace std;

namespace custom_nav2_action_client2
{

    /** Nav2ActionClientBase Implementation **/

    Nav2ActionClientBase::Nav2ActionClientBase(rclcpp::Node::SharedPtr node)
        : node_(node) {}

    /** NavigateToPoseClient Implementation **/

    NavigateToPoseClient::NavigateToPoseClient(rclcpp::Node::SharedPtr node,
                                               std::shared_ptr<planner_correction::DetectionTracker> detection_tracker__)
        : Nav2ActionClientBase(node), goal_active_(false), detection_tracker_(detection_tracker__)
    {
        action_client_ = rclcpp_action::create_client<ActionType>(node_, "/navigate_to_pose");
        move_amr_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
            "/move_amr_topic", rclcpp::QoS(10),
            std::bind(&NavigateToPoseClient::MoveAMRCallBack, this, std::placeholders::_1));

        *detection_tracker_ = planner_correction::DetectionTracker::DETECT_NOTHING;
    }

    // amr approaches the table
    void NavigateToPoseClient::MoveAMRCallBack(const geometry_msgs::msg::Vector3::ConstSharedPtr &msg)
    {
        ActionType::Goal next_goal;
        next_goal.pose.header.frame_id = "map";
        next_goal.pose.header.stamp = node_->now();
        next_goal.pose.pose.position.x = nav_to_pose_actual_pose_.pose.position.x + msg->x;
        next_goal.pose.pose.position.y = nav_to_pose_actual_pose_.pose.position.y + msg->y;
        next_goal.pose.pose.position.z = nav_to_pose_actual_pose_.pose.position.z + msg->z;
        next_goal.pose.pose.orientation = nav_to_pose_desired_pose_.pose.pose.orientation;
        is_table_destination_ = false;
        SendGoal(next_goal);
        // also run a parallel thread for the arm motion planner
    }

    void NavigateToPoseClient::initialize()
    {
        amr_correction_ = std::make_shared<planner_correction::AMRCorrection>(node_, detection_tracker_);
    }

    void NavigateToPoseClient::SendGoal(const ActionType::Goal &goal_msg)
    {
        if (!action_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available.");
            return;
        }

        nav_to_pose_desired_pose_ = goal_msg;

        auto goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
        goal_options.goal_response_callback = std::bind(&NavigateToPoseClient::GoalResponseCallback, this, std::placeholders::_1);
        goal_options.feedback_callback = std::bind(&NavigateToPoseClient::FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options.result_callback = std::bind(&NavigateToPoseClient::ResultCallback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, goal_options);

        geometry_msgs::msg::PoseStamped pose = goal_msg.pose;

        tf2::Quaternion tf_quat;
        tf2::fromMsg(pose.pose.orientation, tf_quat);
        double roll, pitch;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, desired_yaw);
        amr_correction_->desired_yaw_ = desired_yaw;
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
        // std::cout << " feedback->distance_remaining " << feedback->distance_remaining << std::endl;

        if (feedback->distance_remaining < 0.3 && feedback->distance_remaining > 0.01)
        {
            // std::cout << " feedback->distance_remaining " << feedback->distance_remaining << std::endl;
            nav_to_pose_actual_pose_.pose.position.x = feedback->current_pose.pose.position.x;
            nav_to_pose_actual_pose_.pose.position.y = feedback->current_pose.pose.position.y;
            nav_to_pose_actual_pose_.pose.position.z = feedback->current_pose.pose.position.z;
            nav_to_pose_actual_pose_.pose.orientation.x = feedback->current_pose.pose.orientation.x;
            nav_to_pose_actual_pose_.pose.orientation.y = feedback->current_pose.pose.orientation.y;
            nav_to_pose_actual_pose_.pose.orientation.z = feedback->current_pose.pose.orientation.z;
            nav_to_pose_actual_pose_.pose.orientation.w = feedback->current_pose.pose.orientation.w;
            if (amr_correction_ && !amr_correction_->odometry_check_sub_)
            {
                std::cout << "subs activate" << std::endl;
                amr_correction_->odometry_check_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                    "/odom", rclcpp::QoS(10),
                    std::bind(&planner_correction::AMRCorrection::OdometryCheckCallback, amr_correction_.get(), std::placeholders::_1));
            }
        }
    }

    void NavigateToPoseClient::ResultCallback(const GoalHandle::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded.");
            amr_correction_->actual_pose_stamped.pose = nav_to_pose_actual_pose_.pose;
            amr_correction_->actual_pose_stamped.header.frame_id = "map";
            amr_correction_->actual_pose_stamped.header.stamp = node_->now();
            amr_correction_->desired_pose_stamped.pose.orientation = nav_to_pose_desired_pose_.pose.pose.orientation;
            if (is_table_destination_)
                *detection_tracker_ = planner_correction::DetectionTracker::DETECT_TABLE;
            else
            {
                *detection_tracker_ = planner_correction::DetectionTracker::DETECT_NOTHING;
                // if (*detection_tracker_ == planner_correction::DetectionTracker::DETECT_TABLE)
                //     *detection_tracker_ = planner_correction::DetectionTracker::DETECT_BOXES;
            }
            amr_correction_->CorrectOrientation(is_table_destination_);
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
        : Nav2ActionClientBase(node), goal_active_(false), current_goal_is_p1_(false), utilities_(nullptr)
    {
        action_client_ = rclcpp_action::create_client<ActionType>(node_, "/compute_path_to_pose");
    }

    void ComputePathToPoseClient::SendGoal(const ActionType::Goal &goal_msg, bool is_p1)
    {
        current_goal_is_p1_ = is_p1;

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
            RCLCPP_ERROR(node_->get_logger(), "ComputePathToPoseClient Goal was rejected by the server.");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "ComputePathToPoseClient Goal accepted by the server.");
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
                utilities_->OnPathComputed(result.result->path, current_goal_is_p1_); // 1st time true, 2nd time false
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
                                                                                p2_received_(false)
    {
        // Initialize publishers
        initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
        nav_to_pose.is_table_destination_ = false;

        // Set default destination
        default_destination_ = "home";
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

        parameter_callback_handle_ = node_->add_on_set_parameters_callback(
            std::bind(&Nav2Utilities::DestinationUpdatedCallback, this, std::placeholders::_1));

        // Set utilities pointer in path_to_pose so we can get callbacks
        path_to_pose.SetUtilities(this);

        std::string config_file = "/home/akarshan/mobile_cobot_ws/src/r1d1_description/config/locations.yaml";
        table_analyser = std::make_unique<waypointGen::TablePathAnalyzer>(config_file, 1.0);
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
        std::vector<std::string> doors_or_home = {"door_A", "door_B", "door_C", "door_D", "door_E", "home"};
        std::vector<std::string> tables = {"table_A", "table_B", "table_C", "table_D"};

        if (std::find(doors_or_home.begin(), doors_or_home.end(), destination_) != doors_or_home.end())
        {
            // destination is not table
            nav_to_pose.is_table_destination_ = false;
            nav_to_pose.SendGoal(destination_goal_);
        }
        else if (std::find(tables.begin(), tables.end(), destination_) != tables.end())
        {
            // 1) Get current pose
            nav_to_pose.is_table_destination_ = true;
            current_pose_ = GetCurrentPose();

            // 2) We want to send two goals P1 and P2 to compute_path_to_pose
            table_analyser->P1.header.stamp = node_->now();
            geometry_msgs::msg::PoseStamped p1_start = current_pose_;
            geometry_msgs::msg::PoseStamped p1_goal = table_analyser->P1;
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

    // after completion of 1st goal, send 2nd goal. after completion of 2nd goal, stop
    void Nav2Utilities::OnPathComputed(const nav_msgs::msg::Path &path, bool is_p1)
    {
        // This is called after a ComputePathToPose result is received
        if (!waiting_for_paths_)
            return;

        if (is_p1) // 1st time
        {
            p1_path_ = path;
            p1_received_ = true;
            RCLCPP_INFO(node_->get_logger(), "P1 path received. Length: %zu", p1_path_.poses.size());

            // Now send P2 goal
            table_analyser->P2.header.stamp = node_->now();
            geometry_msgs::msg::PoseStamped p2_start = current_pose_;
            geometry_msgs::msg::PoseStamped p2_goal = table_analyser->P2;

            nav2_msgs::action::ComputePathToPose::Goal p2_compute_goal;
            p2_compute_goal.goal = p2_goal;
            p2_compute_goal.start = p2_start;
            p2_compute_goal.use_start = true;
            path_to_pose.SendGoal(p2_compute_goal, false); // This is P2
        }
        else // 2nd time
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

        if (p1_length < p2_length)
        {
            RCLCPP_INFO(node_->get_logger(), "P1 is shorter. Sending P1 as navigate_to_pose goal.");
            // generate multiple waypoints till P1
            nav2_msgs::action::NavigateToPose::Goal goal_p1;
            goal_p1.pose = table_analyser->P1;
            nav_to_pose.SendGoal(goal_p1);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "P2 is shorter or equal. Sending P2 as navigate_to_pose goal.");
            // generate multiple waypoints till P2
            nav2_msgs::action::NavigateToPose::Goal goal_p2;
            goal_p2.pose = table_analyser->P2;
            nav_to_pose.SendGoal(goal_p2);
        }
    }
}
