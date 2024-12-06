#include "navigation/nav2_client.hpp"

using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace custom_Nav2ActionClient
{
  Nav2ActionClient::Nav2ActionClient() : Node("nav2_client_node"),
                                         tf_buffer_(this->get_clock()),
                                         tf_listener_(tf_buffer_),
                                         default_destination_("home"),
                                         cost_map_is_active(false)
  {
    // service name needs to be searched
    single_goal_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
    global_planner_action_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "/compute_path_to_pose");

    // this->create_subscription<nav_msgs::msg::Odometry>(
    //     "odom", 100, std::bind(&Nav2ActionClient::OdometryCallBack, this, _1));
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&Nav2ActionClient::DestinationUpdatedCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("destination", default_destination_);

    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.stamp = this->now();
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

    global_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_costmap/global_costmap");
    local_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/local_costmap/local_costmap");
    controller_server_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/controller_server");
    velocity_smoother_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/velocity_smoother");
    activate_costmap_comparison_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/obstacle_monitor_node");
    table_detection_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/pointcloud_transformer_node");
  }

  void Nav2ActionClient::ActivatePCProcessing_Parameters(string str)
  {
    cout << "PCProcessing function called" << endl;
    if (!table_detection_param_client_->wait_for_service(10s))
    {
      RCLCPP_ERROR(this->get_logger(), "PCProcessing parameter service not available.");
      return;
    }

    std::vector<rclcpp::Parameter> pc_processing_params;
    pc_processing_params.emplace_back(rclcpp::Parameter("pc_processing_param", str));
    auto pc_processing_future = table_detection_param_client_->set_parameters(pc_processing_params);

    std::thread([this, pc_processing_future = std::move(pc_processing_future), pc_processing_params]() mutable
                {
      try
      {
        std::cout << "Attempting to set parameter 'pc_processing_param'..." << std::endl;
        auto pc_processing_results = pc_processing_future.get();

        for (size_t i = 0; i < pc_processing_results.size(); ++i)
        {
          const auto &result = pc_processing_results[i];
          const auto &param = pc_processing_params[i];
          if (!result.successful)
          {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set parameter '%s' on pc_processing_param: %s",
                         param.get_name().c_str(), result.reason.c_str());
          }
          else
          {
            RCLCPP_INFO(this->get_logger(),
                        "Successfully set parameter '%s' on pc_processing_param",
                        param.get_name().c_str());
          }
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Exception while setting parameters: %s", e.what());
      }
      std::cout << "pc_processing_param operation completed." << std::endl; })
        .detach(); // Detach the thread to allow it to run independently
    cout << "pc_processing_param function done" << endl;
  }

  void Nav2ActionClient::ActivateCostmapParameters(bool activate)
  {
    cout << "ActivateCostmapParameters function called" << endl;
    if (!activate_costmap_comparison_param_client_->wait_for_service(10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Costmap Update parameter service not available.");
      return;
    }

    std::vector<rclcpp::Parameter> costmap_params;
    costmap_params.emplace_back(rclcpp::Parameter("activate_costmap_comparison", activate));
    auto costmap_future = activate_costmap_comparison_param_client_->set_parameters(costmap_params);
    std::thread([this, costmap_future = std::move(costmap_future), costmap_params]() mutable
                {
      try
      {
        std::cout << "Attempting to set parameter 'activate_costmap_comparison'..." << std::endl;
        auto costmap_results = costmap_future.get();

        for (size_t i = 0; i < costmap_results.size(); ++i)
        {
          const auto &result = costmap_results[i];
          const auto &param = costmap_params[i];
          if (!result.successful)
          {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set parameter '%s' on activate_costmap_comparison: %s",
                         param.get_name().c_str(), result.reason.c_str());
          }
          else
          {
            RCLCPP_INFO(this->get_logger(),
                        "Successfully set parameter '%s' on activate_costmap_comparison",
                        param.get_name().c_str());
          }
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Exception while setting parameters: %s", e.what());
      }
      std::cout << "ActivateCostmapParameters operation completed." << std::endl; })
        .detach(); // Detach the thread to allow it to run independently
    cout << "ActivateCostmapParameters function done" << endl;
  }

  void Nav2ActionClient::SetNav2Parameters()
  {
    // Wait for the parameter services to be available
    if (!global_costmap_param_client_->wait_for_service(10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Global costmap parameter service not available.");
      return;
    }

    if (!local_costmap_param_client_->wait_for_service(10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Local costmap parameter service not available.");
      return;
    }

    if (!controller_server_param_client_->wait_for_service(10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Controller server parameter service not available.");
      return;
    }

    if (!velocity_smoother_param_client_->wait_for_service(10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Velocity smoother parameter service not available.");
      return;
    }

    // create parameters for both the global and local costmaps and push them in corresponding vectors
    std::vector<rclcpp::Parameter> global_params, local_params;
    std::vector<rclcpp::Parameter> controller_params, velocity_smoother_params;
    global_params.emplace_back(rclcpp::Parameter("robot_radius", 0.3));
    global_params.emplace_back(rclcpp::Parameter("inflation_layer.inflation_radius", 0.6));
    local_params.emplace_back(rclcpp::Parameter("robot_radius", 0.3));
    local_params.emplace_back(rclcpp::Parameter("inflation_layer.inflation_radius", 0.6));
    // New controller_server parameters
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.min_vel_x", 0.0));
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.min_vel_y", 0.0));
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.max_vel_x", 2.0));
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.max_vel_y", 0.0));
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.max_vel_theta", 1.57));

    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.min_speed_xy", 0.0));
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.max_speed_xy", 2.0));

    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.acc_lim_x", 10.0));
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.acc_lim_y", 0.0));
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.acc_lim_theta", 10.0));
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.decel_lim_x", -10.0));
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.decel_lim_y", 0.0));
    // controller_params.emplace_back(rclcpp::Parameter("FollowPath.decel_lim_theta", -10.0));

    // // New velocity_smoother parameters
    // velocity_smoother_params.emplace_back(rclcpp::Parameter("max_velocity", std::vector<double>{2.0, 0.0, 1.57}));
    // velocity_smoother_params.emplace_back(rclcpp::Parameter("min_velocity", std::vector<double>{-2.0, 0.0, -1.57}));
    // velocity_smoother_params.emplace_back(rclcpp::Parameter("max_accel", std::vector<double>{10.0, 0.0, 10.0}));
    // velocity_smoother_params.emplace_back(rclcpp::Parameter("max_decel", std::vector<double>{-10.0, 0.0, -10.0}));

    // Asynchronously set parameters for the global and local costmaps
    auto global_future = global_costmap_param_client_->set_parameters(global_params);
    auto local_future = local_costmap_param_client_->set_parameters(local_params);
    // auto controller_future = controller_server_param_client_->set_parameters(controller_params);
    // auto velocity_smoother_future = velocity_smoother_param_client_->set_parameters(velocity_smoother_params);

    // Handle the results
    try
    {
      // Wait for global costmap parameter set results
      auto global_results = global_future.get();
      for (size_t i = 0; i < global_results.size(); ++i)
      {
        const auto &result = global_results[i];
        const auto &param = global_params[i];
        if (!result.successful)
          RCLCPP_ERROR(this->get_logger(), "Failed to set parameter '%s' on global_costmap: %s",
                       param.get_name().c_str(), result.reason.c_str());
        else
          RCLCPP_INFO(this->get_logger(), "Successfully set parameter '%s' on global_costmap",
                      param.get_name().c_str());
      }

      // Wait for local costmap parameter set results
      auto local_results = local_future.get();
      for (size_t i = 0; i < local_results.size(); ++i)
      {
        const auto &result = local_results[i];
        const auto &param = local_params[i];
        if (!result.successful)
          RCLCPP_ERROR(this->get_logger(), "Failed to set parameter '%s' on local_costmap: %s",
                       param.get_name().c_str(), result.reason.c_str());
        else
          RCLCPP_INFO(this->get_logger(), "Successfully set parameter '%s' on local_costmap",
                      param.get_name().c_str());
      }

      // Controller Server Results
      // auto controller_results = controller_future.get();
      // for (size_t i = 0; i < controller_results.size(); ++i)
      // {
      //   const auto &result = controller_results[i];
      //   const auto &param = controller_params[i];
      //   if (!result.successful)
      //     RCLCPP_ERROR(this->get_logger(), "Failed to set parameter '%s' on controller_server: %s",
      //                  param.get_name().c_str(), result.reason.c_str());
      //   else
      //     RCLCPP_INFO(this->get_logger(), "Successfully set parameter '%s' on controller_server",
      //                 param.get_name().c_str());
      // }

      // // Velocity Smoother Results
      // auto velocity_smoother_results = velocity_smoother_future.get();
      // for (size_t i = 0; i < velocity_smoother_results.size(); ++i)
      // {
      //   const auto &result = velocity_smoother_results[i];
      //   const auto &param = velocity_smoother_params[i];
      //   if (!result.successful)
      //     RCLCPP_ERROR(this->get_logger(), "Failed to set parameter '%s' on velocity_smoother: %s",
      //                  param.get_name().c_str(), result.reason.c_str());
      //   else
      //     RCLCPP_INFO(this->get_logger(), "Successfully set parameter '%s' on velocity_smoother",
      //                 param.get_name().c_str());
      // }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception while setting parameters: %s", e.what());
    }
  }

  void Nav2ActionClient::SendGoal(const NavigateToPose::Goal &goal_msg)
  {
    single_goal_action_client_->wait_for_action_server();

    auto goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(&Nav2ActionClient::GoalResponseCallBack, this, _1);
    goal_options.feedback_callback = std::bind(&Nav2ActionClient::FeedbackCallBack, this, _1, _2);
    goal_options.result_callback = std::bind(&Nav2ActionClient::ResultCallBack, this, _1);

    single_goal_action_client_->async_send_goal(goal_msg, goal_options);
  }

  void Nav2ActionClient::CancelGoal()
  {
  }

  void Nav2ActionClient::GoalResponseCallBack(const NavigateToPoseGoalHandle::SharedPtr &goal_handle)
  {
    if (!goal_handle)
      RCLCPP_ERROR(this->get_logger(), "Goal is rejected by the nav2 action server");
    else
    {
      this->goal_handle_ = goal_handle;
      RCLCPP_INFO(this->get_logger(), "Goal is accepted by the nav2 action server");
    }
  }
  void Nav2ActionClient::FeedbackCallBack(const NavigateToPoseGoalHandle::SharedPtr &goal_handle,
                                          const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    // RCLCPP_INFO(this->get_logger(), "Received feedback from the nav2 action server");
    (void)goal_handle;

    // cout << "current position = " << feedback->current_pose.pose.position.x << " "
    //      << feedback->current_pose.pose.position.y << " "
    //      << feedback->current_pose.pose.position.z << endl;
    // cout << "current orientation = " << feedback->current_pose.pose.orientation.x << " "
    //      << feedback->current_pose.pose.orientation.y << " "
    //      << feedback->current_pose.pose.orientation.z << " "
    //      << feedback->current_pose.pose.orientation.w << endl;
    cout << "distance remaining = " << feedback->distance_remaining << endl;
    if (feedback->distance_remaining < 2.0 && feedback->distance_remaining >= 0.5 && !cost_map_is_active)
    {
      cout << " table detection acticating " << endl;
      // ActivateCostmapParameters(true);
      ActivatePCProcessing_Parameters("detect_table");
      cost_map_is_active = true;
    }
    else if (feedback->distance_remaining < 0.5 && cost_map_is_active)
    {
      cout << " table detection deacticating " << endl;
      // ActivateCostmapParameters(false);
      cost_map_is_active = false;
    }
    // 1 - activate
    // if (new obs) -> pc_processing_3 (find horizontal dominant plane) -> if(plane) -> cancel + waypoints, else -> do nothing
    // else, do nothing
  }
  void Nav2ActionClient::ResultCallBack(const NavigateToPoseGoalHandle::WrappedResult &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal is succeeded");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal is canceled");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Goal is aborted");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
    }
  }

  void Nav2ActionClient::SetInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &initial_pose)
  {
    RCLCPP_INFO(this->get_logger(), "Setting initial pose");
    for (int i = 0; i < 100; i++)
    {
      initial_pose_pub_->publish(initial_pose);
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }

  geometry_msgs::msg::PoseStamped Nav2ActionClient::GetCurrentPose()
  {
    geometry_msgs::msg::PoseStamped current_pose;

    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      current_pose.header = transform_stamped.header;
      current_pose.pose.position.x = transform_stamped.transform.translation.x;
      current_pose.pose.position.y = transform_stamped.transform.translation.y;
      current_pose.pose.position.z = transform_stamped.transform.translation.z;
      current_pose.pose.orientation = transform_stamped.transform.rotation;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not get current pose: %s", ex.what());
    }

    return current_pose;
  }

  void Nav2ActionClient::SendPlannerGoal(const geometry_msgs::msg::PoseStamped &goal_pose_stamped)
  {
    auto goal_msg = ComputePathToPose::Goal();
    goal_msg.goal = goal_pose_stamped;
    goal_msg.start = GetCurrentPose(); // Implement this method to get the robot's current pose
    goal_msg.planner_id = "";          // Use default planner
    goal_msg.use_start = false;        // Let the planner use the current pose

    auto goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(&Nav2ActionClient::PlannerGoalResponseCallback, this, _1);
    goal_options.result_callback = std::bind(&Nav2ActionClient::PlannerResultCallBack, this, _1);

    global_planner_action_client_->async_send_goal(goal_msg, goal_options);
  }

  void Nav2ActionClient::PlannerGoalResponseCallback(const ComputePathToPoseGoalHandle::SharedPtr &goal_handle)
  {
    if (!goal_handle)
      cout << "planner goal was rejected" << endl;
    else
    {
      this->path_goal_handle_ = goal_handle;
      cout << "planner goal was accepted" << endl;
    }
  }

  void Nav2ActionClient::PlannerResultCallBack(const ComputePathToPoseGoalHandle::WrappedResult &result)
  {
    nav_msgs::msg::Path path;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Received path from planner");
      path = result.result->path;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal is canceled");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Goal is aborted");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
    }
  }

  NavigateToPose::Goal Nav2ActionClient::GetGoalPose(const std::string &filename, const std::string &destination)
  {
    NavigateToPose::Goal goal;
    bool destination_isValid = false;

    try
    {
      YAML::Node config = YAML::LoadFile(filename);

      if (!config["locations"])
      {
        std::cerr << "Error: 'locations' key not found in the YAML file.\n";
        return goal;
      }

      for (const auto &item : config["locations"])
      {
        std::string key = item.first.as<std::string>();

        if (key == destination)
        {
          YAML::Node location = item.second;

          if (!location["position"] || !location["orientation"])
          {
            std::cerr << "Error: 'position' or 'orientation' key not found for destination '" << destination << "'.\n";
            return goal;
          }

          YAML::Node position = location["position"];
          YAML::Node orientation = location["orientation"];

          if (!position["x"] || !position["y"] || !position["z"])
          {
            std::cerr << "Error: 'x', 'y', or 'z' not found in 'position' for destination '" << destination << "'.\n";
            return goal;
          }

          if (!orientation["x"] || !orientation["y"] || !orientation["z"] || !orientation["w"])
          {
            std::cerr << "Error: 'x', 'y', 'z', or 'w' not found in 'orientation' for destination '" << destination << "'.\n";
            return goal;
          }

          goal.pose.pose.position.x = position["x"].as<double>();
          goal.pose.pose.position.y = position["y"].as<double>();
          goal.pose.pose.position.z = position["z"].as<double>();

          goal.pose.pose.orientation.x = orientation["x"].as<double>();
          goal.pose.pose.orientation.y = orientation["y"].as<double>();
          goal.pose.pose.orientation.z = orientation["z"].as<double>();
          goal.pose.pose.orientation.w = orientation["w"].as<double>();

          goal.pose.header.frame_id = "map";
          std::cout << "Destination: " << destination
                    << " | Position: (" << goal.pose.pose.position.x << ", "
                    << goal.pose.pose.position.y << ", "
                    << goal.pose.pose.position.z << ") "
                    << "| Orientation: (" << goal.pose.pose.orientation.x << ", "
                    << goal.pose.pose.orientation.y << ", "
                    << goal.pose.pose.orientation.z << ", "
                    << goal.pose.pose.orientation.w << ")\n";

          destination_isValid = true;
          break;
        }
      }
    }
    catch (const YAML::Exception &e)
    {
      std::cerr << "YAML Exception: " << e.what() << "\n";
    }

    if (destination_isValid)
      return goal;

    std::cerr << "Invalid Destination Name: " << destination << "\n";
    return goal;
  }

  rcl_interfaces::msg::SetParametersResult Nav2ActionClient::DestinationUpdatedCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    cout << "parameter callback called" << endl;
    bool destination_changed = false;
    rcl_interfaces::msg::SetParametersResult result;

    for (const auto &param : parameters)
    {
      if (param.get_name() == "destination")
      {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
        {
          destination_ = param.as_string();
          destination_changed = true;
          RCLCPP_INFO(this->get_logger(), "Destination parameter updated to: '%s'", destination_.c_str());
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Destination parameter has incorrect type. Expected string.");
          result.successful = false;
          result.reason = "Destination parameter must be a string.";
          return result;
        }
        break;
      }
    }

    result.successful = true;
    result.reason = "Parameters set successfully.";
    std::string yaml_file = "/home/akarshan/mobile_cobot_ws/src/r1d1_description/config/locations.yaml";
    NavigateToPose::Goal destination_goal = GetGoalPose(yaml_file, destination_);
    destination_goal.pose.header.stamp = this->now();
    this->SendGoal(destination_goal);
    return result;
  }
}
