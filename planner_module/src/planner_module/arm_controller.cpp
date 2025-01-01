#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "planner_module/arm_contoller.hpp"

using namespace std::chrono_literals;

namespace arm_planner
{
    ArmController::ArmController(const rclcpp::Node::SharedPtr &node) : node_(node)
    {
        octoMap_generator_ = std::make_shared<octoMapGenerator::OctoMapGenerator>(node_);
        // Initialize Action Client
        joint_trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            node_, "/joint_trajectory_controller/follow_joint_trajectory");

        // Wait for the action server to be available
        if (!joint_trajectory_action_client_->wait_for_action_server(30s))
        {
            RCLCPP_ERROR(node_->get_logger(), "Joint Trajectory Action Server not available after waiting");
            rclcpp::shutdown();
        }
        else
            RCLCPP_INFO(node_->get_logger(), "Connected to Joint Trajectory Action Server");

        // parameter registration
        parameter_callback_handle_ = node_->add_on_set_parameters_callback(
            std::bind(&ArmController::ArmGoalUpdatedCallback, this, std::placeholders::_1));

        node_->declare_parameter<std::string>("arm_pose_name", "nav_pose");

        yaml_file_ = "/home/akarshan/mobile_cobot_ws/src/r1d1_description/config/arm_poses.yaml";

        octomap_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/octomap_topic_", rclcpp::QoS(10));

        box_poses_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/box_poses_topic", rclcpp::QoS(10),
            std::bind(&ArmController::BoxPosesCallBack, this, std::placeholders::_1));

        colision_free_planner_client = node_->create_client<custom_interfaces::srv::GoalPoseVector>("colision_free_planner_service");

        activate_arm_motion_planning_ = false;
    }

    void ArmController::SendRequestForColisionFreePlanning(const geometry_msgs::msg::PoseArray &box_poses)
    {
        if (!colision_free_planner_client->wait_for_service(5s)) // Wait for 5 second
        {
            RCLCPP_ERROR(node_->get_logger(), "Service 'colision_free_planner_service' not available.");
            return;
        }

        auto request = std::make_shared<custom_interfaces::srv::GoalPoseVector::Request>();

        request->goal_poses_for_arm = box_poses;
        auto future = colision_free_planner_client->async_send_request(request, std::bind(&ArmController::HandleResponse, this, std::placeholders::_1));
    }
    void ArmController::HandleResponse(rclcpp::Client<custom_interfaces::srv::GoalPoseVector>::SharedFuture future)
    {
        auto result = future.get();
        if (result->reply)
        {
            RCLCPP_INFO(node_->get_logger(), "Service call succeeded: ");
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Service call failed: ");
        }
    }

    void ArmController::BoxPosesCallBack(const geometry_msgs::msg::PoseArray::ConstSharedPtr &box_poses_msg)
    {
        // print the poses when activate_arm_motion_planning_ is true;
        if (!activate_arm_motion_planning_)
        {
            while (!activate_arm_motion_planning_)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        /// service call
        SendRequestForColisionFreePlanning(*box_poses_msg);
        for (const auto &pose : box_poses_msg->poses)
            std::cout << pose.position.x << " " << pose.position.y << " " << pose.position.y << " "
                      << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.y << " " << pose.orientation.w << std::endl;
    }

    // Method to create a joint trajectory
    trajectory_msgs::msg::JointTrajectory ArmController::CreateJointTrajectory(const std::vector<double> &positions, double execution_time)
    {
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = {
            "slider",
            "shoulder_joint",
            "upperarm_joint",
            "forearm_joint",
            "wrist_joint_1",
            "wrist_joint_2",
            "wrist_joint_3"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start = rclcpp::Duration::from_seconds(execution_time);
        trajectory.points.push_back(std::move(point));
        return trajectory;
    }

    // Method to send a joint trajectory goal
    void ArmController::SendJointTrajectoryGoal(const trajectory_msgs::msg::JointTrajectory &trajectory)
    {
        // wait for the server for a max 30s
        if (!joint_trajectory_action_client_->wait_for_action_server(30s))
        {
            RCLCPP_ERROR(node_->get_logger(), "Joint Trajectory Action Server not available.");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory = trajectory;

        RCLCPP_INFO(node_->get_logger(), "Sending Joint Trajectory Goal");

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ArmController::JointTrajectoryGoalResponseCallback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&ArmController::JointTrajectoryFeedbackCallback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&ArmController::JointTrajectoryResultCallback, this, _1);
        // send the goal asynchronously
        joint_trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // Action Callback: Goal Response
    void ArmController::JointTrajectoryGoalResponseCallback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle)
    {
        if (!goal_handle)
            RCLCPP_ERROR(node_->get_logger(), "Joint Trajectory Goal was rejected by the server");
        else
        {
            arm_goal_hangle_ = goal_handle;
            RCLCPP_INFO(node_->get_logger(), "Joint Trajectory Goal was accepted by the server");
        }
    }

    // Action Callback: Feedback
    void ArmController::JointTrajectoryFeedbackCallback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle,
                                                        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
    }

    void ArmController::proceedToNextViewpoint(std::string str)
    {
        arm_goal_pose_name_ = str;
        std::vector<double> arm_goal_positions = GetArmGoalPose(yaml_file_, str);
        trajectory_msgs::msg::JointTrajectory arm_goal = CreateJointTrajectory(arm_goal_positions, 1.0);
        this->SendJointTrajectoryGoal(arm_goal);
    }

    void ArmController::triggerSnapshotForCurrentViewpoint(bool stitch)
    {
        RCLCPP_INFO(node_->get_logger(), "Triggering snapshot for viewpoint: %s", arm_goal_pose_name_.c_str());

        static std::mutex m;
        static std::condition_variable cv;
        static bool callback_triggered = false;
        // Reset callback_triggered before starting a new wait
        callback_triggered = false;

        std::thread([this, stitch]()
                    {
                        auto stitcher = std::make_shared<octoMapGenerator::PointCloudStitcher>(node_, "map", m, cv, callback_triggered, 
                                                                                                      stitch, octomap_pub_);  //accumulated_cloud_
                        RCLCPP_INFO(node_->get_logger(), "Stitcher object created in separate thread, waiting for snapshots.");

                        // Wait until callback is triggered
                        {
                            std::unique_lock<std::mutex> lock(m);
                            // Wait until callback_triggered is true
                            cv.wait(lock, [&callback_triggered]()
                                    { return callback_triggered; });
                        }

                        // this->accumulated_cloud_ = stitcher->accumulated_cloud_;
                        
                        RCLCPP_INFO(node_->get_logger(), "Snapshot callback triggered, proceeding..."); })
            .detach();
        // std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    // Action Callback: Result
    void ArmController::JointTrajectoryResultCallback(const GoalHandleFollowJointTrajectory::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "Joint Trajectory Goal succeeded");
            if (arm_goal_pose_name_ == "c1")
            {
                triggerSnapshotForCurrentViewpoint(false);
                proceedToNextViewpoint("c2");
            }
            else if (arm_goal_pose_name_ == "c2")
            {
                triggerSnapshotForCurrentViewpoint(false);
                proceedToNextViewpoint("c3");
            }
            else if (arm_goal_pose_name_ == "c3")
            {
                triggerSnapshotForCurrentViewpoint(true);
                proceedToNextViewpoint("nav_pose");
                activate_arm_motion_ = true;
            }
            else if (arm_goal_pose_name_ == "nav_pose")
            {
                activate_arm_motion_planning_ = true;
                // update a ros2 param of
            }

            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Joint Trajectory Goal was canceled");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node_->get_logger(), "Joint Trajectory Goal was aborted");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Joint Trajectory Goal received unknown result code");
            break;
        }
    }

    std::vector<double> ArmController::GetArmGoalPose(const std::string &filename, const std::string &pose_)
    {
        std::vector<double> angles;
        bool pose_isValid = false;

        try
        {
            YAML::Node config = YAML::LoadFile(filename);

            if (!config["arm_poses"])
            {
                RCLCPP_ERROR(node_->get_logger(), "Error: 'arm_poses' key not found in the YAML file.");
                return angles;
            }

            for (const auto &item : config["arm_poses"])
            {
                std::string key = item.first.as<std::string>();

                if (key == pose_)
                {
                    YAML::Node pose = item.second;
                    if (!pose["d1"] || !pose["q1"] || !pose["q2"] || !pose["q3"] || !pose["q4"] || !pose["q5"] || !pose["q6"])
                    {
                        RCLCPP_ERROR(node_->get_logger(), "Error: 'q1' to 'q6' not found for pose '%s'.", pose_.c_str());
                        return angles;
                    }
                    angles.push_back(pose["d1"].as<double>());
                    angles.push_back(pose["q1"].as<double>());
                    angles.push_back(pose["q2"].as<double>());
                    angles.push_back(pose["q3"].as<double>());
                    angles.push_back(pose["q4"].as<double>());
                    angles.push_back(pose["q5"].as<double>());
                    angles.push_back(pose["q6"].as<double>());

                    RCLCPP_INFO(node_->get_logger(),
                                "Loaded Pose: %s | Angles: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                                pose_.c_str(),
                                angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6]);

                    pose_isValid = true;
                    break;
                }
            }
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "YAML Exception: %s", e.what());
        }

        if (pose_isValid)
            return angles;

        RCLCPP_ERROR(node_->get_logger(), "Invalid Pose Name: %s", pose_.c_str());
        return angles;
    }

    rcl_interfaces::msg::SetParametersResult ArmController::ArmGoalUpdatedCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for (const auto &param : parameters)
        {
            if (param.get_name() == "arm_pose_name")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
                {
                    arm_goal_pose_name_ = param.as_string();
                    RCLCPP_INFO(node_->get_logger(), "Destination parameter updated to: '%s'", arm_goal_pose_name_.c_str());
                }
                else
                {
                    RCLCPP_ERROR(node_->get_logger(), "Destination parameter has incorrect type. Expected string.");
                    result.successful = false;
                    result.reason = "Destination parameter must be a string.";
                    return result;
                }
                break;
            }
        }

        result.successful = true;
        result.reason = "Parameters set successfully.";
        std::string yaml_file_ = "/home/akarshan/mobile_cobot_ws/src/r1d1_description/config/arm_poses.yaml";
        std::vector<double> arm_goal_positions = GetArmGoalPose(yaml_file_, arm_goal_pose_name_);
        double execution_time_ = 4.0;
        if (arm_goal_pose_name_ == "c1" || arm_goal_pose_name_ == "c2" || arm_goal_pose_name_ == "c3")
            execution_time_ = 2.0;
        trajectory_msgs::msg::JointTrajectory arm_goal = CreateJointTrajectory(arm_goal_positions, execution_time_);
        this->SendJointTrajectoryGoal(arm_goal);
        return result;
    }

}
