#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "planner_module/arm_contoller.hpp"

using namespace std::chrono_literals;

namespace arm_planner
{
    ArmController::ArmController() : Node("custom_arm_controller_node")
    {
        // Initialize Action Client
        joint_trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/joint_trajectory_controller/follow_joint_trajectory");

        // Wait for the action server to be available
        if (!joint_trajectory_action_client_->wait_for_action_server(30s))
        {
            RCLCPP_ERROR(this->get_logger(), "Joint Trajectory Action Server not available after waiting");
            rclcpp::shutdown();
        }
        else
            RCLCPP_INFO(this->get_logger(), "Connected to Joint Trajectory Action Server");

        // parameter registration
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ArmController::ArmGoalUpdatedCallback, this, std::placeholders::_1));

        this->declare_parameter<std::string>("arm_pose_name", "nav_pose");
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
            RCLCPP_ERROR(this->get_logger(), "Joint Trajectory Action Server not available.");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory = trajectory;

        RCLCPP_INFO(this->get_logger(), "Sending Joint Trajectory Goal");

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
            RCLCPP_ERROR(this->get_logger(), "Joint Trajectory Goal was rejected by the server");
        else
        {
            this->arm_goal_hangle_ = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Joint Trajectory Goal was accepted by the server");
        }
    }

    // Action Callback: Feedback
    void ArmController::JointTrajectoryFeedbackCallback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle,
                                                        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Joint Trajectory Feedback received");
        std::string feedback_str = "Current Positions: ";
        for (const auto &pos : feedback->desired.positions)
            feedback_str += std::to_string(pos) + " ";
        RCLCPP_INFO(this->get_logger(), "%s", feedback_str.c_str());
    }

    // Action Callback: Result
    void ArmController::JointTrajectoryResultCallback(const GoalHandleFollowJointTrajectory::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Joint Trajectory Goal succeeded");
            /*
            if octomap geretation is on, then send the remaining view points to server one by one after the completion of the previous view point trajectory
            Kepp sending untill the last view point is sent. Once the last view point trajectory execution is finished, then stop sending the point
            
            */
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Joint Trajectory Goal was canceled");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(this->get_logger(), "Joint Trajectory Goal was aborted");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Joint Trajectory Goal received unknown result code");
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
                RCLCPP_ERROR(this->get_logger(), "Error: 'arm_poses' key not found in the YAML file.");
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
                        RCLCPP_ERROR(this->get_logger(), "Error: 'q1' to 'q6' not found for pose '%s'.", pose_.c_str());
                        return angles;
                    }
                    angles.push_back(pose["d1"].as<double>());
                    angles.push_back(pose["q1"].as<double>());
                    angles.push_back(pose["q2"].as<double>());
                    angles.push_back(pose["q3"].as<double>());
                    angles.push_back(pose["q4"].as<double>());
                    angles.push_back(pose["q5"].as<double>());
                    angles.push_back(pose["q6"].as<double>());

                    RCLCPP_INFO(this->get_logger(),
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
            RCLCPP_ERROR(this->get_logger(), "YAML Exception: %s", e.what());
        }

        if (pose_isValid)
            return angles;

        RCLCPP_ERROR(this->get_logger(), "Invalid Pose Name: %s", pose_.c_str());
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
                    RCLCPP_INFO(this->get_logger(), "Destination parameter updated to: '%s'", arm_goal_pose_name_.c_str());
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
        std::string yaml_file = "/home/akarshan/mobile_cobot_ws/src/r1d1_description/config/arm_poses.yaml";
        std::vector<double> arm_goal_positions = GetArmGoalPose(yaml_file, arm_goal_pose_name_);
        trajectory_msgs::msg::JointTrajectory arm_goal = CreateJointTrajectory(arm_goal_positions, 5.0);
        this->SendJointTrajectoryGoal(arm_goal);
        return result;
    }
}
