#include <chrono>
#include <functional>
#include <future>
#include <vector>
#include <memory>
#include <string>
#include "planner_module/arm_contoller.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

namespace arm_planner
{
    ArmController::ArmController(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<cMRKinematics::ArmKinematicsSolver> &kinematics_solver,
                                 const std::vector<std::shared_ptr<fcl::CollisionObjectf>> &collision_objects)
        : node_(node), kinematics_solver_(kinematics_solver), collision_objects_(collision_objects)
    {
        octoMap_generator_ = std::make_shared<octoMapGenerator::OctoMapGenerator>(node_, kinematics_solver_, collision_objects_);
        viz_manager_ = std::make_shared<visualization::VisualizationManager>(node_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
        previous_c3_ = false;
        subs_callback_rejected_ = false;
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
            joint_states_vector_.resize(result->joint_states_vector.size());
            for (int i = 1; i < result->joint_states_vector.size(); i++)
            {
                joint_states_vector_[i].push_back(result->joint_states_vector[i].data[0]);
                joint_states_vector_[i].push_back(result->joint_states_vector[i].data[1]);
                joint_states_vector_[i].push_back(result->joint_states_vector[i].data[2]);
                joint_states_vector_[i].push_back(result->joint_states_vector[i].data[3]);
                joint_states_vector_[i].push_back(result->joint_states_vector[i].data[4]);
                joint_states_vector_[i].push_back(result->joint_states_vector[i].data[5]);
                joint_states_vector_[i].push_back(result->joint_states_vector[i].data[6]);
            }
            auto next_joint_trajectory = CreateJointTrajectory(result->joint_states_vector[1].data, 3.0);
            arm_goal_pose_name_ = "picking";
            SendJointTrajectoryGoal(next_joint_trajectory);
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
            std::cout << "rejected" << std::endl;
            subs_callback_rejected_ = true;
            box_6d_poses_ = *box_poses_msg;
            return;
        }
        /// service call
        SendRequestForColisionFreePlanning(*box_poses_msg);
        activate_arm_motion_planning_ = false;
        for (const auto &pose : box_poses_msg->poses)
            std::cout << pose.position.x << " " << pose.position.y << " " << pose.position.z << " "
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
        cMRKinematics::state_info_.joint_states[0] = kinematics_solver_->initial_guess(0) = feedback->actual.positions[0];
        cMRKinematics::state_info_.joint_states[1] = kinematics_solver_->initial_guess(1) = feedback->actual.positions[1];
        cMRKinematics::state_info_.joint_states[2] = kinematics_solver_->initial_guess(2) = feedback->actual.positions[2];
        cMRKinematics::state_info_.joint_states[3] = kinematics_solver_->initial_guess(3) = feedback->actual.positions[3];
        cMRKinematics::state_info_.joint_states[4] = kinematics_solver_->initial_guess(4) = feedback->actual.positions[4];
        cMRKinematics::state_info_.joint_states[5] = kinematics_solver_->initial_guess(5) = feedback->actual.positions[5];
        cMRKinematics::state_info_.joint_states[6] = kinematics_solver_->initial_guess(6) = feedback->actual.positions[6];
       
        std::vector<KDL::Frame> all_link_poses;
        kinematics_solver_->SolveFKAllLinks(kinematics_solver_->initial_guess, all_link_poses);

        geometry_msgs::msg::TransformStamped map_to_base_transform;
        try
        {
            map_to_base_transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
        }
        catch (tf2::ExtrapolationException &ex)
        {
            RCLCPP_WARN(node_->get_logger(), "fcl transformation issue.");
            return;
        }

        Eigen::Matrix4d map_to_base = Eigen::Matrix4d::Identity();
        map_to_base(0, 3) = map_to_base_transform.transform.translation.x;
        map_to_base(1, 3) = map_to_base_transform.transform.translation.y;
        map_to_base(2, 3) = map_to_base_transform.transform.translation.z;
        Eigen::Quaterniond q(map_to_base_transform.transform.rotation.w, map_to_base_transform.transform.rotation.x,
                             map_to_base_transform.transform.rotation.y, map_to_base_transform.transform.rotation.z);
        q.normalize();
        map_to_base.block<3, 3>(0, 0) = q.toRotationMatrix();

        std::string map_ = "map";
        std::vector<std::future<visualization_msgs::msg::MarkerArray>> futures;
        futures.reserve(8);

        // lambda
        auto process_collision = [this, &map_, &all_link_poses, &map_to_base](int i) -> visualization_msgs::msg::MarkerArray
        {
            Eigen::Matrix4d map_to_link = map_to_base * Conversions::KDL_2Transform(all_link_poses[i]);
            collision_objects_[i + 1]->setTranslation(fcl::Vector3f(map_to_link(0, 3), map_to_link(1, 3), map_to_link(2, 3)));
            collision_objects_[i + 1]->setRotation(map_to_link.block<3, 3>(0, 0).cast<float>());
            return viz_manager_->createTriangleMarker(collision_objects_[i + 1], 404 + i, map_);
        };

        for (int i = 0; i <= 7; ++i)
        {
            futures.emplace_back(std::async(std::launch::async, process_collision, i));
        }
        visualization_msgs::msg::MarkerArray global_marker_array;
        for (auto &f : futures)
        {
            try
            {
                visualization_msgs::msg::MarkerArray marker_array = f.get();
                global_marker_array.markers.insert(global_marker_array.markers.end(), marker_array.markers.begin(), marker_array.markers.end());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(node_->get_logger(), "Exception in async task: %s", e.what());
            }
            catch (...)
            {
                RCLCPP_ERROR(node_->get_logger(), "Unknown exception in async task.");
            }
        }
        viz_manager_->marker_pub_->publish(global_marker_array);
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
                previous_c3_ = false;
            }
            else if (arm_goal_pose_name_ == "c2")
            {
                triggerSnapshotForCurrentViewpoint(false);
                proceedToNextViewpoint("c3");
                previous_c3_ = false;
            }
            else if (arm_goal_pose_name_ == "c3")
            {
                triggerSnapshotForCurrentViewpoint(true);
                proceedToNextViewpoint("nav_pose");
                previous_c3_ = true;
            }
            else if (arm_goal_pose_name_ == "nav_pose")
            {
                if (previous_c3_)
                {
                    activate_arm_motion_planning_ = true;
                }
                previous_c3_ = false;
                if (subs_callback_rejected_)
                {
                    SendRequestForColisionFreePlanning(box_6d_poses_);
                    activate_arm_motion_planning_ = false;
                    subs_callback_rejected_ = false;
                }
            }
            else if (arm_goal_pose_name_ == "picking")
            {

                auto next_joint_trajectory = CreateJointTrajectory(joint_states_vector_[3], 10.0);
                arm_goal_pose_name_ = "stop";
                std::cout << joint_states_vector_[3][0] << std::endl;
                std::cout << joint_states_vector_[3][0] << std::endl;
                std::cout << joint_states_vector_[3][0] << std::endl;
                std::cout << joint_states_vector_[3][0] << std::endl;
                std::cout << joint_states_vector_[3][0] << std::endl;
                std::cout << joint_states_vector_[3][0] << std::endl;
                std::cout << joint_states_vector_[3][0] << std::endl;
                std::cout << joint_states_vector_[3][0] << std::endl;

                SendJointTrajectoryGoal(next_joint_trajectory);
            }
            else if (arm_goal_pose_name_ == "stop")
            {
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
