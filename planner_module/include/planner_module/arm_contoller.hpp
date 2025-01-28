#ifndef PLANNER_MODULE_ARM_CONTROLLER_HPP
#define PLANNER_MODULE_ARM_CONTROLLER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <yaml-cpp/yaml.h>

#include "pc_processing/octomap_generator.hpp"
#include "planner_module/kinematics.hpp"

#include "custom_interfaces/srv/goal_pose_vector.hpp"
#include "planner_module/multithreaded_fcl_loader.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace arm_planner
{
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    class ArmController
    {
    public:
        ArmController(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<cMRKinematics::ArmKinematicsSolver<7>> &kinematics_solver,
                      const std::vector<std::shared_ptr<fcl::CollisionObjectf>> &collision_objects);
        ~ArmController() = default;

    private:
        rclcpp::Node::SharedPtr node_;

        // composition
        std::shared_ptr<octoMapGenerator::OctoMapGenerator> octoMap_generator_;
        std::shared_ptr<cMRKinematics::ArmKinematicsSolver<7>> kinematics_solver_;
        std::shared_ptr<visualization::VisualizationManager> viz_manager_;

        std::vector<std::shared_ptr<fcl::CollisionObjectf>> collision_objects_;

        std::atomic<bool> activate_arm_motion_planning_, previous_c3_, subs_callback_rejected_;
        std::atomic<bool> ask_manager_to_process;

        GoalHandleFollowJointTrajectory::SharedPtr arm_goal_hangle_;
        std::string arm_goal_pose_name_, yaml_file_;
        geometry_msgs::msg::PoseArray box_6d_poses_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_completion_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr arm_goal_by_manager_sub_;

        void ManagerCallback(const geometry_msgs::msg::Pose::ConstSharedPtr &msg);
        rclcpp::Client<custom_interfaces::srv::GoalPoseVector>::SharedPtr colision_free_planner_client;
        void SendRequestForColisionFreePlanning(geometry_msgs::msg::PoseArray &box_poses);
        void HandleResponse(rclcpp::Client<custom_interfaces::srv::GoalPoseVector>::SharedFuture future);

        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint_trajectory_action_client_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub_;

        void proceedToNextViewpoint(std::string str);
        void triggerSnapshotForCurrentViewpoint(bool stitch);

        // Methods
        void SendJointTrajectoryGoal(const trajectory_msgs::msg::JointTrajectory &trajectory);
        // void BoxPosesCallBack(const geometry_msgs::msg::PoseArray::ConstSharedPtr &box_poses_msg);
        trajectory_msgs::msg::JointTrajectory CreateJointTrajectory(const std::vector<double> &positions, double execution_time);
        vector<double> GetArmGoalPose(const std::string &filename, const std::string &pose_);

        // Action Callbacks
        void JointTrajectoryGoalResponseCallback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle);
        void JointTrajectoryFeedbackCallback(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle,
                                             const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback);
        void JointTrajectoryResultCallback(const GoalHandleFollowJointTrajectory::WrappedResult &result);

        // ros2 parameter callbacks
        rcl_interfaces::msg::SetParametersResult ArmGoalUpdatedCallback(const std::vector<rclcpp::Parameter> &parameters);
        std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> parameter_callback_handle_; // used for parameter registration

        static std::mutex m;
        static std::condition_variable cv;
        static bool callback_triggered;
        bool activate_arm_motion_;
        std::vector<std::vector<double>> joint_states_vector_;

        // TF2 buffer and listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
}

#endif
