#ifndef PLANNER_CORRECTION_HPP_
#define PLANNER_CORRECTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <iostream>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <atomic>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace planner_correction
{
    enum class DetectionTracker
    {
        DETECT_TABLE,
        DETECT_BOXES,
        DETECT_NOTHING
    };

    class AMRCorrection
    {
    private:
        std::string robot_base_frame_;
        std::string global_frame_;
        std::atomic<bool> start_odometry_check, correction_is_complete;
        rclcpp::TimerBase::SharedPtr correction_timer_;
        std::shared_ptr<DetectionTracker> detection_tracker_;

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pc_processing_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_completion_pub_;

        rclcpp::AsyncParametersClient::SharedPtr table_detection_param_client_;
        rclcpp::AsyncParametersClient::SharedPtr arm_controller_param_client_;
        void ActivatePCProcessingParameters(const std::string &param);
        void ActivateArm_ForSnapshot();

    public:
        explicit AMRCorrection(rclcpp::Node::SharedPtr node, std::shared_ptr<DetectionTracker> detection_tracker__);
        void CorrectOrientation(bool publish_data);
        ~AMRCorrection() = default;

        double desired_yaw_;
        geometry_msgs::msg::PoseStamped actual_pose_stamped, desired_pose_stamped;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_check_sub_;
        void OdometryCheckCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg_);
    };
}

#endif