#include "planner_module/planner_correction.hpp"
using namespace std::chrono_literals;
using namespace std;

namespace planner_correction
{
    AMRCorrection::AMRCorrection(rclcpp::Node::SharedPtr node, std::shared_ptr<DetectionTracker> detection_tracker__)
        : node_(node), detection_tracker_(detection_tracker__)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pc_processing_pub_ = node_->create_publisher<std_msgs::msg::String>("/activate_pc_processing", 10);
        start_odometry_check = correction_is_complete = false;
        table_detection_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/pointcloud_processor_node");
        arm_controller_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/custom_arm_controller_node");
    }

    void AMRCorrection::OdometryCheckCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg_)
    {
        if (start_odometry_check)
        {
            if (fabs(fabs(odom_msg_->pose.pose.orientation.x) - fabs(desired_pose_stamped.pose.orientation.x)) < 0.02 &&
                fabs(fabs(odom_msg_->pose.pose.orientation.y) - fabs(desired_pose_stamped.pose.orientation.y)) < 0.02 &&
                fabs(fabs(odom_msg_->pose.pose.orientation.z) - fabs(desired_pose_stamped.pose.orientation.z)) < 0.02 &&
                fabs(fabs(odom_msg_->pose.pose.orientation.w) - fabs(desired_pose_stamped.pose.orientation.w)) < 0.02)
            {
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd_vel);
                start_odometry_check = false;
                correction_is_complete = true;
                odometry_check_sub_.reset();
                std::cout << "subscriber is inactive" << std::endl;
            }
        }
    }

    void AMRCorrection::CorrectOrientation(bool publish_data)
    {
        start_odometry_check = true;
        correction_is_complete = false;

        tf2::Quaternion tf_quat;
        tf2::fromMsg(actual_pose_stamped.pose.orientation, tf_quat);
        double roll, pitch, actual_yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, actual_yaw);

        double angle_difference = desired_yaw_ - actual_yaw;
        // Normalize angle -> principle value (-pi to +pi)
        while (angle_difference > M_PI)
            angle_difference -= 2 * M_PI;
        while (angle_difference < -M_PI)
            angle_difference += 2 * M_PI;

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.angular.z = (angle_difference > 0) ? 1.5 : -1.5;

        rclcpp::Rate rate(10);
        correction_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(10),
            [this, cmd_vel, publish_data]() { // Removed 'mutable'
                if (!correction_is_complete)
                {
                    cmd_vel_pub_->publish(cmd_vel);
                }
                else
                {
                    geometry_msgs::msg::Twist cmd_vel_;
                    cmd_vel_.angular.z = 0.0;
                    for (int i = 0; i < 200; i++)
                    {
                        cmd_vel_pub_->publish(cmd_vel_);
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    }
                    // std_msgs::msg::String detection;
                    std::string detection = "detect_table";
                    if (publish_data)
                        ActivatePCProcessingParameters(detection);
                    else
                        ActivateArm_ForSnapshot();

                    correction_timer_.reset();
                }
            });
    }

    void AMRCorrection::ActivatePCProcessingParameters(const std::string &param)
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

    void AMRCorrection::ActivateArm_ForSnapshot()
    {
        if (!arm_controller_param_client_->wait_for_service(10s))
        {
            RCLCPP_ERROR(node_->get_logger(), "arm_pose_name parameter service not available.");
            return;
        }

        // vector of params of target node that i want to change
        std::vector<rclcpp::Parameter> params = {rclcpp::Parameter("arm_pose_name", "c1")};
        auto future = arm_controller_param_client_->set_parameters(params);

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

}