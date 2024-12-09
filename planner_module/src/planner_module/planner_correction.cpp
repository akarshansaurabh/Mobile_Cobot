#include "planner_module/planner_correction.hpp"

namespace planner_correction
{
    AMRCorrection::AMRCorrection(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pc_processing_pub_ = node_->create_publisher<std_msgs::msg::String>("/activate_pc_processing", 10);
        start_odometry_check = correction_is_complete = false;
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
        std::cout << "angle_difference " << angle_difference << std::endl;

        // Normalize angle -> principle value (-pi to +pi)
        while (angle_difference > M_PI)
            angle_difference -= 2 * M_PI;
        while (angle_difference < -M_PI)
            angle_difference += 2 * M_PI;

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.angular.z = (angle_difference > 0) ? 1.5 : -1.5;
        if (angle_difference > 0)
            std::cout << "anti" << std::endl;
        else
            std::cout << "clockwise" << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        rclcpp::Rate rate(10);
        std::cout << "correction started!" << std::endl;
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
                    std::cout << "correction completed!" << std::endl;
                    std::cout << "stopping" << std::endl;
                    for (int i = 0; i < 200; i++)
                    {
                        cmd_vel_pub_->publish(cmd_vel_);
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    }
                    std_msgs::msg::String table_detection;
                    if (publish_data)
                    {
                        table_detection.data = "detect_table";
                        std::cout << "table detection!" << std::endl;
                        pc_processing_pub_->publish(table_detection);
                    }
                    else
                    {
                        std::cout << "boxes 6d pose estimation!" << std::endl;
                        table_detection.data = "detect_boxes";
                        pc_processing_pub_->publish(table_detection);
                    }

                    correction_timer_.reset();
                }
            });
    }
}