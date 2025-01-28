#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "navigation/nav2_client3.hpp"
#include <eigen3/Eigen/Dense>

namespace manager
{
    struct GoalIndexTracker
    {
        int num_of_boxes, current_amr_pose_index, current_arm_pose_index;
    };

    class Manager
    {
    public:
        Manager(rclcpp::Node::SharedPtr node,
                custom_nav2_action_client2::NavigateToPoseClient &navigate_to_pose_client,
                custom_nav2_action_client2::NavigateThroughPosesClient &navigate_through_poses_client,
                custom_nav2_action_client2::Nav2Utilities &nav2_utilities);
        void GenerateWaypointsAndSend_1stGoal();

    private:
        GoalIndexTracker goal_index_tracker_;

        geometry_msgs::msg::PoseStamped getCurrentRobotPose();
        geometry_msgs::msg::PoseStamped createPose(const geometry_msgs::msg::PoseStamped &reference_pose, double new_x,
                                                   double new_y, const geometry_msgs::msg::Quaternion &new_orientation);

        void BoxPosesCallBack(const geometry_msgs::msg::PoseArray::ConstSharedPtr &box_poses_msg);
        void GoalCompletionCallBack(const std_msgs::msg::Bool::ConstSharedPtr &box_poses_msg);
        void ArmGoalCompletionCallBack(const std_msgs::msg::Bool::ConstSharedPtr &msg);

        rclcpp::Node::SharedPtr node_;
        custom_nav2_action_client2::NavigateThroughPosesClient &navigate_through_poses_client_;
        custom_nav2_action_client2::NavigateToPoseClient &navigate_to_pose_client_;
        custom_nav2_action_client2::Nav2Utilities &nav2_utilities_;

        geometry_msgs::msg::PoseArray box_poses_;
        std::vector<geometry_msgs::msg::PoseStamped> all_waypoints;

        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr box_poses_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_completion_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_goal_completion_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr arm_goal_by_manager_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_octamap_pub_;
    };
}

#endif
