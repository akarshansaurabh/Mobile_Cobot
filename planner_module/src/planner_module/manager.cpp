#include "planner_module/manager.hpp"

namespace manager
{
    Manager::Manager(rclcpp::Node::SharedPtr node,
                     custom_nav2_action_client2::NavigateToPoseClient &navigate_to_pose_client,
                     custom_nav2_action_client2::NavigateThroughPosesClient &navigate_through_poses_client,
                     custom_nav2_action_client2::Nav2Utilities &nav2_utilities)
        : node_(node),
          navigate_to_pose_client_(navigate_to_pose_client),
          navigate_through_poses_client_(navigate_through_poses_client),
          nav2_utilities_(nav2_utilities)
    {
        box_poses_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
            "/box_poses_topic", rclcpp::QoS(10),
            std::bind(&Manager::BoxPosesCallBack, this, std::placeholders::_1));

        goal_completion_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/manager_nav2_topic", rclcpp::QoS(10),
            std::bind(&Manager::GoalCompletionCallBack, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "[Manager] Manager constructed.");
    }

    void Manager::GoalCompletionCallBack(const std_msgs::msg::Bool::ConstSharedPtr &box_poses_msg)
    {
        if (!all_waypoints.empty())
            all_waypoints.erase(all_waypoints.begin());
        if (!all_waypoints.empty())
        {
            nav2_msgs::action::NavigateToPose::Goal goal_msg;
            goal_msg.pose = all_waypoints[0];
            goal_msg.behavior_tree = "";
            navigate_to_pose_client_.SendGoal(goal_msg);
        }
        std::cout << "size after removing " << all_waypoints.size() << std::endl;
    }

    void Manager::BoxPosesCallBack(const geometry_msgs::msg::PoseArray::ConstSharedPtr &box_poses_msg)
    {
        box_poses_ = *box_poses_msg;
        RCLCPP_INFO(node_->get_logger(), "[Manager] Received %zu box poses.", box_poses_.poses.size());
        std::cout << "before sorting" << std::endl;
        for (const auto &pose : box_poses_msg->poses)
            std::cout << pose.position.x << " " << pose.position.y << " " << pose.position.z << " "
                      << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.y << " " << pose.orientation.w << std::endl;

        auto sequence_generation_lambda = [](geometry_msgs::msg::PoseArray &box_poses)
        {
            if (box_poses.poses.empty() || box_poses.poses.size() == 1)
                return;

            double x_center = 0.0;
            double y_center = 0.0;
            for (const auto &pose : box_poses.poses)
            {
                x_center += pose.position.x;
                y_center += pose.position.y;
            }
            x_center /= box_poses.poses.size();
            y_center /= box_poses.poses.size();

            double max_y = -std::numeric_limits<double>::infinity();
            int index_max_y = -1;
            for (int i = 0; i < box_poses.poses.size(); ++i)
            {
                if (box_poses.poses[i].position.y > max_y)
                {
                    max_y = box_poses.poses[i].position.y;
                    index_max_y = i;
                }
            }

            if (index_max_y == -1)
                return;

            double dx_start = box_poses.poses[index_max_y].position.x - x_center;
            double dy_start = box_poses.poses[index_max_y].position.y - y_center;
            double angle_start = std::atan2(dy_start, dx_start);

            std::vector<std::pair<geometry_msgs::msg::Pose, double>> pose_angle_pairs;
            pose_angle_pairs.reserve(box_poses.poses.size());

            for (const auto &pose : box_poses.poses)
            {
                double dx = pose.position.x - x_center;
                double dy = pose.position.y - y_center;
                double angle = std::atan2(dy, dx);

                double adjusted_angle = angle - angle_start;

                if (adjusted_angle < 0)
                    adjusted_angle += 2 * M_PI;
                pose_angle_pairs.emplace_back(pose, adjusted_angle);
            }

            std::sort(pose_angle_pairs.begin(), pose_angle_pairs.end(),
                      [](const std::pair<geometry_msgs::msg::Pose, double> &a,
                         const std::pair<geometry_msgs::msg::Pose, double> &b) -> bool
                      {
                          return a.second < b.second;
                      });

            for (size_t i = 0; i < box_poses.poses.size(); ++i)
                box_poses.poses[i] = pose_angle_pairs[i].first;
        };
        sequence_generation_lambda(box_poses_);
        std::cout << "after sorting" << std::endl;
        for (const auto &pose_ : box_poses_.poses)
            std::cout << pose_.position.x << " " << pose_.position.y << " " << pose_.position.z << " " << std::endl;

        GenerateWaypointsAndSend_1stGoal();
    }

    void Manager::GenerateWaypointsAndSend_1stGoal()
    {
        geometry_msgs::msg::PoseStamped current_robot_pose = getCurrentRobotPose();

        all_waypoints.reserve(box_poses_.poses.size() * 3);

        double current_y = current_robot_pose.pose.position.y;
        geometry_msgs::msg::Quaternion plus_90, original_orientation, minus_90;

        auto orientation_lambda = [](const geometry_msgs::msg::Quaternion &current_orientation, double tz)
            -> geometry_msgs::msg::Quaternion
        {
            Eigen::Quaterniond eigen_quat(current_orientation.w, current_orientation.x,
                                          current_orientation.y, current_orientation.z);
            Eigen::Matrix3d eigen_rot = eigen_quat.toRotationMatrix();
            auto rot_z_lambda = [](double t)
                -> Eigen::Matrix3d
            {
                Eigen::Matrix3d rz;
                rz << cos(t), -sin(t), 0,
                    sin(t), cos(t), 0,
                    0, 0, 1;
                return rz;
            };
            Eigen::Matrix3d goal_rot = eigen_rot * rot_z_lambda(tz);
            Eigen::Quaterniond goal_orientation(goal_rot);
            geometry_msgs::msg::Quaternion ans;
            ans.x = goal_orientation.x();
            ans.y = goal_orientation.y();
            ans.z = goal_orientation.z();
            ans.w = goal_orientation.w();
            return ans;
        };

        for (size_t i = 0; i < box_poses_.poses.size(); ++i) // box_poses_.poses.size()
        {
            double box_y = box_poses_.poses[i].position.y;
            double diff = (box_y - current_y);

            if (diff > 0.0 && i == 0) // +90,F,-90
            {
                std::cout << "plus plus plus plus plus plus plus plus plus " << std::endl;
                auto plus_90 = orientation_lambda(current_robot_pose.pose.orientation, 1.57);
                auto original_orientation = orientation_lambda(plus_90, 0.0);
                auto minus_90 = orientation_lambda(original_orientation, -1.57);

                auto pose1 = createPose(current_robot_pose, current_y, plus_90);
                auto pose2 = createPose(current_robot_pose, box_y + 0.4, original_orientation);
                auto pose3 = createPose(current_robot_pose, box_y + 0.4, minus_90);
                all_waypoints.push_back(pose1);
                all_waypoints.push_back(pose2);
                all_waypoints.push_back(pose3);
            }
            else // -90,F,+90
            {
                std::cout << "minus minus minus minus minus minus minus minus minus " << std::endl;
                auto minus_90 = orientation_lambda(current_robot_pose.pose.orientation, -1.57);
                auto original_orientation = orientation_lambda(minus_90, 0.0);
                auto plus_90 = orientation_lambda(original_orientation, 1.57);

                auto pose1 = createPose(current_robot_pose, current_y, minus_90);
                auto pose2 = createPose(current_robot_pose, box_y - 0.3, original_orientation);
                auto pose3 = createPose(current_robot_pose, box_y - 0.3, plus_90);
                all_waypoints.push_back(pose1);
                all_waypoints.push_back(pose2);
                all_waypoints.push_back(pose3);
            }
            current_robot_pose.pose = all_waypoints.back().pose;
        }

        // 3) Build the NavigateToPose goal
        std::cout << "waypoints" << std::endl;
        for (const auto &pose_ : all_waypoints)
            std::cout << pose_.pose.position.x << " " << pose_.pose.position.y << " " << pose_.pose.position.z << " "
                      << pose_.pose.orientation.x << " " << pose_.pose.orientation.y << " " << pose_.pose.orientation.z << " " << pose_.pose.orientation.w << std::endl;

        nav2_msgs::action::NavigateToPose::Goal goal_msg;
        goal_msg.pose = all_waypoints[0];
        goal_msg.behavior_tree = "";
        navigate_to_pose_client_.SendGoal(goal_msg);
    }

    geometry_msgs::msg::PoseStamped Manager::getCurrentRobotPose()
    {
        geometry_msgs::msg::PoseStamped pose_out = nav2_utilities_.GetCurrentPose();
        RCLCPP_DEBUG(node_->get_logger(),
                     "Current robot pose from Nav2Utilities: [%.2f, %.2f, %.2f]",
                     pose_out.pose.position.x, pose_out.pose.position.y,
                     tf2::getYaw(pose_out.pose.orientation));
        return pose_out;
    }

    // reference_pose is current pose
    geometry_msgs::msg::PoseStamped Manager::createPose(const geometry_msgs::msg::PoseStamped &reference_pose,
                                                        double new_y, const geometry_msgs::msg::Quaternion &new_orientation)
    {
        geometry_msgs::msg::PoseStamped result_pose = reference_pose;
        result_pose.pose.position.y = new_y;
        result_pose.pose.orientation = new_orientation;
        result_pose.header.stamp = node_->now();
        return result_pose;
    }

} // namespace manager
