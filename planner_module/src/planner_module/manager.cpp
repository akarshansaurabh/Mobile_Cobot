#include "planner_module/manager.hpp"
#include "maths/commonmathssolver.hpp"

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

        arm_goal_completion_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/arm_goal_completion_topic", rclcpp::QoS(10),
            std::bind(&Manager::ArmGoalCompletionCallBack, this, std::placeholders::_1));

        table_vertices_sub_ = node_->create_subscription<geometry_msgs::msg::Polygon>(
            "/table_vertices_topic", rclcpp::QoS(10),
            std::bind(&Manager::TableVerticesCompletionCallBack, this, std::placeholders::_1));

        arm_goal_by_manager_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("/arm_goal_by_manager_topic", 10);
        clear_octamap_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/clear_octamap_topic", 10);

        goal_index_tracker_.current_amr_pose_index = 0;
        goal_index_tracker_.current_arm_pose_index = 0;
        box_poses_.poses.clear();

        Rab.setIdentity();
        Rbc << 0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0;

        RCLCPP_INFO(node_->get_logger(), "[Manager] Manager constructed.");
    }

    // after nth arm goal completion, goal_index_tracker_.current_arm_pose_index = n
    void Manager::ArmGoalCompletionCallBack(const std_msgs::msg::Bool::ConstSharedPtr &msg)
    {
        if (!all_waypoints.empty())
            all_waypoints.erase(all_waypoints.begin());
        if (!all_waypoints.empty())
        {
            nav2_msgs::action::NavigateToPose::Goal goal_msg;
            goal_msg.pose = all_waypoints[0];
            goal_msg.behavior_tree = "";
            navigate_to_pose_client_.SendGoal(goal_msg);
            goal_index_tracker_.current_amr_pose_index++;
        }
        if (goal_index_tracker_.current_arm_pose_index == box_poses_.poses.size())
        {
            std::cout << "all arm goals handled, hence clearing octomap" << std::endl;
            goal_index_tracker_.current_arm_pose_index = 0;
            std_msgs::msg::Bool msg;
            msg.data = true;
            clear_octamap_pub_->publish(msg);
        }
        std::cout << "size after removing " << all_waypoints.size() << std::endl;
    }

    void Manager::GoalCompletionCallBack(const std_msgs::msg::Bool::ConstSharedPtr &box_poses_msg)
    {
        // if (goal_index_tracker_.current_amr_pose_index % 3 == 0)
        if (goal_index_tracker_.current_amr_pose_index > 0 &&
            all_waypoints_with_arm_activation_info_[goal_index_tracker_.current_amr_pose_index - 1].second)
        {
            if (!box_poses_.poses.empty())
            {
                geometry_msgs::msg::Pose msg;
                msg = box_poses_.poses[goal_index_tracker_.current_arm_pose_index];
                arm_goal_by_manager_pub_->publish(msg);
                goal_index_tracker_.current_arm_pose_index++;
                std::cout << "goal is sent to arm bt manager" << std::endl;
            }
            return;
        }

        if (!all_waypoints.empty())
            all_waypoints.erase(all_waypoints.begin());
        if (!all_waypoints.empty())
        {
            nav2_msgs::action::NavigateToPose::Goal goal_msg;
            goal_msg.pose = all_waypoints[0];
            goal_msg.behavior_tree = "";
            navigate_to_pose_client_.SendGoal(goal_msg);
            goal_index_tracker_.current_amr_pose_index++;
        }
        if (all_waypoints.size() == 0)
        {
            goal_index_tracker_.current_amr_pose_index = 0;
        }
        std::cout << "size after removing " << all_waypoints.size() << std::endl;
    }

    void Manager::TableVerticesCompletionCallBack(const geometry_msgs::msg::Polygon::ConstSharedPtr &table_vertices_msg)
    {
        table_vertices.clear();
        geometry_msgs::msg::Point xyz;
        for (const auto &v : table_vertices_msg->points)
        {
            xyz.x = v.x;
            xyz.y = v.y;
            xyz.z = v.z;
            table_vertices.push_back(xyz);
        }
        std::cout << "table ABCD " << std::endl;
        for (const auto &pose_ : table_vertices)
            std::cout << pose_.x << " " << pose_.y << " " << pose_.z << " " << std::endl;
    }

    void Manager::BoxPosesCallBack(const geometry_msgs::msg::PoseArray::ConstSharedPtr &box_poses_msg)
    {
        box_poses_ = *box_poses_msg;
        RCLCPP_INFO(node_->get_logger(), "[Manager] Received %zu box poses.", box_poses_.poses.size());

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

    void Manager::GenerateIntermediateWaypoint_BetweenCurrentAndGoal(const geometry_msgs::msg::Pose &start_pose, const geometry_msgs::msg::Pose &end_pose,
                                                                     const geometry_msgs::msg::Point &box)
    {
        Eigen::Matrix4d Tstart = Conversions::Pose_2Eigen(start_pose);
        Eigen::Matrix4d Tend = Conversions::Pose_2Eigen(end_pose);

        Eigen::Matrix3d Rstart = Tstart.block<3, 3>(0, 0);
        Eigen::Matrix3d Rend = Tend.block<3, 3>(0, 0);
        Eigen::Vector3d tstart = Tstart.block<3, 1>(0, 3);
        Eigen::Vector3d tend = Tend.block<3, 1>(0, 3);

        Eigen::Vector3d xcap_start = Rstart.col(0);
        Eigen::Vector3d xcap_end = Rend.col(0);

        double angle = CommonMathsSolver::Vectors3D::AngleBetween(xcap_start, xcap_end) * 180.0 / M_PI;
        constexpr double ANGLE_TOLERANCE = 5.0; // tolerance in degrees

        auto BuildRotationFromX = [](const Eigen::Vector3d &xcap)
        {
            Eigen::Vector3d x = xcap.normalized();
            Eigen::Vector3d z(0, 0, 1);
            Eigen::Vector3d y = z.cross(x).normalized();

            Eigen::Matrix3d R;
            R.col(0) = x;
            R.col(1) = y;
            R.col(2) = z;
            return R;
        };

        auto MakePoseStamped = [this](const Eigen::Vector3d &trans,
                                      const Eigen::Matrix3d &rot,
                                      const std::string &frame_id = "map")
            -> geometry_msgs::msg::PoseStamped
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = frame_id;
            ps.header.stamp = node_->now();
            // Set position
            ps.pose.position.x = trans.x();
            ps.pose.position.y = trans.y();
            ps.pose.position.z = trans.z();
            // Convert rotation matrix -> quaternion
            Eigen::Quaterniond q(rot);
            q.normalize();
            ps.pose.orientation.x = q.x();
            ps.pose.orientation.y = q.y();
            ps.pose.orientation.z = q.z();
            ps.pose.orientation.w = q.w();
            return ps;
        };

        if (std::fabs(angle - 0.0) < ANGLE_TOLERANCE)
        {
            Eigen::Vector3d line = (tend - tstart);
            if (line.norm() < 1e-9)
            {
                std::cout << "Start and End are the same. No intermediate waypoints.\n";
                return;
            }
            Eigen::Vector3d xcap = line.normalized();
            Eigen::Matrix3d R1 = BuildRotationFromX(xcap);
            geometry_msgs::msg::PoseStamped P1 = MakePoseStamped(tstart, R1);
            geometry_msgs::msg::PoseStamped P2 = MakePoseStamped(tend, R1);
            geometry_msgs::msg::PoseStamped P3 = MakePoseStamped(tend, Rend);
            all_waypoints.push_back(P1);
            all_waypoints.push_back(P2);
            all_waypoints.push_back(P3);
            all_waypoints_with_arm_activation_info_.push_back(make_pair(P1, false));
            all_waypoints_with_arm_activation_info_.push_back(make_pair(P2, false));
            all_waypoints_with_arm_activation_info_.push_back(make_pair(P3, true));
        }
        else if (std::fabs(angle - 90.0) < ANGLE_TOLERANCE)
        {
            // we'll name them A(start), C(end). We want to find B & D
            // For simplicity, let's say:
            //   A = (xA,yA), C = (xC,yC)
            //   B = (xA,yC), D = (xC,yA)
            // Then we pick the one far from box or whichever logic is needed.
            double xA = tstart.x();
            double yA = tstart.y();
            double xC = tend.x();
            double yC = tend.y();

            Eigen::Vector2d B2d(xA, yC); // B
            Eigen::Vector2d D2d(xC, yA); // D
            Eigen::Vector2d box2d(box.x, box.y);
            double dist_from_B = (B2d - box2d).norm();
            double dist_from_D = (D2d - box2d).norm();

            Eigen::Vector2d P2d = (dist_from_B > dist_from_D) ? B2d : D2d;
            Eigen::Vector3d P0(P2d.x(), P2d.y(), tstart.z());
            // (4c) x1 cap => unit vector from start to P
            Eigen::Vector3d lineAP = (P0 - tstart);
            Eigen::Vector3d x1 = lineAP.normalized();

            Eigen::Vector3d linePB = (tend - P0);
            Eigen::Vector3d x2 = linePB.normalized();

            Eigen::Matrix3d R1 = BuildRotationFromX(x1);
            Eigen::Matrix3d R2 = BuildRotationFromX(x2);

            geometry_msgs::msg::PoseStamped P1 = MakePoseStamped(tstart, R1);
            geometry_msgs::msg::PoseStamped P2 = MakePoseStamped(P0, R1);
            geometry_msgs::msg::PoseStamped P3 = MakePoseStamped(P0, R2);
            geometry_msgs::msg::PoseStamped P4 = MakePoseStamped(tend, R2);
            geometry_msgs::msg::PoseStamped P5 = MakePoseStamped(tend, Rend);

            all_waypoints.push_back(P1);
            all_waypoints.push_back(P2);
            all_waypoints.push_back(P3);
            all_waypoints.push_back(P4);
            all_waypoints.push_back(P5);
            all_waypoints_with_arm_activation_info_.push_back(make_pair(P1, false));
            all_waypoints_with_arm_activation_info_.push_back(make_pair(P2, false));
            all_waypoints_with_arm_activation_info_.push_back(make_pair(P3, false));
            all_waypoints_with_arm_activation_info_.push_back(make_pair(P4, false));
            all_waypoints_with_arm_activation_info_.push_back(make_pair(P5, true));
        }
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
            Eigen::Matrix3d goal_rot = eigen_rot * CommonMathsSolver::OrientationNTransformaton::Compute_Rz(tz);
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
            geometry_msgs::msg::Pose G = ComputeNextGoal(box_poses_.poses[i].position, 0.3, current_robot_pose.pose.position);
            GenerateIntermediateWaypoint_BetweenCurrentAndGoal(current_robot_pose.pose, G, box_poses_.poses[i].position);
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
        goal_index_tracker_.current_amr_pose_index++;
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

    geometry_msgs::msg::Pose Manager::ComputeNextGoal(const geometry_msgs::msg::Point &box, double delta,
                                                      const geometry_msgs::msg::Point &current_point)
    {
        geometry_msgs::msg::Pose next_goal;
        geometry_msgs::msg::PoseStamped robot_current_pose_stamped = getCurrentRobotPose();
        auto robot_current_rot_mat = (Conversions::Pose_2Eigen(robot_current_pose_stamped.pose)).block<3, 3>(0, 0);
        Eigen::Vector3d current_xcap = robot_current_rot_mat.col(0);
        Eigen::Vector3d direction_of_motion((box.x - current_point.x), (box.y - current_point.y), 0.0), delta_dir(0.0, 0.0, 0.0);
        direction_of_motion.normalize();

        const geometry_msgs::msg::Point &A = table_vertices[0];
        const geometry_msgs::msg::Point &B = table_vertices[1];
        const geometry_msgs::msg::Point &C = table_vertices[2];
        Eigen::Vector3d ab_cap((B.x - A.x), (B.y - A.y), (B.z - A.z));
        double current_angle_with_ab = acos(current_xcap.dot(ab_cap) / (current_xcap.norm() * ab_cap.norm()));
        // Compute distances
        double distance_from_AB = CommonMathsSolver::Geometry::ComputePerpendicularDistance(box, A, B);
        double distance_from_BC = CommonMathsSolver::Geometry::ComputePerpendicularDistance(box, B, C);
        std::cout << "distance_from_AB " << distance_from_AB << std::endl;
        std::cout << "distance_from_BC " << distance_from_BC << std::endl;

        // Determine desired orientation
        bool line_AB_true = distance_from_AB <= distance_from_BC;
        Eigen::Matrix3d desired_R = (distance_from_AB <= distance_from_BC) ? Rab : Rbc;
        std::cout << desired_R << std::endl;
        next_goal.orientation = Conversions::EigenM_2ROSQuat(desired_R);
        if (line_AB_true)
        {
            double bot_from_BC = CommonMathsSolver::Geometry::ComputePerpendicularDistance(robot_current_pose_stamped.pose.position, B, C);
            if (fabs(current_angle_with_ab - 1.57) < 0.1)
                delta_dir.y() = direction_of_motion.y() / fabs(direction_of_motion.y());
            else
            {
                ////yz known
                next_goal.position.x = (A.x + B.x) / 2.0 - bot_from_BC - 0.1;
                next_goal.position.y = box.y + delta;
                next_goal.position.z = robot_current_pose_stamped.pose.position.z;
                return next_goal;
            }
        }
        else // has to face bc
        {
            double bot_from_AB = CommonMathsSolver::Geometry::ComputePerpendicularDistance(robot_current_pose_stamped.pose.position, A, B);
            if (fabs(current_angle_with_ab - 0.0) < 0.1 || fabs(current_angle_with_ab - 3.14) < 0.1) // bot facing bc
                delta_dir.x() = direction_of_motion.x() / fabs(direction_of_motion.x());
            else // bot facing ab
            {
                // xz known
                next_goal.position.x = box.x + delta;
                next_goal.position.y = (B.y + C.y) / 2.0 - bot_from_AB - 0.1;
                next_goal.position.z = robot_current_pose_stamped.pose.position.z;
                return next_goal;
            }
        }
        std::cout << "x (-1,0,1) " << delta_dir.x() << std::endl;
        std::cout << "y (-1,0,1) " << delta_dir.y() << std::endl;

        // Compute P and Q
        auto computePQ = [](const geometry_msgs::msg::Point &current_point,
                            const geometry_msgs::msg::Point &box,
                            geometry_msgs::msg::Point &P,
                            geometry_msgs::msg::Point &Q)
        {
            // Assuming axis-aligned rectangle
            P.x = current_point.x;
            P.y = box.y;
            P.z = current_point.z;

            Q.x = box.x;
            Q.y = current_point.y;
            Q.z = current_point.z;
        };
        geometry_msgs::msg::Point P, Q;
        computePQ(current_point, box, P, Q);
        bool P_outside = CommonMathsSolver::Geometry::isPointOutsideRectangle(P, table_vertices);

        if (P_outside)
        {
            next_goal.position.x = P.x + (delta * delta_dir.x());
            next_goal.position.y = P.y + (delta * delta_dir.y());
            next_goal.position.z = P.z;
        }
        else
        {
            next_goal.position.x = Q.x + (delta * delta_dir.x());
            next_goal.position.y = Q.y + (delta * delta_dir.y());
            next_goal.position.z = Q.z;
        }
        return next_goal;
    }
}
