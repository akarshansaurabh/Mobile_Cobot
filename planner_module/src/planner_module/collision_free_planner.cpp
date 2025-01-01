#include "planner_module/collision_free_planner.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

namespace collision_free_planning
{
    CollisionFreePlanner::CollisionFreePlanner(const std::shared_ptr<rclcpp::Node> &node,
                                               const std::shared_ptr<octomap::OcTree> &octree)
        : node_(node), octree_(octree)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_frame_ = "map";
        end_effector_link_ = "wrist3_Link_1";

        // We'll introduce a parameter for unknown cells in octomap
        node_->declare_parameter<bool>("treat_unknown_as_occupied", false);
        node_->get_parameter<bool>("treat_unknown_as_occupied", treat_unknown_as_occupied_);
    }

    // void CollisionFreePlanner::buildOctomap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
    //                                         double resolution)
    // {
    //     // Create an OcTree
    //     octree_ = std::make_shared<octomap::OcTree>(resolution);

    //     // Insert points
    //     for (const auto &pt : pcl_cloud->points)
    //     {
    //         if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
    //         {
    //             octomap::point3d p(pt.x, pt.y, pt.z);
    //             octree_->updateNode(p, true);
    //         }
    //     }
    //     octree_->updateInnerOccupancy();

    //     RCLCPP_INFO(node_->get_logger(),
    //                 "[CollisionFreePlanner] OctoMap built: %zu pts, resolution=%.3f",
    //                 pcl_cloud->size(), resolution);
    // }

    std::vector<geometry_msgs::msg::Pose> CollisionFreePlanner::planPath(const geometry_msgs::msg::Pose &box_top_face_pose)
    {
        std::vector<geometry_msgs::msg::Pose> full_path;
        if (!octree_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "No OctoMap available! Call buildOctomap() before planning.");
            return full_path;
        }

        geometry_msgs::msg::Pose current_ee_pose = getCurrentEndEffectorPose();

        geometry_msgs::msg::Pose pre_grasp_pose = box_top_face_pose;
        pre_grasp_pose.position.z += 0.10;

        std::array<double, 3> start_pos{current_ee_pose.position.x,
                                        current_ee_pose.position.y,
                                        current_ee_pose.position.z};

        std::array<double, 3> pre_grasp_pos{pre_grasp_pose.position.x,
                                            pre_grasp_pose.position.y,
                                            pre_grasp_pose.position.z};
        auto path_to_pre = plan3D(start_pos, pre_grasp_pos);
        auto poses_to_pre = statesToPath(path_to_pre);

        std::array<double, 3> box_pos{box_top_face_pose.position.x,
                                      box_top_face_pose.position.y,
                                      box_top_face_pose.position.z};
        auto path_to_box = plan3D(pre_grasp_pos, box_pos);
        auto poses_to_box = statesToPath(path_to_box);

        Eigen::Quaternionf q_start(current_ee_pose.orientation.w,
                                   current_ee_pose.orientation.x,
                                   current_ee_pose.orientation.y,
                                   current_ee_pose.orientation.z);

        Eigen::Quaternionf q_pre(pre_grasp_pose.orientation.w,
                                 pre_grasp_pose.orientation.x,
                                 pre_grasp_pose.orientation.y,
                                 pre_grasp_pose.orientation.z);

        Eigen::Quaternionf q_box(box_top_face_pose.orientation.w,
                                 box_top_face_pose.orientation.x,
                                 box_top_face_pose.orientation.y,
                                 box_top_face_pose.orientation.z);

        int n1 = static_cast<int>(poses_to_pre.size());
        auto slerp1 = slerpOrientations(q_start, q_pre, n1);
        for (int i = 0; i < n1; i++)
        {
            poses_to_pre[i].orientation.x = slerp1[i].x();
            poses_to_pre[i].orientation.y = slerp1[i].y();
            poses_to_pre[i].orientation.z = slerp1[i].z();
            poses_to_pre[i].orientation.w = slerp1[i].w();
        }

        int n2 = static_cast<int>(poses_to_box.size());
        auto slerp2 = slerpOrientations(q_pre, q_box, n2);
        for (int i = 0; i < n2; i++)
        {
            poses_to_box[i].orientation.x = slerp2[i].x();
            poses_to_box[i].orientation.y = slerp2[i].y();
            poses_to_box[i].orientation.z = slerp2[i].z();
            poses_to_box[i].orientation.w = slerp2[i].w();
        }

        full_path.reserve(n1 + n2);
        full_path.insert(full_path.end(), poses_to_pre.begin(), poses_to_pre.end());
        full_path.insert(full_path.end(), poses_to_box.begin(), poses_to_box.end());

        RCLCPP_INFO(node_->get_logger(),
                    "[CollisionFreePlanner] Path has %zu waypoints (first seg=%d, second seg=%d)",
                    full_path.size(), n1, n2);
        std::cout << "n1 = " << n1 << std::endl;
        std::cout << "n1 = " << n1 << std::endl;
        std::cout << "n1 = " << n1 << std::endl;
        std::cout << "n1 = " << n1 << std::endl;
        std::cout << "n1 = " << n1 << std::endl;
        std::cout << "n1 = " << n1 << std::endl;
        std::cout << "n1 = " << n1 << std::endl;
        std::cout << "n2 = " << n2 << std::endl;
        std::cout << "n2 = " << n2 << std::endl;
        std::cout << "n2 = " << n2 << std::endl;
        std::cout << "n2 = " << n2 << std::endl;
        std::cout << "n2 = " << n2 << std::endl;
        std::cout << "n2 = " << n2 << std::endl;
        std::cout << "n2 = " << n2 << std::endl;
        std::cout << "n2 = " << n2 << std::endl;
        std::cout << "n2 = " << n2 << std::endl;

        return full_path;
    }

    geometry_msgs::msg::Pose CollisionFreePlanner::getCurrentEndEffectorPose() const
    {
        geometry_msgs::msg::Pose ee_pose;
        ee_pose.orientation.w = 1.0;

        if (!node_)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("CollisionFreePlanner"),
                                "No node handle. Unable to do TF lookups.");
            return ee_pose;
        }

        try
        {
            auto transform_stamped = tf_buffer_->lookupTransform(map_frame_, end_effector_link_,
                                                                 tf2::TimePointZero, tf2::durationFromSec(1.0));

            ee_pose.position.x = transform_stamped.transform.translation.x;
            ee_pose.position.y = transform_stamped.transform.translation.y;
            ee_pose.position.z = transform_stamped.transform.translation.z;
            ee_pose.orientation = transform_stamped.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[CollisionFreePlanner] TF for end effector failed: %s",
                        ex.what());
        }
        return ee_pose;
    }

    std::vector<ob::ScopedState<ob::RealVectorStateSpace>> CollisionFreePlanner::plan3D(const std::array<double, 3> &start,
                                                                                        const std::array<double, 3> &goal)
    {
        std::vector<ob::ScopedState<ob::RealVectorStateSpace>> solution;

        if (!octree_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[CollisionFreePlanner] No OctoMap for plan3D. Aborting.");
            return solution;
        }

        // 1) Create the RealVectorStateSpace
        auto space = std::make_shared<ob::RealVectorStateSpace>(DIM_);
        ob::RealVectorBounds bounds(DIM_);
        // Hard-coded bounding box for demonstration
        bounds.setLow(0, -2.0); // x in [-2, 2]
        bounds.setHigh(0, 2.0);
        bounds.setLow(1, -2.0); // y in [-2, 2]
        bounds.setHigh(1, 2.0);
        bounds.setLow(2, 0.0); // z in [0, 2]
        bounds.setHigh(2, 2.0);
        space->setBounds(bounds);

        // 2) Setup SimpleSetup
        og::SimpleSetup ss(space);

        // 3) State validity checker
        auto validityFn = [this](const ob::State *st)
        {
            return this->isStateValid(st);
        };
        ss.setStateValidityChecker(validityFn);

        // 4) Start & goal states
        ob::ScopedState<> start_state(space);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = start[0];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = start[1];
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = start[2];

        ob::ScopedState<> goal_state(space);
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];

        ss.setStartAndGoalStates(start_state, goal_state);

        // 5) Planner
        ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

        // 6) Solve
        ob::PlannerStatus solved = ss.solve(2.0); // 2-second time budget

        if (solved)
        {
            ss.simplifySolution();

            auto path_geometric = ss.getSolutionPath().as<og::PathGeometric>();
            for (std::size_t i = 0; i < path_geometric->getStateCount(); i++)
            {
                ob::ScopedState<> s(space);
                s = path_geometric->getState(i);
                solution.push_back(s);
            }
            RCLCPP_INFO(node_->get_logger(),
                        "[CollisionFreePlanner] Found path with %zu states",
                        path_geometric->getStateCount());
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[CollisionFreePlanner] OMPL could not find a path in plan3D()");
        }
        return solution;
    }

    std::vector<geometry_msgs::msg::Pose> CollisionFreePlanner::statesToPath(const std::vector<ob::ScopedState<ob::RealVectorStateSpace>> &states) const
    {
        std::vector<geometry_msgs::msg::Pose> path;
        path.reserve(states.size());
        geometry_msgs::msg::Pose pose;
        // identity orientation for now
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        for (auto &st : states)
        {
            pose.position.x = st->as<ob::RealVectorStateSpace::StateType>()->values[0];
            pose.position.y = st->as<ob::RealVectorStateSpace::StateType>()->values[1];
            pose.position.z = st->as<ob::RealVectorStateSpace::StateType>()->values[2];

            path.push_back(pose);
        }
        return path;
    }

    bool CollisionFreePlanner::isStateValid(const ob::State *state) const
    {
        if (!octree_)
        {
            // No map => not valid
            return false;
        }

        const auto *reals = state->as<ob::RealVectorStateSpace::StateType>();
        double x = reals->values[0];
        double y = reals->values[1];
        double z = reals->values[2];

        if (x < -2.0 || x > 2.0 || y < -2.0 || y > 2.0 || z < 0.0 || z > 2.0)
            return false;

        // 3) Check occupancy from OctoMap
        octomap::point3d query(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
        octomap::OcTreeNode *node = octree_->search(query);

        if (!node)
        {
            // **Unknown space** encountered
            // "Policy" question: treat unknown as free or occupied?
            //   - If we are conservative, we treat unknown as occupied => return false
            //   - If we are aggressive, we treat unknown as free => return true
            // Here, we do what the parameter says:
            return (!treat_unknown_as_occupied_);
        }
        else
        {
            if (octree_->isNodeOccupied(node))
                return false;
            else
                return true;
        }
    }

    std::vector<Eigen::Quaternionf> CollisionFreePlanner::slerpOrientations(const Eigen::Quaternionf &start_q,
                                                                            const Eigen::Quaternionf &goal_q,
                                                                            int n) const
    {
        std::vector<Eigen::Quaternionf> quaternions;
        quaternions.reserve(n);

        if (n <= 0)
        {
            return quaternions;
        }
        else if (n == 1)
        {
            quaternions.push_back(start_q);
            return quaternions;
        }

        const float dot_val = fabsf(start_q.dot(goal_q));
        const float eps = 1e-6f;
        bool are_same = (dot_val > 1.0f - eps);

        if (are_same)
        {
            for (int i = 0; i < n; i++)
                quaternions.push_back(start_q);
            return quaternions;
        }

        for (int i = 0; i < n; i++)
        {
            float t = static_cast<float>(i) / static_cast<float>(n - 1);
            Eigen::Quaternionf q = start_q.slerp(t, goal_q);
            quaternions.push_back(q);
        }
        return quaternions;
    }
}