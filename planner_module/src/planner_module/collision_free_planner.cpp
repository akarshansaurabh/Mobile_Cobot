#include "planner_module/collision_free_planner.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

namespace collision_free_planning
{
    // struct MyCollisionCallback : public fcl::BroadPhaseCollisionManagerf::CollisionCallBack
    // {
    //     bool *collisionFound; // we store a pointer to indicate if collision was found
    //     MyCollisionCallback(bool *found) : collisionFound(found) {}

    //     bool collisionCallback(fcl::CollisionObjectf *obj1,
    //                            fcl::CollisionObjectf *obj2) override
    //     {
    //         fcl::CollisionRequestf request;
    //         request.num_max_contacts = 1;
    //         request.enable_contact = false;
    //         request.enable_cost = false;
    //         fcl::CollisionResultf result;

    //         fcl::collide(obj1, obj2, request, result);
    //         if (result.isCollision())
    //         {
    //             *collisionFound = true;
    //             return true;
    //         }
    //         return false; // false => continue checking
    //     }
    // };

    CollisionFreePlanner::CollisionFreePlanner(const std::shared_ptr<rclcpp::Node> &node,
                                               const std::shared_ptr<octomap::OcTree> &octree,
                                               const std::shared_ptr<cMRKinematics::ArmKinematicsSolver<7>> &kinematics_solver,
                                               const geometry_msgs::msg::TransformStamped &map_to_base_transform,
                                               const std::vector<std::shared_ptr<fcl::CollisionObjectf>> &link_collision_objects)
        : node_(node), octree_(octree), kinematics_solver_(kinematics_solver), map_to_base_transform_(map_to_base_transform),
          link_collision_objects_(link_collision_objects)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        map_frame_ = "map";
        end_effector_link_ = "ee_link";

        std::shared_ptr<fcl::OcTreef> fcl_octree = std::make_shared<fcl::OcTreef>(octree_);
        environment_collision_ = std::make_shared<fcl::CollisionObjectf>(fcl_octree);

        // manager_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
        // manager_->registerObject(environment_collision_.get());
        // for (auto &link_obj : link_collision_objects_)
        //     manager_->registerObject(link_obj.get());
        // manager_->setup();
        // RCLCPP_INFO(node_->get_logger(),
        //             "[CollisionFreePlanner] FCL broad-phase manager created with %zu link objs + 1 environment.",
        //             link_collision_objects_.size());
    }

    std::vector<std::vector<double>> CollisionFreePlanner::planPath(const geometry_msgs::msg::Pose &box_top_face_pose)
    {
        std::vector<std::vector<double>> full_path;
        if (!octree_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "No OctoMap available! Call buildOctomap() before planning.");
            return full_path;
        }
        auto start = std::chrono::high_resolution_clock::now();
        // Compute Grasp and Pre Grasp Pose in Base
        Eigen::Matrix4d Tmapbase = Conversions::TransformStamped_2Eigen(map_to_base_transform_);
        Eigen::Matrix4d Tmapbox = Conversions::Pose_2Eigen(box_top_face_pose);
        Tmapbox(2, 3) += 0.015;
        Eigen::Matrix4d Tbasebox_grasp_pose = Tmapbase.inverse() * Tmapbox;
        Eigen::Vector3d z_cap = -Tbasebox_grasp_pose.block<3, 1>(0, 2);
        Eigen::Vector3d y_cap = Tbasebox_grasp_pose.block<3, 1>(0, 1);
        Eigen::Vector3d x_cap = y_cap.cross(z_cap);
        x_cap.normalize();
        Tbasebox_grasp_pose.block<3, 1>(0, 0) = x_cap;
        Tbasebox_grasp_pose.block<3, 1>(0, 1) = y_cap;
        Tbasebox_grasp_pose.block<3, 1>(0, 2) = z_cap;
        Eigen::Matrix4d pre_grasp_pose = Tbasebox_grasp_pose;
        pre_grasp_pose(2, 3) += 0.10;

        // IK for pre grasp pose
        KDL::JntArray start_positions(7), pre_grasp_positions(7), grasp_positions(7);
        start_positions(0) = cMRKinematics::state_info_.joint_states[0];
        start_positions(1) = cMRKinematics::state_info_.joint_states[1];
        start_positions(2) = cMRKinematics::state_info_.joint_states[2];
        start_positions(3) = cMRKinematics::state_info_.joint_states[3];
        start_positions(4) = cMRKinematics::state_info_.joint_states[4];
        start_positions(5) = cMRKinematics::state_info_.joint_states[5];
        start_positions(6) = cMRKinematics::state_info_.joint_states[6];
        if (!kinematics_solver_->SolveIK(Conversions::Transform_2KDL(pre_grasp_pose)))
        {
            std::cout << "[pre_grasp_pose] IK NOT FOUND" << std::endl;
            return full_path;
        }
        pre_grasp_positions = kinematics_solver_->q;

        //****************************************************************************
        std::vector<KDL::Frame> all_link_poses_7dof;
        kinematics_solver_->SolveFKAllLinks(pre_grasp_positions, all_link_poses_7dof);
        auto T_base_slider = Conversions::KDL_2Transform(all_link_poses_7dof[1]);

        const int num_samples = 10;
        double lower_prismatic = -0.1;
        double upper_prismatic = 0.5;
        double step = (upper_prismatic - lower_prismatic) / (num_samples - 1);

        bool found_collision_free = false, ompl_successful = false;
        std::unique_ptr<cMRKinematics::ArmKinematicsSolver<6>> kinematics_solver_6dof_;
        std::string urdf_param = "robot_description";
        std::string root_link = "arm_base_link_1";
        std::string tip_link = "ee_link";
        kinematics_solver_6dof_ = std::make_unique<cMRKinematics::ArmKinematicsSolver<6>>(node_, urdf_param,
                                                                                          root_link, tip_link, false);
        kinematics_solver_6dof_->initial_guess(0) = pre_grasp_positions(1);
        kinematics_solver_6dof_->initial_guess(1) = pre_grasp_positions(2);
        kinematics_solver_6dof_->initial_guess(2) = pre_grasp_positions(3);
        kinematics_solver_6dof_->initial_guess(3) = pre_grasp_positions(4);
        kinematics_solver_6dof_->initial_guess(4) = pre_grasp_positions(5);
        kinematics_solver_6dof_->initial_guess(5) = pre_grasp_positions(6);
        auto space = std::make_shared<ob::RealVectorStateSpace>(7);
        ob::RealVectorBounds bounds(7);
        bounds.setLow(0, -0.1);
        bounds.setHigh(0, 0.5);
        for (int i = 1; i < 7; i++)
        {
            bounds.setLow(i, -3.14);
            bounds.setHigh(i, 3.14);
        }
        space->setBounds(bounds);

        int max_itr = -1, max_itr2 = -1;
        std::vector<ob::ScopedState<ob::RealVectorStateSpace>> start_to_pregrasp, pregrasp_to_grasp;

        while (!found_collision_free && max_itr < 10)
        {
            max_itr++;
            std::cout << "max_itr " << max_itr << std::endl;
            for (int i = 0; i < num_samples; ++i)
            {
                double d1 = lower_prismatic + i * step;
                T_base_slider(2, 3) = 1.1 - d1;
                auto T_slider_box = T_base_slider.inverse() * Tbasebox_grasp_pose;
                if (!kinematics_solver_6dof_->SolveIK(Conversions::Transform_2KDL(T_slider_box)))
                    continue;

                KDL::JntArray candidate_7dof(7);
                ob::ScopedState<ob::RealVectorStateSpace> st(space);
                st->values[0] = candidate_7dof(0) = d1;
                for (int j = 0; j < 6; j++)
                    st->values[j + 1] = candidate_7dof(j + 1) = kinematics_solver_6dof_->q(j);

                if (isStateValid(st.get()))
                {
                    found_collision_free = true;
                    for (int idx = 0; idx < 7; idx++)
                        grasp_positions(idx) = candidate_7dof(idx);
                    break;
                }
            }
        }
        if (!found_collision_free)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "[planPath] No collision-free IK found among prismatic samples.");
            return full_path;
        }
        //*************************************************************************
        std::vector<std::future<std::vector<ob::ScopedState<ob::RealVectorStateSpace>>>> futures;
        futures.reserve(2);
        auto lambda_ = [this](const KDL::JntArray &first_state, const KDL::JntArray &second_state)
            -> std::vector<ob::ScopedState<ob::RealVectorStateSpace>>
        {
            return PlanInJointSpace(first_state, second_state);
        };
        futures.emplace_back(std::async(std::launch::async, lambda_, start_positions, pre_grasp_positions));
        futures.emplace_back(std::async(std::launch::async, lambda_, pre_grasp_positions, grasp_positions));
        for (int i = 0; i < 2; i++)
        {
            if (i == 0)
                start_to_pregrasp = futures[i].get();
            else
                pregrasp_to_grasp = futures[i].get();
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
        if (start_to_pregrasp.size() < 2 || pregrasp_to_grasp.size() < 2)
        {
            std::cout << "ompl Planning failed" << std::endl;
            return full_path;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        auto start_to_pregrasp_vector2D = StatesToPath(start_to_pregrasp);
        auto pregrasp_to_grasp_vector2D = StatesToPath(pregrasp_to_grasp);
        full_path.insert(full_path.end(), start_to_pregrasp_vector2D.begin(), start_to_pregrasp_vector2D.end());
        full_path.insert(full_path.end(), pregrasp_to_grasp_vector2D.begin(), pregrasp_to_grasp_vector2D.end());

        RCLCPP_INFO(node_->get_logger(),
                    "[CollisionFreePlanner] Path has %zu waypoints (first seg=%d, second seg=%d)",
                    full_path.size(), start_to_pregrasp_vector2D.size(), pregrasp_to_grasp_vector2D.size());

        return full_path;
    }

    geometry_msgs::msg::Pose CollisionFreePlanner::getCurrentEndEffectorPose() const
    {
        geometry_msgs::msg::Pose ee_pose;

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

    std::vector<ob::ScopedState<ob::RealVectorStateSpace>> CollisionFreePlanner::PlanInJointSpace(const KDL::JntArray &start,
                                                                                                  const KDL::JntArray &goal)
    {
        std::vector<ob::ScopedState<ob::RealVectorStateSpace>> solution;

        if (!octree_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[CollisionFreePlanner] No OctoMap for plan3D. Aborting.");
            return solution;
        }

        // 1) Create the RealVectorStateSpace
        auto space = std::make_shared<ob::RealVectorStateSpace>(7);
        ob::RealVectorBounds bounds(7);
        bounds.setLow(0, -0.7);
        bounds.setHigh(0, 0.5);
        for (int i = 1; i < 7; i++)
        {
            if (i == 2 || i == 3)
            {
                bounds.setLow(i, -2.5);
                bounds.setHigh(i, 2.5);
            }
            else
            {
                bounds.setLow(i, -3.14);
                bounds.setHigh(i, 3.14);
            }
        }
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
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = start(0);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = start(1);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = start(2);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = start(3);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = start(4);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = start(5);
        start_state->as<ob::RealVectorStateSpace::StateType>()->values[6] = start(6);

        ob::ScopedState<> goal_state(space);
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal(0);
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal(1);
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal(2);
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[3] = goal(3);
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[4] = goal(4);
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[5] = goal(5);
        goal_state->as<ob::RealVectorStateSpace::StateType>()->values[6] = goal(6);

        ss.setStartAndGoalStates(start_state, goal_state);

        // 5) Planner
        ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

        // 6) Solve
        ob::PlannerStatus solved = ss.solve(5.0); // 2-second time budget

        if (solved)
        {
            ss.simplifySolution();

            auto path_geometric = ss.getSolutionPath().as<og::PathGeometric>();
            for (std::size_t i = 0; i < path_geometric->getStateCount(); i++)
            {
                ob::ScopedState<ob::RealVectorStateSpace> s(space);
                s = path_geometric->getState(i);
                std::cout << "State " << i << " : ";
                for (std::size_t j = 0; j < space->getDimension(); j++)
                {
                    double val = s->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];
                    std::cout << val << " ";
                }
                std::cout << std::endl;
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
            solution.clear();
        }
        return solution;
    }

    std::vector<std::vector<double>> CollisionFreePlanner::StatesToPath(
        const std::vector<ob::ScopedState<ob::RealVectorStateSpace>> &states) const
    {
        std::vector<std::vector<double>> vector_of_jointstates;
        vector_of_jointstates.reserve(states.size());

        // Create a 7-element vector:
        std::vector<double> jointstates(7);

        for (auto &st : states)
        {
            jointstates[0] = st->as<ob::RealVectorStateSpace::StateType>()->values[0];
            jointstates[1] = st->as<ob::RealVectorStateSpace::StateType>()->values[1];
            jointstates[2] = st->as<ob::RealVectorStateSpace::StateType>()->values[2];
            jointstates[3] = st->as<ob::RealVectorStateSpace::StateType>()->values[3];
            jointstates[4] = st->as<ob::RealVectorStateSpace::StateType>()->values[4];
            jointstates[5] = st->as<ob::RealVectorStateSpace::StateType>()->values[5];
            jointstates[6] = st->as<ob::RealVectorStateSpace::StateType>()->values[6];

            // Push the 7-element vector into the 2D vector
            vector_of_jointstates.push_back(jointstates);
        }

        return vector_of_jointstates;
    }

    bool CollisionFreePlanner::isStateValid(const ob::State *state) const
    {
        // 1) Update link transforms according to the joint configuration
        if (!updateLinkCollisionTransforms(state))
            return false;

        // 2) Prepare a FCL collision request
        fcl::CollisionRequestf request;
        request.enable_contact = false; // We only need boolean collision check
        request.num_max_contacts = 1;   // Stop after one contact
        request.enable_cost = false;    // We skip cost computations

        fcl::CollisionResultf result;

        // 3) Check link vs. environment
        if (environment_collision_)
            for (int i = 2; i < 9; i++)
            {
                fcl::collide(link_collision_objects_[i].get(), environment_collision_.get(), request, result);
                if (result.isCollision())
                {
                    std::cout << "collision is there" << std::endl;
                    return false;
                }
            }

        // for (int i = 3; i < 8; i++)
        //     for (int j = i + 1; j < 9; j++)
        //     {
        fcl::collide(link_collision_objects_[5].get(), link_collision_objects_[7].get(), request, result);
        if (result.isCollision())
        {
            std::cout << "self collision is there" << std::endl;
            return false;
        }
        //     }
        return true;

        // manager_->update();
        // return !broadPhaseCheckCollision();
    }

    // bool CollisionFreePlanner::broadPhaseCheckCollision() const
    // {
    //     bool collisionFound = false;
    //     MyCollisionCallback callback(&collisionFound);
    //     manager_->collide(&callback);
    //     return collisionFound;
    // }

    bool CollisionFreePlanner::updateLinkCollisionTransforms(const ob::State *state) const
    {
        // Convert RealVectorStateSpace -> KDL::JntArray
        KDL::JntArray joint_positions(7);
        const auto *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
        for (int i = 0; i < 7; i++)
            joint_positions(i) = values[i];

        // Solve forward kinematics
        std::vector<KDL::Frame> all_link_poses;
        kinematics_solver_->SolveFKAllLinks(joint_positions, all_link_poses);

        Eigen::Matrix4d map_to_base = Eigen::Matrix4d::Identity();
        map_to_base(0, 3) = map_to_base_transform_.transform.translation.x;
        map_to_base(1, 3) = map_to_base_transform_.transform.translation.y;
        map_to_base(2, 3) = map_to_base_transform_.transform.translation.z;
        Eigen::Quaterniond q(map_to_base_transform_.transform.rotation.w, map_to_base_transform_.transform.rotation.x,
                             map_to_base_transform_.transform.rotation.y, map_to_base_transform_.transform.rotation.z);
        q.normalize();
        map_to_base.block<3, 3>(0, 0) = q.toRotationMatrix();

        for (int i = 0; i <= 7; ++i)
        {
            Eigen::Matrix4d map_to_link = map_to_base * Conversions::KDL_2Transform(all_link_poses[i]);
            link_collision_objects_[i + 1]->setTranslation(fcl::Vector3f(map_to_link(0, 3), map_to_link(1, 3), map_to_link(2, 3)));
            link_collision_objects_[i + 1]->setRotation(map_to_link.block<3, 3>(0, 0).cast<float>());
        }

        return true;
    }

    std::vector<Eigen::Quaternionf> CollisionFreePlanner::slerpOrientations(const Eigen::Quaternionf &start_q,
                                                                            const Eigen::Quaternionf &goal_q,
                                                                            int n) const
    {
        std::vector<Eigen::Quaternionf> quaternions;
        quaternions.reserve(n);

        if (n <= 0)
        {
            quaternions.clear();
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