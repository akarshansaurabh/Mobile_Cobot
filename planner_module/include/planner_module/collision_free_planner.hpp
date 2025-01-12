#ifndef COLLISION_FREE_PLANNER_HPP
#define COLLISION_FREE_PLANNER_HPP

#include <memory>
#include <string>
#include <array>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// OctoMap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// fcl
#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <Eigen/Dense>

#include "planner_module/kinematics.hpp"
#include "miscellaneous/conversion.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// currently this class is incomplete. Need to add FCL wrapper classes in future

namespace collision_free_planning
{
    class CollisionFreePlanner
    {
    public:
        explicit CollisionFreePlanner(const std::shared_ptr<rclcpp::Node> &node,
                                      const std::shared_ptr<octomap::OcTree> &octree,
                                      const std::shared_ptr<cMRKinematics::ArmKinematicsSolver> &kinematics_solver,
                                      const geometry_msgs::msg::TransformStamped &map_to_base_transform,
                                      const std::vector<std::shared_ptr<fcl::CollisionObjectf>> &link_collision_objects);
        ~CollisionFreePlanner() = default;
        std::vector<std::vector<double>> planPath(const geometry_msgs::msg::Pose &box_top_face_pose);

    private:
        static constexpr int DIM_ = 7;

        std::shared_ptr<rclcpp::Node> node_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string map_frame_;
        std::string end_effector_link_;

        std::shared_ptr<octomap::OcTree> octree_;
        bool treat_unknown_as_occupied_;

        std::shared_ptr<fcl::CollisionObjectf> environment_collision_;
        std::vector<std::shared_ptr<fcl::CollisionObjectf>> link_collision_objects_;
        // std::shared_ptr<fcl::BroadPhaseCollisionManagerf> manager_;

        geometry_msgs::msg::TransformStamped map_to_base_transform_;

        // composition
        std::shared_ptr<cMRKinematics::ArmKinematicsSolver> kinematics_solver_;

    private:
        geometry_msgs::msg::Pose getCurrentEndEffectorPose() const;
        std::vector<ob::ScopedState<ob::RealVectorStateSpace>> PlanInJointSpace(const KDL::JntArray &start,
                                                                                const KDL::JntArray &goal);
        std::vector<std::vector<double>> StatesToPath(const std::vector<ob::ScopedState<ob::RealVectorStateSpace>> &states) const;
        bool isStateValid(const ob::State *state) const;
        bool updateLinkCollisionTransforms(const ob::State *state) const;
        // bool broadPhaseCheckCollision() const;
        std::vector<Eigen::Quaternionf> slerpOrientations(const Eigen::Quaternionf &start_q,
                                                          const Eigen::Quaternionf &goal_q,
                                                          int n) const;
    };
}

#endif
