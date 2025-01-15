#ifndef OCTOMAP_GENERATOR_HPP
#define OCTOMAP_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/common/transforms.h>

#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <condition_variable>

#include "custom_interfaces/srv/boxpose_estimator.hpp"
#include "custom_interfaces/srv/goal_pose_vector.hpp"

// OctoMap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// FCL
#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>

#include "visualizations/visualization_manager.hpp"
#include "planner_module/collision_free_planner.hpp"
#include "planner_module/kinematics.hpp"
#include "miscellaneous/conversion.hpp"

using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace octoMapGenerator
{
    extern pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
    extern geometry_msgs::msg::TransformStamped map_to_base_transform;

    class PointCloudStitcher
    {
    public:
        PointCloudStitcher(rclcpp::Node::SharedPtr node, const std::string &map_frame, std::mutex &m,
                           std::condition_variable &cv, bool &callback_triggered_flag, bool stitch,
                           rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub___);

    private:
        rclcpp::Node::SharedPtr node_;

        float voxel_grid_leaf_size_;
        int mean_k_;
        double std_dev_mul_thresh_;
        bool process_callback;

        std::string map_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped map_to_camera_transform;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub_;
        rclcpp::Client<custom_interfaces::srv::BoxposeEstimator>::SharedPtr box6dposes_client_;

        void SendRequestFor6DPoseEstimation(double delta, const sensor_msgs::msg::PointCloud2 &cloud);
        void HandleResponse(rclcpp::Client<custom_interfaces::srv::BoxposeEstimator>::SharedFuture future);

        bool start_stiching;
        static Eigen::Affine3d poseToEigen(const geometry_msgs::msg::Transform &transform);
        void pointCloudSnapShotCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &snapshot_msg);

        // Synchronization variables
        std::mutex &mutex_;
        std::condition_variable &cv_;
        bool &callback_triggered_;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformCloudToMap(const sensor_msgs::msg::PointCloud2 &msg);
        void addCloudSnapshot(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &snapshot);
        void finalizeStitching(float leaf_size = 0.015f);
    };

    class OctoMapGenerator
    {
    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<octomap::OcTree> octree_;
        rclcpp::Service<custom_interfaces::srv::GoalPoseVector>::SharedPtr colision_free_planner_server_;
        std::vector<std::shared_ptr<fcl::CollisionObjectf>> link_collision_objects_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<cMRKinematics::ArmKinematicsSolver<7>> kinematics_solver_;

        void ServerCallbackForColisionFreePlanning(const std::shared_ptr<custom_interfaces::srv::GoalPoseVector::Request> request,
                                                   std::shared_ptr<custom_interfaces::srv::GoalPoseVector::Response> response);

    public:
        OctoMapGenerator(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<cMRKinematics::ArmKinematicsSolver<7>> &kinematics_solver,
                         const std::vector<std::shared_ptr<fcl::CollisionObjectf>> &link_collision_objects);
        void buildOctomap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, double resolution = 0.02);
    };
}

#endif