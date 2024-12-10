#ifndef OCTOMAP_GENERATOR_HPP
#define OCTOMAP_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>

/*
   This code uses method 2 (direct tf2 lookup) to transform camera frame pointclouds into map frame.
   We assume all frames are published to TF: map->...->camera.
   Steps:
   1) For each snapshot at time t, we have a point cloud in camera_frame.
   2) Use tf_buffer_->lookupTransform("map", camera_frame, t) to get map->camera.
   3) Transform the cloud and accumulate.
   4) After all snapshots (c1, c2, c3), apply voxel grid filter.

   Thread-safety:
   - A mutex protects accumulated_cloud_.
*/
namespace octoMapGenerator
{

    class PointCloudStitcherMethod2
    {
    public:
        PointCloudStitcherMethod2(rclcpp::Node::SharedPtr node, const std::string &map_frame);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformCloudToMap(const sensor_msgs::msg::PointCloud2 &msg);
        void addCloudSnapshot(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &snapshot);
        void finalizeStitching(float leaf_size = 0.015f);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getStitchedCloud();

    private:
        rclcpp::Node::SharedPtr node_;

        std::string map_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped map_to_camera_transform;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

        std::mutex cloud_mutex_;

        static Eigen::Affine3d poseToEigen(const geometry_msgs::msg::Transform &transform);
        void pointCloudSnapShotCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &snapshot_msg);
    };
}

#endif