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
#include "custom_interfaces/msg/table_vertices.hpp"

using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace octoMapGenerator
{
    extern pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_;
    extern std::vector<geometry_msgs::msg::Point> table_vertices_;

    class PointCloudStitcher
    {
    public:
        PointCloudStitcher(rclcpp::Node::SharedPtr node, const std::string &map_frame, std::mutex &m,
                           std::condition_variable &cv, bool &callback_triggered_flag, bool stitch,
                           rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub___);
        void SetVertices(const std::vector<geometry_msgs::msg::Point> table_vertices__);

    private:
        rclcpp::Node::SharedPtr node_;

        float voxel_grid_leaf_size_;
        int mean_k_;
        double std_dev_mul_thresh_;

        std::string map_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped map_to_camera_transform;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Subscription<custom_interfaces::msg::TableVertices>::SharedPtr table_vertices_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub_;
        rclcpp::Client<custom_interfaces::srv::BoxposeEstimator>::SharedPtr box6dposes_client_;

        void SendRequestFor6DPoseEstimation(const std::vector<geometry_msgs::msg::Point> &table_vertices,
                                            double delta, const sensor_msgs::msg::PointCloud2 &cloud);
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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getStitchedCloud();
    };
}

#endif