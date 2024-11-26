// pointcloud_transformer.hpp

#ifndef POINTCLOUD_TRANSFORMER_HPP
#define POINTCLOUD_TRANSFORMER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_eigen/tf2_eigen.hpp> // For tf2::fromMsg
#include <pcl_ros/transforms.hpp>  // For pcl::transformPointCloud
#include <cmath>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>

// For timing
#include <chrono>

namespace pointcloud_transformer
{

    class PointCloudTransformer : public rclcpp::Node
    {
    public:
        PointCloudTransformer();
        ~PointCloudTransformer() = default;

    private:
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr raw_input_cloud_msg);
        // Helper function to filter points within a cylinder
        sensor_msgs::msg::PointCloud2 FilterPointsInCylinder(
            const sensor_msgs::msg::PointCloud2 &cloud,
            float radius, float min_z, float max_z);
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr FilterPointsInCylinder(const sensor_msgs::msg::PointCloud2 &cloud,
        //                                                               float radius, float min_z, float max_z);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::string target_frame_;

        // Downsampling Parameters
        float voxel_grid_leaf_size_;
        // Outlier Removal Parameters
        int mean_k_;
        double std_dev_mul_thresh_;
        // Cylinder Filtering Parameters
        float cylinder_radius_;
        float cylinder_min_z_;
        float cylinder_max_z_;
    };
}

#endif
