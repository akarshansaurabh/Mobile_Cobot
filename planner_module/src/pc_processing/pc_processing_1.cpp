#include "pc_processing/pc_processing_1.hpp"

namespace pointcloud_transformer
{
    PointCloudTransformer::PointCloudTransformer()
        : Node("pc_processing_1_node"),
          target_frame_("map"),
          voxel_grid_leaf_size_(0.035f), // Default voxel grid leaf size (5cm)
          mean_k_(50),                   // Default mean K for outlier removal
          std_dev_mul_thresh_(1.0)       // Default std dev multiplier threshold
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);  // tracks transform at current time

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/arm_rgbd_camera/points", 10,
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for the transformed point cloud
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/arm_rgbd_camera/points_map", 10);
    }

    void PointCloudTransformer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Convert ROS PointCloud2 message to PCL PointCloud with RGB
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *raw_pcl_cloud);

        // Downsample the point cloud using VoxelGrid filter
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(raw_pcl_cloud);
        voxel_filter.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
        voxel_filter.filter(*downsampled_pcl_cloud);

        // Outlier Removal using StatisticalOutlierRemoval
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_downsampled_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(downsampled_pcl_cloud);
        sor.setMeanK(mean_k_);
        sor.setStddevMulThresh(std_dev_mul_thresh_);
        sor.filter(*filtered_downsampled_pcl_cloud);

        // Convert filtered PCL PointCloud back to ROS PointCloud2
        sensor_msgs::msg::PointCloud2 downsampled_filtered_msg;
        pcl::toROSMsg(*filtered_downsampled_pcl_cloud, downsampled_filtered_msg);
        downsampled_filtered_msg.header = msg->header; // Preserve the original header

        sensor_msgs::msg::PointCloud2 transformed_cloud;

        try
        {
            // Wait for the transform to become available
            if (!tf_buffer_->canTransform(target_frame_, downsampled_filtered_msg.header.frame_id,
                                          downsampled_filtered_msg.header.stamp, rclcpp::Duration::from_seconds(1.0)))
            {
                RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s at time %d.%d",
                            downsampled_filtered_msg.header.frame_id.c_str(), target_frame_.c_str(),
                            downsampled_filtered_msg.header.stamp.sec, downsampled_filtered_msg.header.stamp.nanosec);
                return;
            }

            // Perform the transformation
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(target_frame_, downsampled_filtered_msg.header.frame_id,
                                                                                         downsampled_filtered_msg.header.stamp);
            tf2::doTransform(downsampled_filtered_msg, transformed_cloud, transform);
            transformed_cloud.header.frame_id = target_frame_;

            // Publish the transformed point cloud
            pointcloud_pub_->publish(transformed_cloud);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> processing_time = end_time - start_time;

        // Log the processing time
        RCLCPP_INFO(this->get_logger(),
                    "Processed point cloud: downsampled from %zu to %zu points, filtered to %zu points. Processing took %.2f ms.",
                    raw_pcl_cloud->points.size(),
                    downsampled_pcl_cloud->points.size(),
                    filtered_downsampled_pcl_cloud->points.size(),
                    processing_time.count());
    }
}
