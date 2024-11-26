#include "pc_processing/pc_processing_2.hpp"

namespace pointcloud_transformer2
{

    PointCloudTransformer::PointCloudTransformer()
        : Node("pointcloud_transformer_node"),
          target_frame_("map"),
          base_frame_("base_link"),
          voxel_grid_leaf_size_(0.07f), // Smaller leaf size for finer downsampling
          mean_k_(50),                  // Adjusted mean K for outlier removal
          std_dev_mul_thresh_(1.0),     // Tighter threshold for outlier removal
          cylinder_radius_(2.0f),       // Reduced radius for focused region
          cylinder_min_z_(0.0f),
          cylinder_max_z_(2.0f)
    {
        // Initialize TF2 Buffer and Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscriber for the point cloud data
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/arm_rgbd_camera/points", rclcpp::QoS(10),
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for the processed point cloud
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/arm_rgbd_camera/points_map", rclcpp::QoS(10));
    }

    void PointCloudTransformer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        auto start_time = std::chrono::steady_clock::now();

        // Check for empty point cloud
        if (msg->width == 0 || msg->height == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud.");
            return;
        }

        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Check if point cloud conversion was successful
        if (pcl_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Converted PCL point cloud is empty.");
            return;
        }

        // Downsample the point cloud using VoxelGrid filter (in-place operation)
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(pcl_cloud);
        voxel_filter.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
        voxel_filter.filter(*pcl_cloud);

        // Remove outliers using StatisticalOutlierRemoval filter (in-place operation)
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_filter;
        sor_filter.setInputCloud(pcl_cloud);
        sor_filter.setMeanK(mean_k_);
        sor_filter.setStddevMulThresh(std_dev_mul_thresh_);
        sor_filter.filter(*pcl_cloud);

        // Transform point cloud to base_frame_
        Eigen::Matrix4f transform_to_base = Eigen::Matrix4f::Identity();
        try
        {
            geometry_msgs::msg::TransformStamped tf_cam_to_base =
                tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

            // Convert geometry_msgs Transform to Eigen Matrix
            Eigen::Affine3d eigen_transform = tf2::transformToEigen(tf_cam_to_base.transform);
            transform_to_base = eigen_transform.cast<float>().matrix();
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform from %s to %s: %s",
                        msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
            return;
        }

        // Apply the transformation (in-place)
        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, transform_to_base);

        // Filter points within the cylindrical region
        filterPointCloudInCylinder(pcl_cloud);

        // Check if any points remain after filtering
        if (pcl_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points remain after cylinder filtering.");
            return;
        }

        // Transform point cloud to target_frame_ (map frame)
        Eigen::Matrix4f transform_to_map = Eigen::Matrix4f::Identity();
        try
        {
            geometry_msgs::msg::TransformStamped tf_base_to_map =
                tf_buffer_->lookupTransform(target_frame_, base_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

            // Convert geometry_msgs Transform to Eigen Matrix
            Eigen::Affine3d eigen_transform = tf2::transformToEigen(tf_base_to_map.transform);
            transform_to_map = eigen_transform.cast<float>().matrix();
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform from %s to %s: %s",
                        base_frame_.c_str(), target_frame_.c_str(), ex.what());
            return;
        }

        // Apply the transformation (in-place)
        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, transform_to_map);

        // Convert the PCL point cloud back to ROS PointCloud2 message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*pcl_cloud, output_msg);
        output_msg.header.stamp = msg->header.stamp;
        output_msg.header.frame_id = target_frame_;

        // Publish the processed point cloud
        pointcloud_pub_->publish(output_msg);

        auto end_time = std::chrono::steady_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        // Log the processing time
        RCLCPP_INFO(this->get_logger(), "Processed point cloud in %ld ms.", processing_time);
    }

    void PointCloudTransformer::filterPointCloudInCylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
    {
        // Use PCL's CropBox filter to approximate cylinder filtering for efficiency
        pcl::CropBox<pcl::PointXYZRGB> crop_filter;
        crop_filter.setInputCloud(cloud);

        // Define the bounding box (approximated cylinder as a box)
        Eigen::Vector4f min_point(-cylinder_radius_, -cylinder_radius_, cylinder_min_z_, 1.0f);
        Eigen::Vector4f max_point(cylinder_radius_, cylinder_radius_, cylinder_max_z_, 1.0f);

        crop_filter.setMin(min_point);
        crop_filter.setMax(max_point);

        // Filtered output
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        crop_filter.filter(*temp_cloud);

        // Further refine by exact cylinder equation if necessary
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        final_cloud->reserve(temp_cloud->size()); // Reserve memory

        for (const auto &point : temp_cloud->points)
        {
            float distance_squared = point.x * point.x + point.y * point.y;
            if (distance_squared <= cylinder_radius_ * cylinder_radius_)
                final_cloud->points.push_back(point);
        }

        final_cloud->width = static_cast<uint32_t>(final_cloud->points.size());
        final_cloud->height = 1;
        final_cloud->is_dense = true;
        cloud.swap(final_cloud);
    }
} 
