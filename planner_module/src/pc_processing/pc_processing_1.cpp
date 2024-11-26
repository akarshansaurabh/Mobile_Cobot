#include "pc_processing/pc_processing_1.hpp"

using namespace std;

namespace pointcloud_transformer
{
    PointCloudTransformer::PointCloudTransformer()
        : Node("pc_processing_1_node"),
          target_frame_("map"),
          voxel_grid_leaf_size_(0.1), // Default voxel grid leaf size (5cm)
          mean_k_(40),                // Default mean K for outlier removal
          std_dev_mul_thresh_(1.0),   // Default std dev multiplier threshold
          cylinder_radius_(2.0),      // Radius of 1.5 meters
          cylinder_min_z_(0.0f),      // Adjust based on robot's height
          cylinder_max_z_(2.25)       // Adjust based on robot's height
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // tracks transform at current time

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/arm_rgbd_camera/points", 10,
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for the transformed point cloud
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/arm_rgbd_camera/points_map", 10);
    }

    void PointCloudTransformer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        /*
            Convert ROS PointCloud2 message to PCL PointCloud with RGB (cam frame)
            Downsample the point cloud using VoxelGrid filter  (cam frame)
            Outlier Removal using StatisticalOutlierRemoval  (cam frame)
            Convert filtered PCL PointCloud back to ROS PointCloud2  (cam frame)
            Perform the transformation into map frame
        */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *raw_pcl_cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(raw_pcl_cloud);
        voxel_filter.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
        voxel_filter.filter(*downsampled_pcl_cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_downsampled_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(downsampled_pcl_cloud);
        sor.setMeanK(mean_k_);
        sor.setStddevMulThresh(std_dev_mul_thresh_);
        sor.filter(*filtered_downsampled_pcl_cloud);

        sensor_msgs::msg::PointCloud2 downsampled_filtered_msg;
        pcl::toROSMsg(*filtered_downsampled_pcl_cloud, downsampled_filtered_msg);
        downsampled_filtered_msg.header = msg->header; // Preserve the original header

        sensor_msgs::msg::PointCloud2 transformed_cloud_base, roi_cloud_base, transformed_cloud_map;

        // transform wrt base_link, cylindrical filtering, transform wrt map
        std::string base_frame = "base_link";
        try
        {
            // Wait for the transform to become available
            if (!tf_buffer_->canTransform(base_frame, downsampled_filtered_msg.header.frame_id,
                                          downsampled_filtered_msg.header.stamp, rclcpp::Duration::from_seconds(1.0)))
            {
                RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s at time %d.%d",
                            downsampled_filtered_msg.header.frame_id.c_str(), base_frame.c_str(),
                            downsampled_filtered_msg.header.stamp.sec, downsampled_filtered_msg.header.stamp.nanosec);
                return;
            }

            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(base_frame, downsampled_filtered_msg.header.frame_id,
                                                                                         downsampled_filtered_msg.header.stamp);
            tf2::doTransform(downsampled_filtered_msg, transformed_cloud_base, transform);
            transformed_cloud_base.header = msg->header;
            transformed_cloud_base.header.frame_id = base_frame;
            roi_cloud_base = FilterPointsInCylinder(transformed_cloud_base, cylinder_radius_, cylinder_min_z_, cylinder_max_z_);
            roi_cloud_base.header = msg->header;
            roi_cloud_base.header.frame_id = base_frame;
            // Publish the transformed point cloud
            // pointcloud_pub_->publish(transformed_cloud);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        // transform wrt map
        try
        {
            // Wait for the transform to become available
            if (!tf_buffer_->canTransform(target_frame_, base_frame,
                                          downsampled_filtered_msg.header.stamp, rclcpp::Duration::from_seconds(1.0)))
            {
                RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s at time %d.%d",
                            base_frame.c_str(), target_frame_.c_str(),
                            downsampled_filtered_msg.header.stamp.sec, downsampled_filtered_msg.header.stamp.nanosec);
                return;
            }

            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(target_frame_, base_frame,
                                                                                         downsampled_filtered_msg.header.stamp);
            tf2::doTransform(roi_cloud_base, transformed_cloud_map, transform);
            transformed_cloud_map.header.frame_id = target_frame_;

            // Publish the transformed point cloud
            pointcloud_pub_->publish(transformed_cloud_map);
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

    // Helper Function wrt base_link (reason : cylinder defined in base frame)
    sensor_msgs::msg::PointCloud2 PointCloudTransformer::FilterPointsInCylinder(
        const sensor_msgs::msg::PointCloud2 &cloud,
        float radius, float min_z, float max_z)
    {
        // Initialize the filtered cloud with the same header and fields
        sensor_msgs::msg::PointCloud2 filtered_cloud;
        filtered_cloud.header = cloud.header;
        filtered_cloud.height = 1;       // Unordered point cloud
        filtered_cloud.width = 0;        // To be updated based on filtered points
        filtered_cloud.is_dense = false; // Could contain NaNs after filtering
        filtered_cloud.is_bigendian = cloud.is_bigendian;
        filtered_cloud.point_step = cloud.point_step;
        filtered_cloud.fields = cloud.fields; // Copy all fields

        // Reserve space for filtered data (optional optimization)
        // Estimate that up to half the points may pass the filter
        filtered_cloud.data.reserve(cloud.data.size() / 2);

        // Create iterators for reading from the input cloud
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        // Initialize a counter to keep track of the point index
        size_t point_idx = 0;

        // Iterate through each point in the input cloud
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_idx)
        {
            if (*iter_x <= this->cylinder_radius_ && *iter_y <= this->cylinder_radius_ && (*iter_z) >= this->cylinder_min_z_ && (*iter_z) <= this->cylinder_max_z_)
            {
                float distance = std::sqrt((*iter_x) * (*iter_x) + (*iter_y) * (*iter_y));
                if (distance <= this->cylinder_radius_)
                {
                    // Calculate the starting byte index of the current point
                    size_t point_offset = point_idx * cloud.point_step;

                    // Append the point's data to the filtered_cloud.data buffer
                    filtered_cloud.data.insert(
                        filtered_cloud.data.end(),
                        cloud.data.begin() + point_offset,
                        cloud.data.begin() + point_offset + cloud.point_step);

                    // Increment the width of the filtered cloud
                    filtered_cloud.width += 1;
                }
            }
        }
        // Update the row_step based on the new width
        filtered_cloud.row_step = filtered_cloud.point_step * filtered_cloud.width;
        cout << "number of points inside the cylinder = " << filtered_cloud.width << endl;
        return filtered_cloud;
    }
}

