#include "pc_processing/octomap_generator.hpp"

namespace octoMapGenerator
{
    PointCloudStitcherMethod2::PointCloudStitcherMethod2(rclcpp::Node::SharedPtr node, const std::string &map_frame)
        : node_(node), map_frame_(map_frame)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        accumulated_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    }

    /*
        cam wrt map
        downsampling and pc transformation
        addition
    */
    void PointCloudStitcherMethod2::pointCloudSnapShotCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &snapshot_msg)
    {
        std::string camera_frame = msg.header.frame_id;
        try
        {
            map_to_camera_transform = tf_buffer_->lookupTransform(map_frame_, camera_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "TF error %s->%s: %s", map_frame_.c_str(), camera_frame.c_str(), ex.what());
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_snapshot_in_map = transformCloudToMap(*snapshot_msg);
        addCloudSnapshot(pcl_snapshot_in_map);

        pointcloud_sub_.reset();
    }

    // source of error - robot is at A1 when snapshot taken, but in TF, robot is at A1'
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudStitcherMethod2::transformCloudToMap(const sensor_msgs::msg::PointCloud2 &msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_camera(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(msg, *cloud_in_camera);

        // Downsample using VoxelGrid filter
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(cloud_in_camera);
        voxel_filter.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
        voxel_filter.filter(*cloud_in_camera);
        // Remove outliers
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_filter;
        sor_filter.setInputCloud(cloud_in_camera);
        sor_filter.setMeanK(mean_k_);
        sor_filter.setStddevMulThresh(std_dev_mul_thresh_);
        sor_filter.filter(*cloud_in_camera);

        Eigen::Affine3d map_cam = poseToEigen(map_to_camera_transform.transform);

        pcl::transformPointCloud(*cloud_in_camera, *cloud_in_camera, map_cam.matrix());
        return cloud_in_camera;
    }

    void PointCloudStitcherMethod2::addCloudSnapshot(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &snapshot)
    {
        *accumulated_cloud_ += *snapshot;
    }

    void PointCloudStitcherMethod2::finalizeStitching(float leaf_size)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(accumulated_cloud_);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*accumulated_cloud_);
        RCLCPP_INFO(node_->get_logger(), "Final stitched cloud has %zu points.", accumulated_cloud_->size());
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudStitcherMethod2::getStitchedCloud()
    {
        return accumulated_cloud_;
    }

    Eigen::Affine3d PointCloudStitcherMethod2::poseToEigen(const geometry_msgs::msg::Transform &transform)
    {
        Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
        Eigen::Translation3d t(transform.translation.x, transform.translation.y, transform.translation.z);
        return t * q;
    }
}