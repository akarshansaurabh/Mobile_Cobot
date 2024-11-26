#ifndef POINTCLOUD_TRANSFORMER2_HPP
#define POINTCLOUD_TRANSFORMER2_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>

namespace pointcloud_transformer2
{

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer();
    ~PointCloudTransformer() = default;

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &raw_input_cloud_msg);

    // Helper function to filter points within a cylindrical region
    void filterPointCloudInCylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    // ROS Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    // TF2 Buffer and Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string target_frame_;
    std::string base_frame_;

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

} // namespace pointcloud_transformer

#endif // POINTCLOUD_TRANSFORMER_HPP
