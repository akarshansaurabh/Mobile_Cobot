#ifndef POINTCLOUD_TRANSFORMER2_HPP
#define POINTCLOUD_TRANSFORMER2_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/time.h>


// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/surface/convex_hull.h>

// Visualization Headers
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <limits>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace std;

namespace pointcloud_transformer2
{
    class PointCloudTransformer : public rclcpp::Node
    {
    public:
        PointCloudTransformer();
        ~PointCloudTransformer() = default;

    private:
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
        void pointCloud_TableDetectionCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
        void removePlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
        void extractClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                             std::vector<pcl::PointIndices> &cluster_indices);
        bool identifyTopFace(const pcl::PointIndices &cluster_indices, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &top_faces_cloud, geometry_msgs::msg::Pose &box_pose);
        bool TableIsPresent(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, vector<geometry_msgs::msg::Point> &vertices);

        rcl_interfaces::msg::SetParametersResult StartPC_ProcessingCallback(const std::vector<rclcpp::Parameter> &parameters);
        OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        // Helper function to filter points within a cylindrical region
        void filterPointCloudIn_ROI(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std_msgs::msg::Header &header_);

        // visualization
        void publishMarkerArray(const std::vector<geometry_msgs::msg::Pose> &box_poses);
        void visualizeVector(const geometry_msgs::msg::Point &origin, const geometry_msgs::msg::Vector3 &vector, int id,
                             visualization_msgs::msg::MarkerArray &marker_array, float r, float g, float b);
        void publishTableVertices(const std::vector<geometry_msgs::msg::Point> &vertices);

        // ROS Subscribers and Publishers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr table_detection_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

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

        // Plane Removal Parameters
        double plane_distance_threshold_;
        int min_plane_inliers_;
        bool ransac_performed_successfully_;

        // Clustering Parameters
        int min_cluster_size_, max_cluster_size_;
        double cluster_tolerance_;

        // Tracking previous markers for deletion
        size_t previous_marker_count_;
        int min_required_points_for_RANSAC_;

        // ROS2 Parameters
        std::string start_processing_;

        int sub_counter;
    };

} // namespace pointcloud_transformer

#endif // POINTCLOUD_TRANSFORMER_HPP
