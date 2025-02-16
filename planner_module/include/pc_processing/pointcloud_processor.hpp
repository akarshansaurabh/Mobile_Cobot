#ifndef POINTCLOUD_PROCESSOR_HPP
#define POINTCLOUD_PROCESSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// #include <pcl/segmentation/dbscan.h>

#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include "visualizations/visualization_manager.hpp"

#include <iostream>
#include <memory>
#include "planner_module/waypoint_gen.hpp"
#include "custom_interfaces/srv/boxpose_estimator.hpp"
#include "custom_interfaces/msg/table_vertices.hpp"
#include "pc_processing/octomap_generator.hpp"

#include <future>
#include <limits>

using namespace std;

namespace pointcloud_processing
{
    struct LineSegment
    {
        Eigen::Vector3f start;
        Eigen::Vector3f end;
        float length;
    };

    class PointCloudProcessor : public rclcpp::Node
    {
    public:
        PointCloudProcessor();
        void initialize();
        ~PointCloudProcessor() = default;

    private:
        std::vector<geometry_msgs::msg::Point> table_vertices;
        double min_x, max_x, min_y, max_y;
        // Callback functions
        void pointCloudTableDetectionCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
        void Server6DPoseCallback(const std::shared_ptr<custom_interfaces::srv::BoxposeEstimator::Request> request,
                                  std::shared_ptr<custom_interfaces::srv::BoxposeEstimator::Response> response);
        // Processing functions
        void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, bool is_table);
        bool transformPointCloudToTargetFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std_msgs::msg::Header &header);
        void filterPointCloudInROI(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std_msgs::msg::Header &header,
                                   const std::vector<geometry_msgs::msg::Point> &table_vertices, double min_x, double max_x, double min_y, double max_y);
        void removeNormalPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
        void extractClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                             std::vector<pcl::PointIndices> &cluster_indices);
        bool computeOBBForCluster(const pcl::PointIndices &cluster_indices,
                                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                  geometry_msgs::msg::Pose &box_pose);
        bool tableIsPresent(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<geometry_msgs::msg::Point> &vertices, const rclcpp::Time &timestamp);
        bool ComputeBox4DPose(const pcl::PointIndices &cluster_indices, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &top_faces_cloud, geometry_msgs::msg::Pose &box_pose);
        bool ComputeBox6DPose(const pcl::PointIndices &cluster_indices, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &top_faces_cloud, geometry_msgs::msg::Pose &box_pose);

        // Parameter callback
        rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters);
        OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

        // ROS subscribers and publishers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr table_detection_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr move_amr_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr box_poses_pub;
        rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr table_vertices_pub;
        rclcpp::Publisher<custom_interfaces::msg::TableVertices>::SharedPtr table_vertices_pub_;
        rclcpp::Service<custom_interfaces::srv::BoxposeEstimator>::SharedPtr box6dposes_server_;

        // TF2 buffer and listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Parameters
        std::string target_frame_;
        std::string base_frame_;

        // Preprocessing parameters
        float voxel_grid_leaf_size_;
        int mean_k_;
        double std_dev_mul_thresh_;

        // ROI filtering parameters
        float roi_min_x_, roi_max_x_;
        float roi_min_y_, roi_max_y_;
        float roi_min_z_, roi_max_z_;

        // Plane removal parameters
        double plane_distance_threshold_;
        int min_plane_inliers_;
        bool ransac_success_;

        // Clustering parameters
        int min_cluster_size_, max_cluster_size_;
        double cluster_tolerance_;

        // Processing control
        std::string processing_mode_;
        int box_detection_counter_, table_detection_counter_;

        std::shared_ptr<visualization::VisualizationManager> vis_manager_;
        std::shared_ptr<waypointGen::TableWayPointGen> way_point_generator_;

        // Parameter clients
        rclcpp::AsyncParametersClient::SharedPtr nav2_actionclient_param_client_;
        // rclcpp::AsyncParametersClient::SharedPtr arm_controller_param_client_;
        void ResendTableAsDestination();
    };
}

#endif
