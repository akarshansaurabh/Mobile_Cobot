#ifndef THREED_PERCEPTION_HPP
#define THREED_PERCEPTION_HPP

#include <mutex>
#include <atomic>
#include <omp.h>

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

#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d_omp.h> // For NormalEstimationOMP

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/search/kdtree.h>

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

namespace environment3DPerception
{
    enum class PerceptionObject
    {
        ENTIRE_3D_SCENE,
        WALL,
        FLOOR,
        UNKNOWN_OBJECT,
        SUB_UNKNOWN_OBJECT
    };

    struct SegmentTreeNode
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud;
        PerceptionObject object_type;
        std::vector<std::shared_ptr<SegmentTreeNode>> children;

        SegmentTreeNode() : object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()) {}
    };

    namespace SegmentationCondition
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cleanCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_with_normals,
                                                      pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr &tree, int K);
        bool tableLegCondition(const pcl::PointXYZRGBNormal &p1, const pcl::PointXYZRGBNormal &p2, float squared_distance);
    }

    class Environment3DPerception
    {
    public:
        // Constructor requires a shared Node pointer
        explicit Environment3DPerception(rclcpp::Node::SharedPtr node);
        ~Environment3DPerception() = default;

    private:
        // ros2 callbacks
        rcl_interfaces::msg::SetParametersResult onParameterEvent(const std::vector<rclcpp::Parameter> &parameters);
        void timerCallback();
        void frontCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void armCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // Transform and merge
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr preprocessAndTransform(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);
        bool transformToMapFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in_out, const std::string &from_frame);
        // Post-processing of the final merged cloud
        void postProcessing();

        // ros2 variables
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr arm_sub_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_; // used for parameter registration
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        // TF buffer and listener
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        // Mutex for shared cloud
        std::mutex cloud_mutex_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr environment_3d_pointcloud_;

        // Parameter: activate_3d_perception
        std::string activate_3d_perception_;

        // To ensure each subscriber callback only fires once
        std::atomic<bool> front_camera_received_;
        std::atomic<bool> arm_camera_received_;

        // -------------- Segmentation Pipeline --------------
        // Build a 3-level segmentation tree:
        //  - Root: entire scene
        //  - Children: walls, floor, object
        //  - For "object": further sub-clustering => each sub-object is a child
        std::shared_ptr<SegmentTreeNode> Construct3DScene_SegmentationTree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractAllVerticalPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_cloud, float min_inliers);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractFloorPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_cloud, float min_inliers);
        void subdivideObjectNode(std::shared_ptr<SegmentTreeNode> object_node);
        std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
        segmentTableLegsConditionalWithCleaning(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);
        void traverseSegmentationTree(const std::shared_ptr<SegmentTreeNode> &node, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_pcl_cloud);
    };
}

#endif