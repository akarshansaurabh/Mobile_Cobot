#include "pc_processing/pointcloud_processor.hpp"

namespace pointcloud_processing
{
    PointCloudProcessor::PointCloudProcessor()
        : Node("pointcloud_processor_node"),
          target_frame_("map"),
          base_frame_("base_link"),
          voxel_grid_leaf_size_(0.01f),
          mean_k_(50),
          std_dev_mul_thresh_(1.0),
          roi_min_x_(0.0f),
          roi_max_x_(6.0f),
          roi_min_y_(-4.0f),
          roi_max_y_(4.0f),
          roi_min_z_(0.0f),
          roi_max_z_(2.0f),
          plane_distance_threshold_(0.01),
          min_plane_inliers_(800),
          ransac_success_(false),
          min_cluster_size_(30),
          max_cluster_size_(1000),
          cluster_tolerance_(0.07),
          processing_mode_("stop"),
          box_detection_counter_(0),
          table_detection_counter_(0)
    {
        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Parameter callback
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PointCloudProcessor::parameterCallback, this, std::placeholders::_1));

        // Publisher for the processed point cloud
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/arm_rgbd_camera/points_map", rclcpp::QoS(10));

        // Declare parameters
        this->declare_parameter("pc_processing_param", "stop");
    }
    void PointCloudProcessor::initialize()
    {
        vis_manager_ = std::make_shared<visualization::VisualizationManager>(shared_from_this());
    }

    // Callback for processing parameters
    rcl_interfaces::msg::SetParametersResult PointCloudProcessor::parameterCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            if (param.get_name() == "pc_processing_param")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
                {
                    table_detection_counter_ = box_detection_counter_ = 0;
                    voxel_grid_leaf_size_ = 0.01;
                    min_plane_inliers_ = 800;
                    min_cluster_size_ = 30;
                    max_cluster_size_ = 1000;
                    processing_mode_ = param.as_string();
                    RCLCPP_INFO(this->get_logger(), "Processing mode set to: '%s'", processing_mode_.c_str());
                    ransac_success_ = false;
                    if (processing_mode_ == "detect_boxes")
                    {
                        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                            "/arm_rgbd_camera/points", rclcpp::QoS(10),
                            std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));
                    }
                    else if (processing_mode_ == "detect_table")
                    {
                        table_detection_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                            "/arm_rgbd_camera/points", rclcpp::QoS(10),
                            std::bind(&PointCloudProcessor::pointCloudTableDetectionCallback, this, std::placeholders::_1));
                    }
                    else
                    {
                        // Unrecognized mode
                        RCLCPP_WARN(this->get_logger(), "Unrecognized processing mode: '%s'", processing_mode_.c_str());
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid type for 'pc_processing_param'. Expected string.");
                    result.successful = false;
                    result.reason = "Invalid parameter type.";
                }
                break;
            }
        }
        return result;
    }

    void PointCloudProcessor::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        box_detection_counter_++;
        auto start_time = std::chrono::steady_clock::now();

        // Convert ROS PointCloud2 message to PCL PointCloud
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Preprocess the point cloud
        preprocessPointCloud(pcl_cloud);

        // Transform point cloud to target frame
        if (!transformPointCloudToTargetFrame(pcl_cloud, msg->header))
            return;

        // Filter points within ROI
        // filterPointCloudInROI(pcl_cloud, msg->header);
        // if (pcl_cloud->empty())
        // {
        //     RCLCPP_WARN(this->get_logger(), "No points remain after ROI filtering.");
        //     pointcloud_sub_.reset();
        //     return;
        // }

        // Remove planes
        removePlanes(pcl_cloud);
        if (pcl_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points remain after plane removal.");
            return;
        }

        // Extract clusters
        std::vector<pcl::PointIndices> cluster_indices;
        extractClusters(pcl_cloud, cluster_indices);
        RCLCPP_INFO(this->get_logger(), "Number of clusters: %zu", cluster_indices.size());

        // // Identify top faces
        bool box_found = false;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr top_faces(new pcl::PointCloud<pcl::PointXYZRGB>());
        std::vector<geometry_msgs::msg::Pose> box_poses;
        geometry_msgs::msg::Pose box_pose;

        for (const auto &indices : cluster_indices)
            if (identifyTopFace(indices, pcl_cloud, top_faces, box_pose))
            {
                box_found = true;
                box_poses.push_back(std::move(box_pose));
            }

        if (box_found)
        {
            RCLCPP_INFO(this->get_logger(), "Box found in the point cloud.");
            pcl_cloud.swap(top_faces);
            vis_manager_->publishMarkerArray(box_poses);
        }
        else
            RCLCPP_WARN(this->get_logger(), "Box not found in the point cloud.");

        // Publish the processed point cloud
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*pcl_cloud, output_msg);
        output_msg.header.stamp = msg->header.stamp;
        output_msg.header.frame_id = target_frame_;
        pointcloud_pub_->publish(output_msg);

        auto end_time = std::chrono::steady_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        RCLCPP_INFO(this->get_logger(), "Processed point cloud in %ld ms.", processing_time);

        if (!ransac_success_ || box_detection_counter_ > 2) // box_found
        {
            RCLCPP_INFO(this->get_logger(), "Resetting subscriber.");
            pointcloud_sub_.reset();
        }
    }

    void PointCloudProcessor::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
    {
        // Downsample using VoxelGrid filter
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
        voxel_filter.filter(*cloud);

        // Remove outliers
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_filter;
        sor_filter.setInputCloud(cloud);
        sor_filter.setMeanK(mean_k_);
        sor_filter.setStddevMulThresh(std_dev_mul_thresh_);
        sor_filter.filter(*cloud);
    }

    bool PointCloudProcessor::transformPointCloudToTargetFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std_msgs::msg::Header &header)
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_->lookupTransform(target_frame_, header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.1));

            Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped.transform);
            Eigen::Matrix4f transform_matrix = transform.cast<float>().matrix();

            pcl::transformPointCloud(*cloud, *cloud, transform_matrix);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
            return false;
        }
    }

    void PointCloudProcessor::filterPointCloudInROI(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std_msgs::msg::Header &header)
    {
        // Use CropBox filter
        pcl::CropBox<pcl::PointXYZRGB> crop_filter;
        crop_filter.setInputCloud(cloud);

        geometry_msgs::msg::PointStamped local_point_min, local_point_max, global_point_min, global_point_max;
        local_point_min.header.frame_id = local_point_max.header.frame_id = base_frame_;
        local_point_min.header.stamp = local_point_max.header.stamp = header.stamp;
        local_point_min.point.x = roi_min_x_;
        local_point_min.point.y = roi_min_y_;
        local_point_min.point.z = roi_min_z_;
        local_point_max.point.x = roi_max_x_;
        local_point_max.point.y = roi_max_y_;
        local_point_max.point.z = roi_max_z_;
        try
        {
            tf_buffer_->transform(local_point_min, global_point_min, target_frame_, tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }
        try
        {
            tf_buffer_->transform(local_point_max, global_point_max, target_frame_, tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }

        Eigen::Vector4f min_point(global_point_min.point.x, global_point_min.point.y, global_point_min.point.z, 1.0f);
        Eigen::Vector4f max_point(global_point_max.point.x, global_point_max.point.y, global_point_max.point.z, 1.0f);
        crop_filter.setMin(min_point);
        crop_filter.setMax(max_point);
        crop_filter.filter(*cloud);
    }

    void PointCloudProcessor::removePlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
    {
        // Remove dominant planes iteratively (floor, walls, table)
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_distance_threshold_);

        while (true)
        {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() < static_cast<size_t>(min_plane_inliers_))
                break;

            // Calculate plane normal
            Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            plane_normal.normalize();

            // Determine plane type based on normal
            bool is_horizontal = (fabs(plane_normal.dot(Eigen::Vector3f(0, 0, 1))) > 0.95);
            bool is_vertical = (fabs(plane_normal.dot(Eigen::Vector3f(1, 0, 0))) > 0.95) || (fabs(plane_normal.dot(Eigen::Vector3f(0, 1, 0))) > 0.95);

            if (is_horizontal || is_vertical)
            {
                // Remove the plane from the point cloud
                extract.setInputCloud(cloud);
                extract.setIndices(inliers);
                extract.setNegative(true);
                extract.filter(*cloud);
            }
        }
    }

    void PointCloudProcessor::extractClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                              std::vector<pcl::PointIndices> &cluster_indices)
    {
        // Create a KD-Tree for clustering
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        tree->setInputCloud(cloud);

        // Euclidean Cluster Extraction
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
    }

    bool PointCloudProcessor::identifyTopFace(const pcl::PointIndices &cluster_indices, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &top_faces_cloud, geometry_msgs::msg::Pose &box_pose)
    {
        // Extract the cluster point cloud from the input cloud using the provided indices
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract_cluster;
        pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(cluster_indices));
        extract_cluster.setInputCloud(cloud);
        extract_cluster.setIndices(cluster_indices_ptr);
        extract_cluster.setNegative(false);
        extract_cluster.filter(*cluster_cloud);

        if (cluster_cloud->points.size() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Cluster has no points for plane segmentation.");
            ransac_success_ = true;
            voxel_grid_leaf_size_ /= 2.0;
            min_plane_inliers_ *= 2;
            min_cluster_size_ *= 2;
            max_cluster_size_ *= 2;
            return false;
        }

        // Segment planes in the cluster
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::ExtractIndices<pcl::PointXYZRGB> extract_plane;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_distance_threshold_);

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_copy(new pcl::PointCloud<pcl::PointXYZRGB>(*cluster_cloud));
        int num_planes_extracted = 0;

        while (num_planes_extracted < 6)
        {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            seg.setInputCloud(cluster_cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() < 1)
            {
                RCLCPP_ERROR(this->get_logger(), "RANSAC FAILED BECAUSE OF LESS POINTS");
                // num_planes_extracted++;
                // extract_plane.setInputCloud(cluster_cloud);
                // extract_plane.setIndices(inliers);
                // extract_plane.setNegative(true);
                // extract_plane.filter(*cluster_cloud);
                break;
            }

            // Calculate plane normal
            Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            plane_normal.normalize();

            bool is_horizontal = (fabs(plane_normal.dot(Eigen::Vector3f(0, 0, 1))) > 0.95);

            if (is_horizontal)
            {
                // Extract the plane points
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr horizontal_plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
                extract_plane.setInputCloud(cluster_cloud);
                extract_plane.setIndices(inliers);
                extract_plane.setNegative(false);
                extract_plane.filter(*horizontal_plane_cloud);

                // Compute average Z value
                double sum_z = 0.0;
                for (const auto &point : horizontal_plane_cloud->points)
                    sum_z += point.z;
                double avg_z = sum_z / horizontal_plane_cloud->points.size();

                // Define a range for top face (e.g., above 0.5m and below 2.0m)
                if (avg_z > 0.5 && avg_z < 2.0)
                {
                    // Found the top face
                    // Compute centroid and orientation
                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(*horizontal_plane_cloud, centroid);

                    Eigen::Matrix3f covariance;
                    pcl::computeCovarianceMatrixNormalized(*horizontal_plane_cloud, centroid, covariance);

                    // Eigen decomposition
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
                    Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();

                    // Ensure right-handed coordinate system
                    Eigen::Vector3f normal = eigen_vectors.col(0);
                    if (normal.dot(Eigen::Vector3f(0.0, 0.0, 1.0)) < 0)
                        normal = -normal;

                    Eigen::Vector3f axis1 = eigen_vectors.col(1);
                    Eigen::Vector3f axis2 = normal.cross(axis1);

                    // Construct rotation matrix
                    Eigen::Matrix3f rotation;
                    rotation.col(0) = axis1;
                    rotation.col(1) = axis2;
                    rotation.col(2) = normal;

                    // Convert rotation matrix to quaternion
                    Eigen::Quaternionf quat(rotation);

                    // Fill box_pose
                    box_pose.position.x = centroid[0];
                    box_pose.position.y = centroid[1];
                    box_pose.position.z = centroid[2];
                    box_pose.orientation.x = quat.x();
                    box_pose.orientation.y = quat.y();
                    box_pose.orientation.z = quat.z();
                    box_pose.orientation.w = quat.w();
                    // break;
                    *top_faces_cloud += *horizontal_plane_cloud;
                    return true;
                }
            }

            // Remove the plane points from the cluster_cloud_copy
            extract_plane.setInputCloud(cluster_cloud);
            extract_plane.setIndices(inliers);
            extract_plane.setNegative(true);
            extract_plane.filter(*cluster_cloud);
            num_planes_extracted++;
        }
        return false;
    }

    void PointCloudProcessor::pointCloudTableDetectionCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        table_detection_counter_++;

        // Convert ROS PointCloud2 message to PCL PointCloud
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Preprocess the point cloud
        preprocessPointCloud(pcl_cloud);

        // Transform point cloud to target frame
        if (!transformPointCloudToTargetFrame(pcl_cloud, msg->header))
        {
            table_detection_sub_.reset();
            return;
        }

        // Filter points within ROI
        // filterPointCloudInROI(pcl_cloud, msg->header);
        // if (pcl_cloud->empty())
        // {
        //     RCLCPP_WARN(this->get_logger(), "No points remain after ROI filtering.");
        //     return;
        // }

        // Detect table
        std::vector<geometry_msgs::msg::Point> table_vertices;
        if (tableIsPresent(pcl_cloud, table_vertices))
        {
            RCLCPP_INFO(this->get_logger(), "Table detected.");
            vis_manager_->publishTableVertices(table_vertices);
        }
        else
            RCLCPP_WARN(this->get_logger(), "Table not found.");

        // Reset the subscriber after processing
        if (table_detection_counter_ == 2)
            table_detection_sub_.reset();
    }

    bool PointCloudProcessor::tableIsPresent(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<geometry_msgs::msg::Point> &vertices)
    {
        vertices.clear();
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setKeepOrganized(false);

        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_distance_threshold_);

        while (true)
        {
            if (cloud->points.empty())
                return false;

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() < min_plane_inliers_)
                return false;

            Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            plane_normal.normalize();

            double sum_z = 0.0;
            for (const auto &idx : inliers->indices)
                sum_z += cloud->points[idx].z;
            double avg_z = sum_z / static_cast<double>(inliers->indices.size());
            bool is_table = (fabs(plane_normal.dot(Eigen::Vector3f(0, 0, 1))) > 0.95 && (avg_z > 0.5) && (avg_z < 1.5));

            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative((is_table == true) ? false : true);
            extract.filter(*cloud);
            if (is_table)
            {
                // cloud is the table plane
                double min_x, max_x, min_y, max_y;
                min_x = max_x = cloud->points[0].x;
                min_y = max_y = cloud->points[0].y;

                // Iterate through the point cloud once
                for (const auto &point : cloud->points)
                {
                    if (point.x < min_x)
                        min_x = point.x;
                    else if (point.x > max_x)
                        max_x = point.x;
                    if (point.y < min_y)
                        min_y = point.y;
                    else if (point.y > max_y)
                        max_y = point.y;
                }

                std::vector<Eigen::Vector3f> table_corners(4);
                table_corners[0] = Eigen::Vector3f(min_x, min_y, avg_z);
                table_corners[1] = Eigen::Vector3f(max_x, min_y, avg_z);
                table_corners[2] = Eigen::Vector3f(max_x, max_y, avg_z);
                table_corners[3] = Eigen::Vector3f(min_x, max_y, avg_z);

                geometry_msgs::msg::Point point;
                for (const auto &corner : table_corners)
                {
                    point.x = corner.x();
                    point.y = corner.y();
                    point.z = corner.z();
                    vertices.emplace_back(std::move(point));
                }
                return true;
            }
        }
    }
}
