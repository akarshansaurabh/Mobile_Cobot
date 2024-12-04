#include "pc_processing/pc_processing_2.hpp"

using namespace std::chrono_literals;
using namespace std;

namespace pointcloud_transformer2
{
    PointCloudTransformer::PointCloudTransformer()
        : Node("pointcloud_transformer_node"),
          target_frame_("map"),
          base_frame_("base_link"),
          voxel_grid_leaf_size_(0.01), // Smaller leaf size for finer downsampling
          mean_k_(50),                 // Adjusted mean K for outlier removal
          std_dev_mul_thresh_(1.0),    // Tighter threshold for outlier removal
          cylinder_radius_(2.0f),      // Reduced radius for focused region
          cylinder_min_z_(0.0f),
          cylinder_max_z_(2.0f),
          min_plane_inliers_(800),
          plane_distance_threshold_(0.01),
          cluster_tolerance_(0.05),
          min_cluster_size_(30),
          max_cluster_size_(1000),
          min_required_points_for_RANSAC_(50),
          ransac_performed_successfully_(false)
    {
        // Initialize TF2 Buffer and Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Parameter callback
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PointCloudTransformer::StartPC_ProcessingCallback, this, std::placeholders::_1));

        // Publisher for the processed point cloud
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/arm_rgbd_camera/points_map", rclcpp::QoS(10));

        // Publisher for the visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/box_marker_array", rclcpp::QoS(10));

        // table_detection_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/arm_rgbd_camera/points", rclcpp::QoS(10),
        //     std::bind(&PointCloudTransformer::pointCloud_TableDetectionCallback, this, std::placeholders::_1));

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/arm_rgbd_camera/points", rclcpp::QoS(10),
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

        this->declare_parameter("pc_processing_param", "stop");
    }

    void PointCloudTransformer::pointCloud_TableDetectionCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        cylinder_radius_ = 3.0f;
        cout << "callback" << endl;
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

        // Transform point cloud to target_frame_ (map frame)
        Eigen::Matrix4f tf_cam_wrt_map = Eigen::Matrix4f::Identity();
        try
        {
            geometry_msgs::msg::TransformStamped tf_armcam_to_map =
                tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

            // Convert geometry_msgs Transform to Eigen Matrix
            Eigen::Affine3d eigen_transform = tf2::transformToEigen(tf_armcam_to_map.transform);
            tf_cam_wrt_map = eigen_transform.cast<float>().matrix();
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform from %s to %s: %s",
                        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
            return;
        }
        // pcl_cloud -> source;   tf -> source wrt target
        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, tf_cam_wrt_map);

        // Filter points within the cuboidal region
        filterPointCloudIn_ROI(pcl_cloud, msg->header);
        // Check if any points remain after filtering
        if (pcl_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points remain after cylinder filtering.");
            table_detection_sub_.reset();
            return;
        }

        // Remove dominant planes (floor, walls, table)
        vector<geometry_msgs::msg::Point> vertices;
        if (TableIsPresent(pcl_cloud, vertices))
        {
            cout << "table is found" << endl;
            publishTableVertices(vertices);
        }
        else
            cout << "table is NOT found" << endl;

        cylinder_radius_ = 2.0f;
        table_detection_sub_.reset();
        cout << "unsubscribing" << endl;
    }

    void PointCloudTransformer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        sub_counter++;
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

        // Transform point cloud to target_frame_ (map frame)
        Eigen::Matrix4f tf_cam_wrt_map = Eigen::Matrix4f::Identity();
        try
        {
            geometry_msgs::msg::TransformStamped tf_armcam_to_map =
                tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

            // Convert geometry_msgs Transform to Eigen Matrix
            Eigen::Affine3d eigen_transform = tf2::transformToEigen(tf_armcam_to_map.transform);
            tf_cam_wrt_map = eigen_transform.cast<float>().matrix();
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform from %s to %s: %s",
                        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
            return;
        }
        // pcl_cloud -> source;   tf -> source wrt target
        pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, tf_cam_wrt_map);

        // Filter points within the cuboidal region
        filterPointCloudIn_ROI(pcl_cloud, msg->header);
        // Check if any points remain after filtering
        if (pcl_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points remain after cylinder filtering.");
            return;
        }

        // Remove dominant planes (floor, walls, table)
        removePlanes(pcl_cloud);
        // Check if any points remain after plane removal
        if (pcl_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points remain after plane removal.");
            return;
        }

        // Extract clusters from the remaining points
        std::vector<pcl::PointIndices> cluster_indices;
        extractClusters(pcl_cloud, cluster_indices);
        cout << "num of clusters = " << cluster_indices.size() << endl;
        // Identify the box cluster and compute the centroid of its top face
        bool box_found = false;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr top_faces(new pcl::PointCloud<pcl::PointXYZRGB>());
        geometry_msgs::msg::Pose box_pose;
        vector<geometry_msgs::msg::Pose> box_poses;
        for (const auto &indices : cluster_indices)
        {
            // Identify the top face and compute centroid
            if (identifyTopFace(indices, pcl_cloud, top_faces, box_pose))
            {
                box_found = true;
                box_poses.push_back(std::move(box_pose));
            }
        }
        if (!box_found)
        {
            RCLCPP_WARN(this->get_logger(), "Box not found in the point cloud.");
        }
        else
        {
            cout << "box found" << endl;
            pcl_cloud.swap(top_faces);
            publishMarkerArray(box_poses);
        }

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
        if (!ransac_performed_successfully_ || sub_counter > 2)
        {
            cout << "resetting" << endl;
            pointcloud_sub_.reset();
        }
    }

    // imagine cloud wrt map
    void PointCloudTransformer::filterPointCloudIn_ROI(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std_msgs::msg::Header &header_)
    {
        // Use PCL's CropBox filter to approximate cylinder filtering for efficiency
        pcl::CropBox<pcl::PointXYZRGB> crop_filter;
        crop_filter.setInputCloud(cloud);

        // Define the bounding box (approximated cylinder as a box)
        geometry_msgs::msg::PointStamped local_point_min, local_point_max, global_point_min, global_point_max;
        local_point_min.header.frame_id = local_point_max.header.frame_id = base_frame_;
        local_point_min.header.stamp = local_point_max.header.stamp = header_.stamp;
        local_point_min.point.x = -cylinder_radius_;
        local_point_min.point.y = -cylinder_radius_;
        local_point_min.point.z = cylinder_min_z_;
        local_point_max.point.x = cylinder_radius_;
        local_point_max.point.y = cylinder_radius_;
        local_point_max.point.z = cylinder_max_z_;
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

        // Filtered output
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        crop_filter.filter(*cloud);
    }

    bool PointCloudTransformer::TableIsPresent(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, vector<geometry_msgs::msg::Point> &vertices)
    {
        vertices.clear();

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setKeepOrganized(false);

        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_distance_threshold_);

        pcl::ConvexHull<pcl::PointXYZRGB> hull;
        hull.setDimension(2); // 2D Convex Hull on the plane

        while (true)
        {
            if (cloud->points.empty())
            {
                std::cout << "Point cloud is empty. No table detected." << std::endl;
                return false;
            }
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() < min_plane_inliers_)
                break;

            Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            plane_normal.normalize();

            double sum_z = 0.0;
            for (const auto &idx : inliers->indices)
            {
                sum_z += cloud->points[idx].z;
            }
            double avg_z = sum_z / static_cast<double>(inliers->indices.size());
            bool is_table = (fabs(plane_normal.dot(Eigen::Vector3f(0, 0, 1))) > 0.95 && (avg_z > 0.2) && (avg_z < 1.5));
            cout << "average z = " << avg_z << endl;

            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative((is_table == true) ? false : true);
            extract.filter(*cloud);
            if (is_table)
            {
                // cloud is the table plane

                // project the table point cloud on the RANSAC plane
                // pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_projected(new pcl::PointCloud<pcl::PointXYZRGB>());
                // pcl::ProjectInliers<pcl::PointXYZRGB> proj;
                // proj.setModelType(pcl::SACMODEL_PLANE);
                // proj.setInputCloud(cloud);
                // proj.setModelCoefficients(coefficients);
                // proj.filter(*table_projected);

                // // Compute the convex hull of the projected table points
                // pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points(new pcl::PointCloud<pcl::PointXYZRGB>());
                // hull.setInputCloud(table_projected);
                // hull.reconstruct(*hull_points);
                // if (hull_points->points.empty())
                // {
                //     std::cout << "Convex hull computation failed. Table vertices not extracted." << std::endl;
                //     return false;
                // }

                // Eigen::Vector4f centroid;
                // pcl::compute3DCentroid(*cloud, centroid); // hull_points

                // Eigen::Matrix3f covariance;
                // pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance); // hull_points
                // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
                // Eigen::Matrix3f eigVectors = eigen_solver.eigenvectors();

                // Eigen::Vector3f normal = eigVectors.col(0); // Smallest eigenvalue, normal to the plane
                // if (normal.dot(Eigen::Vector3f(0.0, 0.0, 1.0)) < 0)
                //     normal = -normal;
                // Eigen::Vector3f axis1 = eigVectors.col(1); // Largest eigenvalue
                // Eigen::Vector3f axis2 = eigVectors.col(2); // Middle eigenvalue
                // // Ensure right-handed coordinate system
                // axis2 = normal.cross(axis1);
                // axis2.normalize();

                // Eigen::Vector3f major_axis = axis2; // X or Y axis
                // Eigen::Vector3f minor_axis = axis1; // X or Y axis

                // double min_x = std::numeric_limits<double>::max();
                // double max_x = -std::numeric_limits<double>::max();
                // double min_y = std::numeric_limits<double>::max();
                // double max_y = -std::numeric_limits<double>::max();

                // for (const auto &point : hull_points->points)
                // {
                //     // Vector from centroid to point
                //     Eigen::Vector3f vec = point.getVector3fMap() - centroid.head<3>();
                //     // Project onto major and minor axes
                //     double proj_major = vec.dot(major_axis);
                //     double proj_minor = vec.dot(minor_axis); // Assuming minor axis

                //     // Update min and max projections
                //     if (proj_major < min_x)
                //         min_x = proj_major;
                //     if (proj_major > max_x)
                //         max_x = proj_major;
                //     if (proj_minor < min_y)
                //         min_y = proj_minor;
                //     if (proj_minor > max_y)
                //         max_y = proj_minor;
                // }
                double min_x, max_x, min_y, max_y;
                min_x = max_x = cloud->points[0].x;
                min_y = max_y = cloud->points[0].y;

                // Iterate through the point cloud once
                for (const auto &point : cloud->points)
                {
                    // const auto &point = cloud->points[i];

                    // Update min and max for X
                    if (point.x < min_x)
                        min_x = point.x;
                    else if (point.x > max_x)
                        max_x = point.x;

                    // Update min and max for Y
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
                // table_corners[0] = centroid.head<3>() + static_cast<float>(min_x) * major_axis + static_cast<float>(min_y) * minor_axis;
                // table_corners[1] = centroid.head<3>() + static_cast<float>(max_x) * major_axis + static_cast<float>(min_y) * minor_axis;
                // table_corners[2] = centroid.head<3>() + static_cast<float>(max_x) * major_axis + static_cast<float>(max_y) * minor_axis;
                // table_corners[3] = centroid.head<3>() + static_cast<float>(min_x) * major_axis + static_cast<float>(max_y) * minor_axis;

                geometry_msgs::msg::Point point;
                for (const auto &corner : table_corners)
                {
                    point.x = corner.x();
                    point.y = corner.y();
                    point.z = corner.z();
                    vertices.emplace_back(std::move(point));
                }

                // Eigen::Matrix3f rotation;
                // rotation.col(0) = axis1;  // X-axis
                // rotation.col(1) = axis2;  // Y-axis
                // rotation.col(2) = normal; // Z-axis

                // tf2::Matrix3x3 tf_rotation(
                //     rotation(0, 0), rotation(0, 1), rotation(0, 2),
                //     rotation(1, 0), rotation(1, 1), rotation(1, 2),
                //     rotation(2, 0), rotation(2, 1), rotation(2, 2));

                // double roll, pitch, yaw;
                // tf_rotation.getRPY(roll, pitch, yaw);

                // geometry_msgs::msg::Pose pose;
                // // Create a pose message
                // pose.position.x = centroid[0];
                // pose.position.y = centroid[1];
                // pose.position.z = centroid[2];

                // // Convert rotation matrix to quaternion
                // Eigen::Quaternionf quat(rotation);
                // pose.orientation.x = quat.x();
                // pose.orientation.y = quat.y();
                // pose.orientation.z = quat.z();
                // pose.orientation.w = quat.w();
                // vector<geometry_msgs::msg::Pose> poses = {pose};
                // publishMarkerArray(poses);
                return true;
            }
        }
    }

    void PointCloudTransformer::removePlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
    {
        // Remove dominant planes iteratively (floor, walls, table)
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_distance_threshold_);

        bool plane_is_present = true;
        while (plane_is_present)
        {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() < min_plane_inliers_)
            {
                // No more significant planes found
                plane_is_present = false;
                break;
            }

            // Calculate plane normal
            Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            plane_normal.normalize();

            // Determine plane type based on normal and centroid
            bool is_floor = (fabs(plane_normal.dot(Eigen::Vector3f(0, 0, 1))) > 0.95);
            bool is_wall = (fabs(plane_normal.dot(Eigen::Vector3f(1, 0, 0))) > 0.95) || (fabs(plane_normal.dot(Eigen::Vector3f(0, 1, 0))) > 0.95);
            bool is_table = (fabs(plane_normal.dot(Eigen::Vector3f(0, 0, 1))) > 0.95);

            if (is_floor || is_wall || is_table)
            {
                // Remove the plane from the point cloud
                extract.setInputCloud(cloud);
                extract.setIndices(inliers);
                extract.setNegative(true);
                extract.filter(*cloud);
            }
            else
            {
                // Plane does not match floor, wall, or table; stop removing
                plane_is_present = false;
                break;
            }
        }
    }

    void PointCloudTransformer::extractClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
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

    bool PointCloudTransformer::identifyTopFace(const pcl::PointIndices &cluster_indices, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr &top_faces_cloud, geometry_msgs::msg::Pose &box_pose)
    {
        // Extract the cluster point cloud from the input cloud using the provided indices
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract_cluster;
        pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(cluster_indices));
        extract_cluster.setInputCloud(cloud);
        extract_cluster.setIndices(cluster_indices_ptr);
        extract_cluster.setNegative(false);
        extract_cluster.filter(*cluster);

        // Perform plane segmentation to find planar surfaces in the cluster
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_copy(new pcl::PointCloud<pcl::PointXYZRGB>(*cluster));
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::ExtractIndices<pcl::PointXYZRGB> extract_plane;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_distance_threshold_);

        double max_z = -std::numeric_limits<double>::max();

        int num_planes_extracted = 0;
        bool top_face_found = false;
        while (num_planes_extracted < 6)
        {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            seg.setInputCloud(cluster_copy);
            // Ensure cluster_copy has enough points before calling seg.segment
            if (cluster_copy->points.size() < 1) // min_required_points_for_RANSAC_
            {
                RCLCPP_WARN(this->get_logger(), "Cluster has too few points for plane segmentation.");
                ransac_performed_successfully_ = true;
                voxel_grid_leaf_size_ /= 2.0;
                min_plane_inliers_ *= 2;
                min_cluster_size_ *= 2;
                max_cluster_size_ *= 2;
                return false;
            }
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.empty())
                break;

            // Calculate plane normal
            Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            plane_normal.normalize();

            // Remove the extracted plane from the cluster copy
            // cluster_copy has pc of top face if horizonzal plane detected and has everything except
            // detected plane if the detected plane is non horizontal
            extract_plane.setInputCloud(cluster_copy);
            extract_plane.setIndices(inliers);
            extract_plane.setNegative((fabs(plane_normal.dot(Eigen::Vector3f(0, 0, 1))) > 0.95) ? false : true);
            extract_plane.filter(*cluster_copy);
            num_planes_extracted++;
            if (fabs(plane_normal.dot(Eigen::Vector3f(0, 0, 1))) > 0.95)
            {
                top_face_found = true;
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cluster_copy, centroid); // top_face_cloud
                Eigen::Matrix3f covariance;
                pcl::computeCovarianceMatrixNormalized(*cluster_copy, centroid, covariance); // top_face_cloud
                // Perform PCA (Eigen Decomposition)
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
                Eigen::Matrix3f eig_vectors = eigen_solver.eigenvectors();
                // Assign axes correctly
                Eigen::Vector3f normal = eig_vectors.col(0); // Smallest eigenvalue, normal to the plane
                if (normal.dot(Eigen::Vector3f(0.0, 0.0, 1.0)) < 0)
                    normal = -normal;
                Eigen::Vector3f axis1 = eig_vectors.col(1); // Largest eigenvalue
                Eigen::Vector3f axis2 = eig_vectors.col(2); // Middle eigenvalue
                // Ensure right-handed coordinate system
                axis2 = normal.cross(axis1);
                axis2.normalize();
                // Construct rotation matrix
                Eigen::Matrix3f rotation;
                rotation.col(0) = axis1;  // X-axis
                rotation.col(1) = axis2;  // Y-axis
                rotation.col(2) = normal; // Z-axis

                // Create a pose message
                box_pose.position.x = centroid[0];
                box_pose.position.y = centroid[1];
                box_pose.position.z = centroid[2];

                // Convert rotation matrix to quaternion
                Eigen::Quaternionf quat(rotation);
                box_pose.orientation.x = quat.x();
                box_pose.orientation.y = quat.y();
                box_pose.orientation.z = quat.z();
                box_pose.orientation.w = quat.w();
                break;
            }
        }
        *top_faces_cloud += *cluster_copy;
        return top_face_found;
    }

    void PointCloudTransformer::publishMarkerArray(const std::vector<geometry_msgs::msg::Pose> &box_poses)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        int current_marker_id = 0;
        for (const auto &pose : box_poses)
        {
            // Convert quaternion to rotation matrix
            Eigen::Quaternionf quat(pose.orientation.w,
                                    pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z);
            Eigen::Matrix3f rot_matrix = quat.toRotationMatrix();

            // Origin of the vectors
            geometry_msgs::msg::Point origin = pose.position;

            // Define vectors based on rotation matrix columns
            geometry_msgs::msg::Vector3 x_vector;
            x_vector.x = rot_matrix(0, 0);
            x_vector.y = rot_matrix(1, 0);
            x_vector.z = rot_matrix(2, 0);

            geometry_msgs::msg::Vector3 y_vector;
            y_vector.x = rot_matrix(0, 1);
            y_vector.y = rot_matrix(1, 1);
            y_vector.z = rot_matrix(2, 1);

            geometry_msgs::msg::Vector3 z_vector;
            z_vector.x = rot_matrix(0, 2);
            z_vector.y = rot_matrix(1, 2);
            z_vector.z = rot_matrix(2, 2);

            // X-axis (Red)
            visualizeVector(origin, x_vector, current_marker_id++, marker_array, 1.0f, 0.0f, 0.0f);

            // Y-axis (Green)
            visualizeVector(origin, y_vector, current_marker_id++, marker_array, 0.0f, 1.0f, 0.0f);

            // Z-axis (Blue)
            visualizeVector(origin, z_vector, current_marker_id++, marker_array, 0.0f, 0.0f, 1.0f);
        }

        // Publish the MarkerArray
        marker_pub_->publish(marker_array);
    }

    void PointCloudTransformer::visualizeVector(const geometry_msgs::msg::Point &origin, const geometry_msgs::msg::Vector3 &vector, int id,
                                                visualization_msgs::msg::MarkerArray &marker_array, float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = target_frame_;
        marker.header.stamp = this->now();
        marker.ns = "vectors";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.points.resize(2);
        marker.points[0] = origin;
        marker.points[1].x = origin.x + vector.x * 0.4; // Scale for visualization
        marker.points[1].y = origin.y + vector.y * 0.4;
        marker.points[1].z = origin.z + vector.z * 0.4;

        marker.scale.x = 0.05; // Shaft diameter
        marker.scale.y = 0.1;  // Head diameter
        marker.scale.z = 0.1;  // Head length

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration(0, 0); // 0 means marker never auto-deletes
        marker_array.markers.push_back(marker);
    }

    void PointCloudTransformer::publishTableVertices(const std::vector<geometry_msgs::msg::Point> &vertices)
    {
        visualization_msgs::msg::Marker table_marker;
        table_marker.header.frame_id = target_frame_;
        table_marker.header.stamp = this->now();
        table_marker.ns = "table_vertices";
        table_marker.id = 0; // Unique ID for this marker
        table_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        table_marker.action = visualization_msgs::msg::Marker::ADD;

        table_marker.points.resize(5);
        for (size_t i = 0; i < 4; ++i)
            table_marker.points[i] = vertices[i];
        table_marker.points[4] = vertices[0]; // Close the loop

        // Set the scale (line width)
        table_marker.scale.x = 0.02; // 2 cm line width

        // Set the color (e.g., Blue)
        table_marker.color.r = 0.0f;
        table_marker.color.g = 0.0f;
        table_marker.color.b = 1.0f;
        table_marker.color.a = 1.0f; // Fully opaque

        // Optional: Set lifetime (0 means marker persists until overwritten or deleted)
        table_marker.lifetime = rclcpp::Duration(0, 0);

        // Create a MarkerArray and add the table marker
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(table_marker);

        // Publish the MarkerArray
        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published table vertices as a LINE_STRIP marker.");
    }

    rcl_interfaces::msg::SetParametersResult PointCloudTransformer::StartPC_ProcessingCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        cout << "parameter callback called" << endl;
        rcl_interfaces::msg::SetParametersResult result;

        for (const auto &param : parameters)
        {
            if (param.get_name() == "pc_processing_param")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
                {
                    sub_counter = 0;
                    voxel_grid_leaf_size_ = 0.01;
                    min_plane_inliers_ = 800;
                    min_cluster_size_ = 30;
                    max_cluster_size_ = 1000;
                    start_processing_ = param.as_string();

                    RCLCPP_INFO(this->get_logger(), "pc_processing_param parameter updated to: '%s'", start_processing_.c_str());
                    ransac_performed_successfully_ = false;
                    if (start_processing_ == "start_processing")
                        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                            "/arm_rgbd_camera/points", rclcpp::QoS(10),
                            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
                    else if (start_processing_ == "detect_table")
                        table_detection_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                            "/arm_rgbd_camera/points", rclcpp::QoS(10),
                            std::bind(&PointCloudTransformer::pointCloud_TableDetectionCallback, this, std::placeholders::_1));
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "pc_processing_param parameter has incorrect type. Expected string.");
                    result.successful = false;
                    result.reason = "pc_processing_param parameter must be a string.";
                    return result;
                }
                break;
            }
        }

        result.successful = true;
        result.reason = "Parameters set successfully.";
        return result;
    }
}
