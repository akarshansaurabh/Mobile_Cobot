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
          roi_max_x_(10.0f),
          roi_min_y_(-10.0f),
          roi_max_y_(10.0f),
          roi_min_z_(0.4f),
          roi_max_z_(1.5),
          plane_distance_threshold_(0.01),
          min_plane_inliers_(800),
          ransac_success_(false),
          min_cluster_size_(50),
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
            "/arm_rgbd_camera/processed_pc", rclcpp::QoS(10));
        move_amr_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/move_amr_topic", rclcpp::QoS(10));
        box_poses_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/box_poses_topic", rclcpp::QoS(10));
        table_vertices_pub = this->create_publisher<geometry_msgs::msg::Polygon>(
            "/table_vertices_topic", rclcpp::QoS(10));

        box6dposes_server_ = this->create_service<custom_interfaces::srv::BoxposeEstimator>(
            "six_d_pose_estimate_service",
            std::bind(&PointCloudProcessor::Server6DPoseCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Declare parameters
        this->declare_parameter("pc_processing_param", "stop");

        way_point_generator_ = std::make_shared<waypointGen::TableWayPointGen>();

        nav2_actionclient_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "nav2_client3_node");
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

    void PointCloudProcessor::ResendTableAsDestination()
    {
        if (!nav2_actionclient_param_client_->wait_for_service(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "destination parameter service not available.");
            return;
        }

        // vector of params of target node that i want to change
        std::vector<rclcpp::Parameter> params = {rclcpp::Parameter("destination", "table_A")};
        auto future = nav2_actionclient_param_client_->set_parameters(params);

        std::thread([this, future = std::move(future), params]() mutable
                    {
        try
        {
            auto results = future.get();
            for (size_t i = 0; i < results.size(); ++i)
            {
                if (!results[i].successful)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set parameter '%s': %s",
                                 params[i].get_name().c_str(), results[i].reason.c_str());
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Successfully set parameter '%s'",
                                params[i].get_name().c_str());
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while setting parameters: %s", e.what());
        } })
            .detach();
    }

    void PointCloudProcessor::Server6DPoseCallback(const std::shared_ptr<custom_interfaces::srv::BoxposeEstimator::Request> request,
                                                   std::shared_ptr<custom_interfaces::srv::BoxposeEstimator::Response> response)
    {
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg(request->cloud, *pcl_cloud);

        // Preprocess the point cloud
        preprocessPointCloud(pcl_cloud, false);

        // Transform point cloud to target frame
        if (!transformPointCloudToTargetFrame(pcl_cloud, request->cloud.header))
            return;

        // Filter points within ROI
        filterPointCloudInROI(pcl_cloud, request->cloud.header, table_vertices, min_x, max_x, min_y, max_y);
        if (pcl_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points remain after ROI filtering.");
            return;
        }

        // Extract clusters
        std::vector<pcl::PointIndices> cluster_indices;
        min_cluster_size_ = 5;
        max_cluster_size_ = 1000;
        cluster_tolerance_ = 0.01;
        extractClusters(pcl_cloud, cluster_indices);
        min_cluster_size_ = 50;
        max_cluster_size_ = 1000;
        cluster_tolerance_ = 0.07;

        // Identify top faces
        bool box_found = false;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr top_faces(new pcl::PointCloud<pcl::PointXYZRGB>());
        std::vector<geometry_msgs::msg::Pose> box_poses;
        box_poses.reserve(cluster_indices.size());

        std::vector<std::future<std::tuple<bool, geometry_msgs::msg::Pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>> futures;
        for (auto indices : cluster_indices)
        {
            futures.push_back(std::async(std::launch::async, [this, pcl_cloud, indices]()
                                         {
            geometry_msgs::msg::Pose box_pose;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_top_faces(new pcl::PointCloud<pcl::PointXYZRGB>());

            bool found = ComputeBox4DPose(indices, pcl_cloud, local_top_faces, box_pose);
            return std::make_tuple(found, box_pose, local_top_faces); }));
        }
        for (auto &fut : futures)
        {
            // Wait for the thread to finish and get the result
            auto result = fut.get();
            bool found = std::get<0>(result);
            geometry_msgs::msg::Pose pose = std::get<1>(result);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_top_faces = std::get<2>(result);

            if (found)
            {
                box_found = true;
                box_poses.push_back(pose);
                *top_faces += *local_top_faces;
            }
        }

        if (box_found)
        {
            geometry_msgs::msg::PoseArray box_poses_msg;
            for (const auto &pose : box_poses)
                box_poses_msg.poses.push_back(std::move(pose));
            box_poses_pub->publish(box_poses_msg);

            RCLCPP_INFO(this->get_logger(), "Box found in the point cloud.");
            pcl_cloud.swap(top_faces);
            vis_manager_->publishMarkerArray(box_poses);

            // Publish the processed point cloud
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*pcl_cloud, output_msg);
            output_msg.header.stamp = request->cloud.header.stamp;
            output_msg.header.frame_id = target_frame_;
            pointcloud_pub_->publish(output_msg);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Box not found in the point cloud.");
        }
    }

    void PointCloudProcessor::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, bool is_table)
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

        if (is_table)
        {
            std::vector<pcl::PointIndices> cluster_indices;
            min_cluster_size_ = 10000;
            max_cluster_size_ = 400000;
            cluster_tolerance_ = 0.1;
            extractClusters(cloud, cluster_indices);
            min_cluster_size_ = 50;
            max_cluster_size_ = 1000;
            cluster_tolerance_ = 0.07;
            pcl::ExtractIndices<pcl::PointXYZRGB> extract_cluster;
            pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(cluster_indices[0]));
            extract_cluster.setInputCloud(cloud);
            extract_cluster.setIndices(cluster_indices_ptr);
            extract_cluster.setNegative(false);
            extract_cluster.filter(*cloud);
        }
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

    void PointCloudProcessor::filterPointCloudInROI(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const std_msgs::msg::Header &header,
                                                    const std::vector<geometry_msgs::msg::Point> &table_vertices, double min_x, double max_x, double min_y, double max_y)
    {
        // Use CropBox filter
        pcl::CropBox<pcl::PointXYZRGB> crop_filter;
        crop_filter.setInputCloud(cloud);

        Eigen::Vector4f min_point(min_x, min_y, table_vertices[0].z + 0.05, 1.0f);
        Eigen::Vector4f max_point(max_x - 0.07, max_y, table_vertices[0].z + 0.3, 1.0f);

        crop_filter.setMin(min_point);
        crop_filter.setMax(max_point);
        crop_filter.filter(*cloud);
    }

    // not in use
    void PointCloudProcessor::removeNormalPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
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

            // Determine mormal planes
            bool is_vertical = (fabs(plane_normal.dot(Eigen::Vector3f(1, 0, 0))) > 0.95) || (fabs(plane_normal.dot(Eigen::Vector3f(0, 1, 0))) > 0.95);

            if (is_vertical)
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
        cout << "number of eclidean cluster = " << cluster_indices.size();
        // for (const auto &cluster : cluster_indices)
        //     std::cout << "cluster points " << cluster.indices.size() << std::endl;
        // std::cout << "___________________________________" << std::endl;
        if (cluster_indices.size() == 0)
        {
            // DBSCAN Clustering
            pcl::DBSCAN<pcl::PointXYZRGB> dbscan;
            dbscan.setEps(0.02); // Radius to look for neighbors in, adjust this
            dbscan.setMinPts(5); // Minimum number of neighbors to be a core point, adjust this
            dbscan.setSearchMethod(tree);
            dbscan.setInputCloud(cloud);
            dbscan.extract(cluster_indices);
            cout << "number of DBSCAN cluster = " << cluster_indices.size();
        }
    }

    void PointCloudProcessor::pointCloudTableDetectionCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
    {
        table_detection_counter_++;
        cout << "table_detection_counter_ " << table_detection_counter_ << endl;

        // Convert ROS PointCloud2 message to PCL PointCloud
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Preprocess the point cloud
        preprocessPointCloud(pcl_cloud, true);

        // Transform point cloud to target frame
        if (!transformPointCloudToTargetFrame(pcl_cloud, msg->header))
        {
            table_detection_sub_.reset();
            return;
        }

        // Detect table

        if (tableIsPresent(pcl_cloud, table_vertices, msg->header.stamp))
        {
            RCLCPP_INFO(this->get_logger(), "Table detected.");
            vis_manager_->publishTableVertices(table_vertices);
            // compute the distance and send it to client
            if (table_detection_counter_ == 1)
            {
                // calculate forward desired dis robot should based on its current position and table vertices
                geometry_msgs::msg::Point robot_position;
                geometry_msgs::msg::TransformStamped transform_stamped;
                try
                {
                    transform_stamped =
                        tf_buffer_->lookupTransform(target_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
                    return;
                }
                robot_position.x = transform_stamped.transform.translation.x;
                robot_position.y = transform_stamped.transform.translation.y;
                robot_position.z = transform_stamped.transform.translation.z;
                geometry_msgs::msg::Vector3 msg_ = way_point_generator_->ComputeDesiredForwardDistance(table_vertices, robot_position);
                move_amr_pub_->publish(msg_);

                // sort table vertices in anti-clockwsie with 1st point having x_min,y_max
                auto sortTableVerticesAntiClockwise = [this]()
                {
                    if (table_vertices.size() != 4)
                        std::cerr << "Error: table_vertices must contain exactly 4 poses." << std::endl;
                    else
                    {
                        // Step 1: Find the starting vertex with min x and max y
                        int starting_index = -1;
                        double min_x = std::numeric_limits<double>::infinity();
                        double max_y = -std::numeric_limits<double>::infinity();

                        for (int i = 0; i < table_vertices.size(); ++i)
                        {
                            double x = table_vertices[i].x;
                            double y = table_vertices[i].y;
                            if (x < min_x || (fabs(x - min_x) < 0.001 && y > max_y))
                            {
                                min_x = x;
                                max_y = y;
                                starting_index = i;
                            }
                        }
                        std::cout << "A " << table_vertices[starting_index].x << " "
                                  << table_vertices[starting_index].y << " "
                                  << table_vertices[starting_index].z << " " << std::endl;

                        if (starting_index == -1)
                            std::cerr << "Error: Unable to find starting vertex." << std::endl;
                        else
                        {
                            // Step 2: Compute the centroid of all vertices
                            double x_centroid = 0.0;
                            double y_centroid = 0.0;
                            for (const auto &point_xyz : table_vertices)
                            {
                                x_centroid += point_xyz.x;
                                y_centroid += point_xyz.y;
                            }
                            x_centroid /= table_vertices.size();
                            y_centroid /= table_vertices.size();

                            // Step 3: Compute angles and create a vector of VertexAngle
                            std::vector<std::pair<int, double>> vertex_angles;
                            vertex_angles.reserve(table_vertices.size());

                            // Reference angle using starting vertex
                            double dx_start = table_vertices[starting_index].x - x_centroid;
                            double dy_start = table_vertices[starting_index].y - y_centroid;
                            double angle_start = std::atan2(dy_start, dx_start); // Reference angle

                            for (int i = 0; i < table_vertices.size(); ++i)
                            {
                                double dx = table_vertices[i].x - x_centroid;
                                double dy = table_vertices[i].y - y_centroid;
                                double angle = std::atan2(dy, dx); // Angle relative to centroid
                                double adjusted_angle = angle - angle_start;
                                // Normalize the adjusted angle to [0, 2*pi)
                                if (adjusted_angle < 0)
                                    adjusted_angle += 2 * M_PI;
                                vertex_angles.push_back(make_pair(i, adjusted_angle));
                            }

                            // Step 4: Sort the vector based on angle in ascending order
                            std::sort(vertex_angles.begin(), vertex_angles.end(),
                                      [](const std::pair<int, double> &a, const std::pair<int, double> &b) -> bool
                                      {
                                          return a.second < b.second;
                                      });

                            // Step 5: Create a sorted PoseArray based on sorted indices
                            std::vector<geometry_msgs::msg::Point> sorted_points;
                            sorted_points.reserve(table_vertices.size());

                            for (const auto &va : vertex_angles)
                            {
                                sorted_points.push_back(table_vertices[va.first]);
                            }
                            table_vertices = sorted_points;
                        }
                    }
                };
                sortTableVerticesAntiClockwise();
                geometry_msgs::msg::Polygon table_msg;
                geometry_msgs::msg::Point32 xyz_32;
                for (const auto &v : table_vertices)
                {
                    xyz_32.x = v.x;
                    xyz_32.y = v.y;
                    xyz_32.z = v.z;
                    table_msg.points.push_back(xyz_32);
                }
                table_vertices_pub->publish(table_msg);
            }
        }
        else
        {
            cout << "table could not be deteted" << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (table_detection_counter_ == 3)
                table_detection_sub_.reset();
            if (table_detection_counter_ < 3)
                ResendTableAsDestination();
        }

        // Reset the subscriber after processing
        if (table_detection_counter_ == 1)
            table_detection_sub_.reset();
    }

    bool PointCloudProcessor::tableIsPresent(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<geometry_msgs::msg::Point> &vertices, const rclcpp::Time &timestamp)
    {
        vertices.clear();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*cloud));
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setKeepOrganized(false);

        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_distance_threshold_);

        double slicing_avg_height = 0.0;

        while (true)
        {
            if (cloud->points.empty())
            {
                return false;
            }

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0)
            {
                return false;
            }

            if (inliers->indices.size() < min_plane_inliers_) // less than 800
            {
                return false;
            }

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
                // double min_x, max_x, min_y, max_y;
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

    // not in use
    bool PointCloudProcessor::computeOBBForCluster(const pcl::PointIndices &cluster_indices,
                                                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                                   geometry_msgs::msg::Pose &box_pose)
    {
        // Extract cluster cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        {
            pcl::ExtractIndices<pcl::PointXYZRGB> extract_cluster;
            pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(cluster_indices));
            extract_cluster.setInputCloud(cloud);
            extract_cluster.setIndices(cluster_indices_ptr);
            extract_cluster.setNegative(false);
            extract_cluster.filter(*cluster_cloud);
        }

        if (cluster_cloud->empty())
        {
            return false;
        }

        pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
        feature_extractor.setInputCloud(cluster_cloud);
        feature_extractor.compute();

        std::vector<float> moment_of_inertia;
        std::vector<float> eccentricity;
        pcl::PointXYZRGB min_point_AABB;
        pcl::PointXYZRGB max_point_AABB;
        pcl::PointXYZRGB min_point_OBB;
        pcl::PointXYZRGB max_point_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        pcl::PointXYZRGB position_OBB;

        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        feature_extractor.getAABB(min_point_AABB, max_point_AABB);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

        // Convert rotation matrix to quaternion
        Eigen::Quaternionf quat(rotational_matrix_OBB);
        Eigen::Vector3f box_up = rotational_matrix_OBB.col(2);
        Eigen::Vector3f table_up(0.0, 0.0, 1.0);
        Eigen::Quaternionf rotation_needed = Eigen::Quaternionf::FromTwoVectors(box_up, table_up);
        Eigen::Quaternionf adjusted_quat = rotation_needed * quat;

        // Fill box_pose
        box_pose.position.x = position_OBB.x;
        box_pose.position.y = position_OBB.y;
        box_pose.position.z = position_OBB.z;
        box_pose.orientation.x = adjusted_quat.x();
        box_pose.orientation.y = adjusted_quat.y();
        box_pose.orientation.z = adjusted_quat.z();
        box_pose.orientation.w = adjusted_quat.w();

        // We consider we always find a box. If you had a condition, you could check size or shape.
        return true;
    }

    bool PointCloudProcessor::ComputeBox4DPose(const pcl::PointIndices &cluster_indices, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &top_faces_cloud, geometry_msgs::msg::Pose &box_pose)
    {
        // Extract the cluster point cloud from the input cloud using the provided indices
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr horizontal_plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        bool top_face_found = false, is_horizontal = false;
        horizontal_plane_cloud->points.clear();
        Eigen::Vector3f normal1(0.0, 0.0, 0.0), normal2(0.0, 0.0, 0.0);
        Eigen::Vector4f centroid;

        pcl::ExtractIndices<pcl::PointXYZRGB> extract_cluster;
        pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(cluster_indices));
        extract_cluster.setInputCloud(cloud);
        extract_cluster.setIndices(cluster_indices_ptr);
        extract_cluster.setNegative(false);
        extract_cluster.filter(*cluster_cloud);

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
            RCLCPP_INFO(this->get_logger(), "RANSAC STARTED.");
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0)
            {
                RCLCPP_ERROR(this->get_logger(), "RANSAC FAILED BECAUSE OF LESS POINTS");
                break;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "RANSAC SUCCESSFUL");
            }

            // Calculate plane normal
            Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            plane_normal.normalize();

            is_horizontal = (fabs(plane_normal.dot(Eigen::Vector3f(0, 0, 1))) > 0.95);

            if (is_horizontal && !top_face_found)
            {
                // Extract the plane points
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
                    top_face_found = true;
                    normal1 = plane_normal;
                    pcl::compute3DCentroid(*horizontal_plane_cloud, centroid);
                    *top_faces_cloud += *horizontal_plane_cloud;
                }
            }

            if (!is_horizontal)
            {
                // slant
                if (normal2.norm() < 0.01)
                {
                    normal2 = plane_normal;
                }
            }
            if (normal2.norm() > 0.01 && normal1.norm() > 0.01)
            {
                Eigen::Vector3f z_cap(0.0, 0.0, 1.0);
                Eigen::Vector3f y_cap = normal1.cross(normal2);
                y_cap.normalize();
                Eigen::Vector3f x_cap = y_cap.cross(z_cap);
                x_cap.normalize();

                Eigen::Matrix3f rotation;
                rotation.col(0) = x_cap;
                rotation.col(1) = y_cap;
                rotation.col(2) = z_cap;

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
                return true;
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

    bool PointCloudProcessor::ComputeBox6DPose(const pcl::PointIndices &cluster_indices, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &top_faces_cloud, geometry_msgs::msg::Pose &box_pose)
    {
        // Extract cluster point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract_cluster;
        pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(cluster_indices));
        extract_cluster.setInputCloud(cloud);
        extract_cluster.setIndices(cluster_indices_ptr);
        extract_cluster.setNegative(false);
        extract_cluster.filter(*cluster_cloud);

        if (cluster_cloud->empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Input cluster cloud is empty.");
            return false;
        }

        // Plane segmentation setup
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        pcl::ExtractIndices<pcl::PointXYZRGB> extract_plane;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_distance_threshold_); // Use member variable

        std::vector<pcl::ModelCoefficients::Ptr> plane_coefficients;
        std::vector<pcl::PointIndices::Ptr> plane_inliers;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*cluster_cloud));

        int num_planes_extracted = 0;
        while (num_planes_extracted < 3)
        {
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            seg.setInputCloud(remaining_cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0)
            {
                RCLCPP_INFO(this->get_logger(), "RANSAC could not find more planes or no more points left.");
                break; // No more planes found
            }

            plane_coefficients.push_back(coefficients);
            plane_inliers.push_back(inliers);

            // Extract plane points and remove from remaining cloud for next iteration
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            extract_plane.setInputCloud(remaining_cloud);
            extract_plane.setIndices(inliers);
            extract_plane.setNegative(false);
            extract_plane.filter(*extracted_plane_cloud);

            extract_plane.setInputCloud(remaining_cloud);
            extract_plane.setIndices(inliers);
            extract_plane.setNegative(true);
            extract_plane.filter(*remaining_cloud);

            num_planes_extracted++;
        }

        if (plane_coefficients.size() < 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Found less than 3 planes, cannot compute 6D pose reliably.");
            return false;
        }

        std::vector<Eigen::Vector3f> plane_normals;
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> plane_point_clouds;
        std::vector<Eigen::Vector4f> plane_centroids;
        std::vector<double> avg_z_values;

        for (size_t i = 0; i < plane_coefficients.size(); ++i)
        {
            Eigen::Vector3f normal(plane_coefficients[i]->values[0], plane_coefficients[i]->values[1], plane_coefficients[i]->values[2]);
            normal.normalize();
            plane_normals.push_back(normal);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            extract_plane.setInputCloud(cluster_cloud); // Use original cluster cloud, and filter by inliers of each plane
            extract_plane.setIndices(plane_inliers[i]);
            extract_plane.setNegative(false);
            extract_plane.filter(*current_plane_cloud);
            plane_point_clouds.push_back(current_plane_cloud);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*current_plane_cloud, centroid);
            plane_centroids.push_back(centroid);

            double sum_z = 0.0;
            for (const auto &point : current_plane_cloud->points)
                sum_z += point.z;
            double avg_z = sum_z / current_plane_cloud->points.size();
            avg_z_values.push_back(avg_z);
        }

        // Identify top face (plane with max avg_z)
        int top_face_index = 0;
        double max_avg_z = -std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < avg_z_values.size(); ++i)
        {
            if (avg_z_values[i] > max_avg_z)
            {
                max_avg_z = avg_z_values[i];
                top_face_index = i;
            }
        }

        Eigen::Vector3f n_top = plane_normals[top_face_index];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr P_top = plane_point_clouds[top_face_index];
        Eigen::Vector4f centroid_top_face = plane_centroids[top_face_index];

        int plane2_index = -1, plane3_index = -1;
        int current_index = 0;
        for (size_t i = 0; i < plane_normals.size(); ++i)
        {
            if (i != top_face_index)
            {
                if (plane2_index == -1)
                    plane2_index = i;
                else if (plane3_index == -1)
                    plane3_index = i;
                current_index++;
            }
        }

        Eigen::Vector3f n2_ = plane_normals[plane2_index];
        Eigen::Vector3f n3_ = plane_normals[plane3_index];

        Eigen::Vector3f x_cap = n_top.cross(n2_);
        x_cap.normalize();
        Eigen::Vector3f y_cap = n_top.cross(n3_);
        y_cap.normalize();
        Eigen::Vector3f z_cap = x_cap.cross(y_cap);
        z_cap.normalize();

        Eigen::Matrix3f rotation;
        rotation.col(0) = x_cap;
        rotation.col(1) = y_cap;
        rotation.col(2) = z_cap;

        Eigen::Quaternionf quat(rotation);

        // Fill box_pose
        box_pose.position.x = centroid_top_face[0];
        box_pose.position.y = centroid_top_face[1];
        box_pose.position.z = centroid_top_face[2];
        box_pose.orientation.x = quat.x();
        box_pose.orientation.y = quat.y();
        box_pose.orientation.z = quat.z();
        box_pose.orientation.w = quat.w();

        *top_faces_cloud = *P_top; // Output top face cloud
        return true;
    }
}
