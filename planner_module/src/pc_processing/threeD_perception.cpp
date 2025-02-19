#include "pc_processing/threeD_perception.hpp"

namespace environment3DPerception
{
    Environment3DPerception::Environment3DPerception(rclcpp::Node::SharedPtr node)
        : node_(node),
          front_sub_(nullptr),
          arm_sub_(nullptr),
          front_camera_received_(false),
          arm_camera_received_(false)
    {
        environment_3d_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        octomap_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/octomap_topic_", rclcpp::QoS(10));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        node_->declare_parameter<std::string>("activate_3d_perception", "F");
        node_->get_parameter("activate_3d_perception", activate_3d_perception_);

        parameter_callback_handle_ = node_->add_on_set_parameters_callback(std::bind(&Environment3DPerception::onParameterEvent, this, std::placeholders::_1));

        if (activate_3d_perception_ == "T")
        {
            RCLCPP_INFO(node_->get_logger(), "activate_3d_perception is true at startup. Creating timer.");
            timer_ = node_->create_wall_timer(std::chrono::seconds(2),
                                              std::bind(&Environment3DPerception::timerCallback, this));
        }
        else
            RCLCPP_INFO(node_->get_logger(), "activate_3d_perception is false at startup. No timer created.");
    }

    void Environment3DPerception::traverseSegmentationTree(const std::shared_ptr<SegmentTreeNode> &node, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_pcl_cloud)
    {
        if (!node)
        {
            std::cout << "NULL CHILD = 3" << std::endl;
            return;
        }
        *segmented_pcl_cloud += *node->object_cloud;
        for (auto &child : node->children)
            traverseSegmentationTree(child, segmented_pcl_cloud);
    }

    rcl_interfaces::msg::SetParametersResult Environment3DPerception::onParameterEvent(const std::vector<rclcpp::Parameter> &parameters)
    {
        std::cout << "param callback called" << std::endl;
        for (auto &param : parameters)
            if (param.get_name() == "activate_3d_perception")
            {
                activate_3d_perception_ = param.as_string();
                RCLCPP_INFO(node_->get_logger(),
                            "Parameter 'activate_3d_perception' changed to: %s",
                            activate_3d_perception_.c_str());

                if (activate_3d_perception_ == "T")
                {
                    if (!timer_)
                    {
                        RCLCPP_INFO(node_->get_logger(), "Creating timer because param turned true.");
                        timer_ = node_->create_wall_timer(std::chrono::seconds(2),
                                                          std::bind(&Environment3DPerception::timerCallback, this));
                    }
                }
                else if (activate_3d_perception_ == "F")
                {
                    // Stop the timer
                    if (timer_)
                    {
                        timer_.reset();
                        RCLCPP_INFO(node_->get_logger(), "Timer stopped because param turned false.");
                    }
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "Invalid Value.");
                }
            }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void Environment3DPerception::timerCallback()
    {
        // Only create subscribers if param is true
        if (activate_3d_perception_ == "F")
            return;

        if (!front_sub_ && !arm_sub_)
        {
            RCLCPP_INFO(node_->get_logger(), "Creating subscribers for front and arm cameras...");

            front_camera_received_.store(false);
            arm_camera_received_.store(false);

            front_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/front_rgbd_camera/points", rclcpp::QoS(10),
                std::bind(&Environment3DPerception::frontCameraCallback, this, std::placeholders::_1));

            arm_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/arm_rgbd_camera/points", rclcpp::QoS(10),
                std::bind(&Environment3DPerception::armCameraCallback, this, std::placeholders::_1));

            RCLCPP_INFO(node_->get_logger(), "Subscribers created. Waiting for pointcloud messages...");
        }
        else
        {
            if (front_camera_received_.load() && arm_camera_received_.load())
            {
                RCLCPP_INFO(node_->get_logger(), "Both camera callbacks fired. Unsubscribing and post-processing.");

                front_sub_.reset();
                arm_sub_.reset();

                postProcessing();

                RCLCPP_INFO(node_->get_logger(), "Post-processing complete. Next cycle in 2s...");
            }
            else
                RCLCPP_INFO(node_->get_logger(), "Waiting for both camera callbacks...");
        }
    }

    void Environment3DPerception::frontCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!front_camera_received_.load())
        {
            // Preprocess + transform
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = preprocessAndTransform(msg);
            if (processed_cloud)
            {
                std::lock_guard<std::mutex> lk(cloud_mutex_);
                *environment_3d_pointcloud_ += *processed_cloud;
            }

            front_camera_received_.store(true);
        }
    }

    void Environment3DPerception::armCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!arm_camera_received_.load())
        {
            // Preprocess + transform
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud = preprocessAndTransform(msg);
            if (processed_cloud)
            {
                std::lock_guard<std::mutex> lk(cloud_mutex_);
                *environment_3d_pointcloud_ += *processed_cloud;
            }

            arm_camera_received_.store(true);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    Environment3DPerception::preprocessAndTransform(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
    {
        // Convert ROS->PCL
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *raw_cloud);

        // Downsample with VoxelGrid
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(raw_cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        vg.filter(*downsampled);

        // Remove outliers with SOR
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(downsampled);
        sor.setMeanK(30);
        sor.setStddevMulThresh(1.0);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        sor.filter(*filtered);

        // Transform to "map" frame
        if (!transformToMapFrame(filtered, msg->header.frame_id))
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to transform from frame '%s' to 'map'.",
                        msg->header.frame_id.c_str());
            return nullptr;
        }

        return filtered;
    }

    bool Environment3DPerception::transformToMapFrame(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in_out,
        const std::string &from_frame)
    {
        // Lookup transform from from_frame -> "map"
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform(
                "map", from_frame, tf2::TimePointZero, tf2::durationFromSec(0.1)); // or use msg->header.stamp if you have exact sync
            Eigen::Affine3d transform = tf2::transformToEigen(transform_stamped.transform);
            Eigen::Matrix4f transform_matrix = transform.cast<float>().matrix();
            pcl::transformPointCloud(*cloud_in_out, *cloud_in_out, transform_matrix);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
            return false;
        }
    }

    void Environment3DPerception::postProcessing()
    {
        // Lock environment_3d_pointcloud_ while we refine
        std::lock_guard<std::mutex> lk(cloud_mutex_);

        if (environment_3d_pointcloud_->empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Environment 3D pointcloud is empty. Nothing to post-process.");
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "Post-processing environment_3d_pointcloud with size=%lu",
                    environment_3d_pointcloud_->size());

        // Downsample again
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(environment_3d_pointcloud_);
        vg.setLeafSize(0.015f, 0.015f, 0.015f);
        vg.filter(*environment_3d_pointcloud_);

        // 3) Build the segmentation tree
        auto root = Construct3DScene_SegmentationTree(environment_3d_pointcloud_);

        // Convert PCL->ROS
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        traverseSegmentationTree(root, segmented_pcl_cloud);

        pcl::toROSMsg(*segmented_pcl_cloud, ros_cloud);
        ros_cloud.header.frame_id = "map";
        ros_cloud.header.stamp = node_->now();

        // Publish
        octomap_pub_->publish(ros_cloud);
        RCLCPP_INFO(node_->get_logger(), "Published merged environment pointcloud on /octomap_topic_.");

        // Clear the environment_3d_pointcloud_ so next iteration starts fresh
        environment_3d_pointcloud_->clear();
        segmented_pcl_cloud->clear();
        RCLCPP_INFO(node_->get_logger(), "Cleared environment_3d_pointcloud_ after publishing.");
    }

    std::shared_ptr<SegmentTreeNode> Environment3DPerception::Construct3DScene_SegmentationTree(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud)
    {
        // Create the root node
        auto root = std::make_shared<SegmentTreeNode>();
        auto null_child = std::make_shared<SegmentTreeNode>();
        null_child = nullptr;
        root->object_cloud = input_cloud;
        root->object_type = PerceptionObject::ENTIRE_3D_SCENE;

        // Make a copy we can modify
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*input_cloud));

        // 1) Extract all vertical planes => "walls"
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr walls_cloud = extractAllVerticalPlanes(working_cloud, /*min_inliers=*/1000);

        auto walls_node = std::make_shared<SegmentTreeNode>();
        walls_node->children.push_back(null_child);

        walls_node->object_cloud = walls_cloud;
        walls_node->object_type = PerceptionObject::WALL;
        root->children.push_back(walls_node);

        // 2) Extract a floor plane => "floor" (assuming only one floor)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cloud = extractFloorPlane(working_cloud, /*min_inliers=*/200);

        auto floor_node = std::make_shared<SegmentTreeNode>();
        floor_node->children.push_back(null_child);

        floor_node->object_cloud = floor_cloud;
        floor_node->object_type = PerceptionObject::FLOOR;
        root->children.push_back(floor_node);

        // 3) Remainder => "object"
        // The leftover in working_cloud is all the stuff that isn't walls or floor
        auto object_node = std::make_shared<SegmentTreeNode>();
        // for (int i = 0; i < working_cloud->points.size(); i++)
        // {
        //     working_cloud->points[i].r = 0.0;
        //     working_cloud->points[i].g = 0.0;
        //     working_cloud->points[i].b = 255.0;
        // }
        object_node->object_cloud = working_cloud;
        object_node->children.push_back(null_child);

        object_node->object_type = PerceptionObject::UNKNOWN_OBJECT;
        root->children.push_back(object_node);

        // 4) Subdivide the "object" node into smaller sub-objects (3rd level)
        subdivideObjectNode(object_node);

        return root;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    Environment3DPerception::extractAllVerticalPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_cloud,
                                                      float min_inliers)
    {
        // We'll store all vertical planes into one combined "walls" cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr walls_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        // Repeat RANSAC until no more vertical planes found
        while (true)
        {
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.0175f);
            seg.setInputCloud(working_cloud);

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
            seg.segment(*inliers, *coeffs);

            if (inliers->indices.empty() || inliers->indices.size() < min_inliers)
                break;

            float nx = coeffs->values[0];
            float ny = coeffs->values[1];
            float nz = coeffs->values[2];
            float dot_with_z = std::fabs(nz);
            if (dot_with_z > 0.3f)
                break;

            /*
                ANY LARGE VERTICAL PLANE WHICH IS NOT A WALL (STACK, SHELF) WILL BE CONSIDERED AS A WALL
                IF ANY PLANE IS BIGGER THAN WALLS, THEN NO WALL WILL BE EXTRACTED
            */

            // If it's vertical, extract inliers and add them to walls_cloud
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(working_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            extract.filter(*plane_cloud);

            *walls_cloud += *plane_cloud;

            // Remove these inliers from working_cloud
            extract.setNegative(true);
            extract.filter(*working_cloud);
        }
        for (int i = 0; i < walls_cloud->points.size(); i++)
        {
            walls_cloud->points[i].r = 255.0;
            walls_cloud->points[i].g = 0.0;
            walls_cloud->points[i].b = 0.0;
        }
        return walls_cloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    Environment3DPerception::extractFloorPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_cloud,
                                               float min_inliers)
    {
        // We'll just do one pass for the floor
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.02f);
        seg.setInputCloud(working_cloud);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
        seg.segment(*inliers, *coeffs);

        if (inliers->indices.empty() || inliers->indices.size() < min_inliers)
            return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Check if plane is horizontal with average z near 0
        // Normal for a horizontal plane is ~ [0,0,1]
        float nx = coeffs->values[0];
        float ny = coeffs->values[1];
        float nz = coeffs->values[2];
        float dot_with_z = std::fabs(nz);
        if (dot_with_z < 0.8f)
            return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        // We can also check the plane's average z, or compute the plane eq to see if d is near 0.
        // For simplicity, let's skip that or do a quick check:
        // plane eq: ax + by + cz + d = 0 => if c ~ ±1, then d is the offset from origin
        // We'll do a small threshold on d
        float d = coeffs->values[3];
        if (std::fabs(d) > 0.1f)
            return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(working_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.filter(*floor_cloud);

        // Remove from working_cloud
        extract.setNegative(true);
        extract.filter(*working_cloud);
        for (int i = 0; i < floor_cloud->points.size(); i++)
        {
            floor_cloud->points[i].r = 0.0;
            floor_cloud->points[i].g = 255.0;
            floor_cloud->points[i].b = 0.0;
        }
        return floor_cloud;
    }

    void Environment3DPerception::subdivideObjectNode(std::shared_ptr<SegmentTreeNode> object_node)
    {
        // If empty or too small, skip
        if (!object_node->object_cloud || object_node->object_cloud->empty())
            return;

        // We'll do a simple Euclidean Clustering to find sub-objects
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        tree->setInputCloud(object_node->object_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.0175); // tune
        ec.setMinClusterSize(25);        // tune
        ec.setMaxClusterSize(100000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(object_node->object_cloud);
        ec.extract(cluster_indices);
        std::cout << "number of cluster extracted = " << cluster_indices.size() << std::endl;

        int sub_id = 0;
        vector<vector<float>> colour = {
            {0.0, 0.0, 255.0},     // Blue
            {255.0, 20.0, 203.0},  // Dark Pink
            {255.0, 255.0, 0.0},   // Yellow
            {255.0, 165.0, 0.0},   // Orange
            {128.0, 0.0, 128.0},   // Purple
            {128.0, 128.0, 128.0}, // Grey
            {0.0, 100.0, 0.0},     // Dark Green
            {255.0, 255.0, 255.0}, // White
            {0.0, 0.0, 0.0}        // Black
        };
        for (auto &c_indices : cluster_indices)
        {
            cout << "sub_id " << sub_id << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            cluster_cloud->points.reserve(c_indices.indices.size());
            for (auto idx : c_indices.indices)
            {
                cluster_cloud->points.push_back(object_node->object_cloud->points[idx]);
            }
            for (int i = 0; i < cluster_cloud->points.size(); i++)
            {
                if (sub_id < 9)
                {
                    cluster_cloud->points[i].r = colour[sub_id][0];
                    cluster_cloud->points[i].g = colour[sub_id][1];
                    cluster_cloud->points[i].b = colour[sub_id][2];
                }
                else
                {
                    cluster_cloud->points[i].r = 100.0;
                    cluster_cloud->points[i].g = 0.0;
                    cluster_cloud->points[i].b = 0.0;
                }
            }

            auto child_node = std::make_shared<SegmentTreeNode>();
            child_node->object_cloud = cluster_cloud;
            child_node->object_type = PerceptionObject::SUB_UNKNOWN_OBJECT;
            auto null_child = std::make_shared<SegmentTreeNode>();
            null_child = nullptr;
            child_node->children.push_back(null_child);
            object_node->children.push_back(child_node);
            sub_id++;
        }

        RCLCPP_INFO(node_->get_logger(), "Found %lu sub-objects in leftover 'object' cloud.",
                    cluster_indices.size());
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>
    Environment3DPerception::segmentTableLegsConditionalWithCleaning(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud)
    {
        // ------------------ 1) Normal Estimation (OMP) ------------------ //
        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne_omp;
        ne_omp.setNumberOfThreads(4); // use all CPU cores
        ne_omp.setInputCloud(input_cloud);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr raw_tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne_omp.setSearchMethod(raw_tree);
        ne_omp.setKSearch(10); // want at least 10 neighbors

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        ne_omp.compute(*normals);

        // ------------------ 2) Merge Points + Normals ------------------ //
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        pcl::copyPointCloud(*input_cloud, *cloud_with_normals);
        for (size_t i = 0; i < input_cloud->size(); ++i)
        {
            cloud_with_normals->points[i].normal_x = normals->points[i].normal_x;
            cloud_with_normals->points[i].normal_y = normals->points[i].normal_y;
            cloud_with_normals->points[i].normal_z = normals->points[i].normal_z;
        }

        // ------------------ 3) Clean "Bad" Points ------------------ //
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr combo_tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
        combo_tree->setInputCloud(cloud_with_normals);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cleaned = SegmentationCondition::cleanCloud(cloud_with_normals, combo_tree, 10);
        if (cleaned->empty())
        {
            std::cerr << "[Warning] All points removed after cleaning. Returning empty clusters.\n";
            return {};
        }

        // ------------------ 4) Conditional Euclidean Clustering ------------------ //
        pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec;
        cec.setInputCloud(cleaned);
        cec.setSearchMethod(combo_tree);
        cec.setClusterTolerance(0.0175f); // 1.75 cm
        cec.setMinClusterSize(30);
        cec.setMaxClusterSize(100000);
        cec.setConditionFunction(&SegmentationCondition::tableLegCondition);

        std::vector<pcl::PointIndices> cluster_indices;
        cec.segment(cluster_indices);

        // Build final output clusters
        std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clusters;
        clusters.reserve(cluster_indices.size());
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            cluster->reserve(indices.indices.size());
            for (int idx : indices.indices)
            {
                cluster->push_back(cleaned->points[idx]);
            }
            clusters.push_back(cluster);
        }

        return clusters;
    }

    bool SegmentationCondition::tableLegCondition(const pcl::PointXYZRGBNormal &p1, const pcl::PointXYZRGBNormal &p2, float squared_distance)
    {
        // Some thresholds:
        const float distance_thresh = 0.0175;  // 2 cm
        const float orientation_thresh = 0.3f; // how different normal_z can be

        // dot with Z ~ absolute value of normal_z (assuming unit-length normals)
        float dot1_with_z = std::fabs(p1.normal_z);
        float dot2_with_z = std::fabs(p2.normal_z);

        // Euclidean check
        if (squared_distance > distance_thresh * distance_thresh)
            return false;

        // Orientation check: if both points are "close" in normal_z
        float diff = std::fabs(dot1_with_z - dot2_with_z);
        if (diff < orientation_thresh)
            return true;

        return false;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr SegmentationCondition::cleanCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_with_normals,
                                                                                   pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr &tree, int K)
    {
        // We will build a new, "cleaned" cloud
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cleaned(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
        cleaned->reserve(cloud_with_normals->size());

        // Prepare neighbor storage
        std::vector<int> k_indices(K);
        std::vector<float> k_distances(K);

        for (const auto &pt : cloud_with_normals->points)
        {
            if (!pcl::isFinite(pt) ||
                std::fabs(pt.normal_x) + std::fabs(pt.normal_y) + std::fabs(pt.normal_z) < 1e-4f)
            {
                std::cout << "invalied normal" << std::endl;
                std::cout << "-----------------------------------" << std::endl;
                continue;
            }
            int found = tree->nearestKSearch(pt, K, k_indices, k_distances);
            if (found < K)
            {
                std::cout << "invalied point with less neighbours" << std::endl;
                std::cout << "-----------------------------------" << std::endl;
                continue;
            }
            cleaned->push_back(pt);
        }

        cleaned->width = cleaned->size();
        cleaned->height = 1;
        cleaned->is_dense = false; // might not be strictly "dense"

        return cleaned;
    }
}
