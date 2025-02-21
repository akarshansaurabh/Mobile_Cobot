#include "pc_processing/threeD_perception.hpp"

namespace environment3DPerception
{
    ShapeExtraction::ShapeExtraction(std::shared_ptr<visualization::VisualizationManager> viz_manager)
    {
        viz_manager_ = viz_manager;
    }

    void ShapeExtraction::computeAABB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Eigen::Vector3f &min_pt, Eigen::Vector3f &max_pt)
    {
        pcl::PointXYZRGB min_p, max_p;
        pcl::getMinMax3D(*cloud, min_p, max_p);
        min_pt << min_p.x, min_p.y, min_p.z;
        max_pt << max_p.x, max_p.y, max_p.z;
    }
    bool ShapeExtraction::fitCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::ModelCoefficients::Ptr &coefficients)
    {
        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.0175);
        seg.setRadiusLimits(0.0, 9999.0);
        seg.setInputCloud(cloud);
        seg.setInputNormals(normals);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            return false;
        }
        return true;
    }
    int ShapeExtraction::countPointsInsideAABB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt)
    {
        int inside_count = 0;
        for (const auto &pt : cloud->points)
        {
            if (pt.x >= min_pt.x() && pt.x <= max_pt.x() &&
                pt.y >= min_pt.y() && pt.y <= max_pt.y() &&
                pt.z >= min_pt.z() && pt.z <= max_pt.z())
                inside_count++;
        }
        return inside_count;
    }
    int ShapeExtraction::countPointsInsideCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const pcl::ModelCoefficients::Ptr &cyl_coeffs, float extra_padding)
    {
        // Axis point
        Eigen::Vector3f axis_pt(cyl_coeffs->values[0],
                                cyl_coeffs->values[1],
                                cyl_coeffs->values[2]);
        // Axis direction
        Eigen::Vector3f axis_dir(cyl_coeffs->values[3],
                                 cyl_coeffs->values[4],
                                 cyl_coeffs->values[5]);
        axis_dir.normalize();
        float radius = cyl_coeffs->values[6] + extra_padding;

        float min_proj = 1e9f, max_proj = -1e9f;
        for (const auto &pt : cloud->points)
        {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            float proj = axis_dir.dot(p - axis_pt);
            if (proj < min_proj)
                min_proj = proj;
            if (proj > max_proj)
                max_proj = proj;
        }

        int inside_count = 0;
        for (const auto &pt : cloud->points)
        {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            float proj = axis_dir.dot(p - axis_pt);
            if (proj >= min_proj && proj <= max_proj)
            {
                Eigen::Vector3f radial_vec = (p - axis_pt) - proj * axis_dir;
                float rdist = radial_vec.norm();
                if (rdist <= radius)
                    inside_count++;
            }
        }
        return inside_count;
    }
    visualization::Shape3D ShapeExtraction::fitBestBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                               float Rc, float Gc, float Bc)
    {
        visualization::Shape3D ans;
        ans.r = Rc;
        ans.g = Gc;
        ans.b = Bc;
        // cuboid
        Eigen::Vector3f min_pt, max_pt;
        computeAABB(cloud, min_pt, max_pt);

        float aabb_volume = (max_pt.x() - min_pt.x()) *
                            (max_pt.y() - min_pt.y()) *
                            (max_pt.z() - min_pt.z());

        int inside_aabb = countPointsInsideAABB(cloud, min_pt, max_pt);
        float ratio_aabb = inside_aabb / static_cast<float>(cloud->size());

        // 2) Cylinder
        pcl::ModelCoefficients::Ptr cyl_coeffs(new pcl::ModelCoefficients);
        bool cyl_ok = fitCylinder(cloud, normals, cyl_coeffs);
        float cyl_volume = 999999.0f;
        float ratio_cyl = 0.0f;

        if (cyl_ok)
        {
            Eigen::Vector3f axis_pt(cyl_coeffs->values[0], cyl_coeffs->values[1], cyl_coeffs->values[2]);
            Eigen::Vector3f axis_dir(cyl_coeffs->values[3], cyl_coeffs->values[4], cyl_coeffs->values[5]);
            axis_dir.normalize();
            Eigen::Vector3f z_cap = axis_dir;
            // find min_proj, max_proj along axis
            float min_proj = 1e9f, max_proj = -1e9f;
            int index = 0;
            for (const auto &pt : cloud->points)
            {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                float proj = axis_dir.dot(p - axis_pt);
                if (index == 0)
                {
                    index = 1;
                    Eigen::Vector3f radial_vec = (p - axis_pt) - (proj * axis_dir);
                    radial_vec.normalize();
                    Eigen::Vector3f y_cap = radial_vec;
                    Eigen::Vector3f x_cap = y_cap.cross(z_cap);
                    x_cap.normalize();
                    Eigen::Matrix3d rot;
                    rot << x_cap(0), y_cap(0), z_cap(0),
                        x_cap(1), y_cap(1), z_cap(1),
                        x_cap(2), y_cap(2), z_cap(2);
                    ans.pose.orientation = Conversions::EigenM_2ROSQuat(rot);
                }

                if (proj < min_proj)
                    min_proj = proj;
                if (proj > max_proj)
                    max_proj = proj;
            }
            float height = max_proj - min_proj;
            float radius = cyl_coeffs->values[6];
            ans.pose.position.x = (min_proj * axis_dir.x()) + ((height / 2.0) * axis_dir.x());
            ans.pose.position.y = (min_proj * axis_dir.y()) + ((height / 2.0) * axis_dir.y());
            ans.pose.position.z = (min_proj * axis_dir.z()) + ((height / 2.0) * axis_dir.z());
            ans.R = radius;
            ans.H = height;
            cyl_volume = static_cast<float>(M_PI) * radius * radius * height;
            int inside_cyl = countPointsInsideCylinder(cloud, cyl_coeffs);
            ratio_cyl = inside_cyl / static_cast<float>(cloud->size());
        }

        // // 3) Compare which shape is "better"

        // if (std::fabs(aabb_volume - cyl_volume) < 1e-3)
        // {
        //     if (ratio_aabb > ratio_cyl)
        //     {
        //         ans.shape3d = visualization::Shapes::CUBOID;
        //         ans.pose.position.x = (max_pt.x() + min_pt.x()) / 2.0;
        //         ans.pose.position.y = (max_pt.y() + min_pt.y()) / 2.0;
        //         ans.pose.position.z = (max_pt.z() + min_pt.z()) / 2.0;
        //         Eigen::Matrix3d rot;
        //         rot.setIdentity();
        //         ans.pose.orientation = Conversions::EigenM_2ROSQuat(rot);
        //         ans.L = (max_pt.x() - min_pt.x());
        //         ans.B = (max_pt.y() - min_pt.y());
        //         ans.H = (max_pt.z() - min_pt.z());
        //     }
        //     else
        //     {
        //         // return BoundingShapeResult{"Cylinder", cyl_volume, ratio_cyl};
        //         ans.shape3d = visualization::Shapes::CYLINDER;
        //     }
        // }
        // else if (aabb_volume < cyl_volume)
        // {
        //     // return BoundingShapeResult{"AABB", aabb_volume, ratio_aabb};
        //     ans.shape3d = visualization::Shapes::CUBOID;
        //     geometry_msgs::msg::Pose pose;
        //     ans.pose.position.x = (max_pt.x() + min_pt.x()) / 2.0;
        //     ans.pose.position.y = (max_pt.y() + min_pt.y()) / 2.0;
        //     ans.pose.position.z = (max_pt.z() + min_pt.z()) / 2.0;
        //     Eigen::Matrix3d rot;
        //     rot.setIdentity();
        //     ans.pose.orientation = Conversions::EigenM_2ROSQuat(rot);
        //     ans.L = (max_pt.x() - min_pt.x());
        //     ans.B = (max_pt.y() - min_pt.y());
        //     ans.H = (max_pt.z() - min_pt.z());
        // }
        // else
        // {
        //     ans.shape3d = visualization::Shapes::CYLINDER;
        // }

        if (std::fabs(ratio_aabb - ratio_cyl) < 1e-3)
        {
            if (aabb_volume < cyl_volume)
            {
                ans.shape3d = visualization::Shapes::CUBOID;
                ans.pose.position.x = (max_pt.x() + min_pt.x()) / 2.0;
                ans.pose.position.y = (max_pt.y() + min_pt.y()) / 2.0;
                ans.pose.position.z = (max_pt.z() + min_pt.z()) / 2.0;
                Eigen::Matrix3d rot;
                rot.setIdentity();
                ans.pose.orientation = Conversions::EigenM_2ROSQuat(rot);
                ans.L = (max_pt.x() - min_pt.x());
                ans.B = (max_pt.y() - min_pt.y());
                ans.H = (max_pt.z() - min_pt.z());
            }
            else
            {
                // return BoundingShapeResult{"Cylinder", cyl_volume, ratio_cyl};
                ans.shape3d = visualization::Shapes::CYLINDER;
            }
        }
        else if (ratio_aabb > ratio_cyl)
        {
            // return BoundingShapeResult{"AABB", aabb_volume, ratio_aabb};
            ans.shape3d = visualization::Shapes::CUBOID;
            geometry_msgs::msg::Pose pose;
            ans.pose.position.x = (max_pt.x() + min_pt.x()) / 2.0;
            ans.pose.position.y = (max_pt.y() + min_pt.y()) / 2.0;
            ans.pose.position.z = (max_pt.z() + min_pt.z()) / 2.0;
            Eigen::Matrix3d rot;
            rot.setIdentity();
            ans.pose.orientation = Conversions::EigenM_2ROSQuat(rot);
            ans.L = (max_pt.x() - min_pt.x());
            ans.B = (max_pt.y() - min_pt.y());
            ans.H = (max_pt.z() - min_pt.z());
        }
        else
        {
            ans.shape3d = visualization::Shapes::CYLINDER;
        }

        if (ans.shape3d == visualization::Shapes::CUBOID)
        {
            for (int aa = 0; aa < 10; aa++)
            {
                viz_manager_->publishCuboidMarker(ans);
                // viz_manager_->id_--;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        else if (ans.shape3d == visualization::Shapes::CYLINDER)
        {
            for (int aa = 0; aa < 10; aa++)
            {
                viz_manager_->publishCylinderMarker(ans);
                // viz_manager_->id_--;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            std::cout << "cylinder " << std::endl;
        }

        return ans;
    }

    Environment3DPerception::Environment3DPerception(rclcpp::Node::SharedPtr node)
        : node_(node),
          front_sub_(nullptr),
          arm_sub_(nullptr),
          front_camera_received_(false),
          arm_camera_received_(false)
    {
        environment_3d_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        viz_manager_ = std::make_shared<visualization::VisualizationManager>(node);
        shape_extraction = std::make_unique<ShapeExtraction>(viz_manager_);

        octomap_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/octomap_topic_", rclcpp::QoS(10));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        node_->declare_parameter<std::string>("activate_3d_perception", "F");
        node_->get_parameter("activate_3d_perception", activate_3d_perception_);

        parameter_callback_handle_ = node_->add_on_set_parameters_callback(std::bind(&Environment3DPerception::ParameterCallback, this, std::placeholders::_1));

        if (activate_3d_perception_ == "T")
        {
            RCLCPP_INFO(node_->get_logger(), "activate_3d_perception is true at startup. Creating timer.");
            timer_ = node_->create_wall_timer(std::chrono::seconds(2),
                                              std::bind(&Environment3DPerception::timerCallback, this));
        }
        else
            RCLCPP_INFO(node_->get_logger(), "activate_3d_perception is false at startup. No timer created.");

        original_colour = {
            {0.0, 0.0, 255.0},     // Blue
            {255.0, 20.0, 203.0},  // Dark Pink
            {255.0, 255.0, 0.0},   // Yellow
            {255.0, 165.0, 0.0},   // Orange
            {128.0, 0.0, 128.0},   // Purple
            {128.0, 128.0, 128.0}, // Grey
            {0.0, 100.0, 0.0},     // Dark Green
            {255.0, 255.0, 255.0}, // White
            {0.0, 0.0, 0.0},       // Black
            {0.0, 255.0, 255.0},   // Cyan
            {255.0, 0.0, 255.0},   // Magenta
            {139.0, 69.0, 19.0},   // Brown
            {75.0, 0.0, 130.0},    // Indigo
            {173.0, 216.0, 230.0}, // Light Blue
            {240.0, 128.0, 128.0}, // Light Coral
            {34.0, 139.0, 34.0},   // Forest Green
            {255.0, 215.0, 0.0},   // Gold
            {192.0, 192.0, 192.0}  // Silver
        };

        colour = original_colour;
    }

    void Environment3DPerception::traverseSegmentationTree(const std::shared_ptr<SegmentTreeNode> &node, pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_pcl_cloud, int level)
    {
        if (!node)
            return;

        for (int i = 0; i < level + 1; i++)
            std::cout << " ";
        std::cout << "A" << std::endl;

        *segmented_pcl_cloud += *node->object_cloud;
        for (auto &child : node->children)
            traverseSegmentationTree(child, segmented_pcl_cloud, level + 1);
    }

    rcl_interfaces::msg::SetParametersResult Environment3DPerception::ParameterCallback(const std::vector<rclcpp::Parameter> &parameters)
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
            colour = original_colour;

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
        traverseSegmentationTree(root, segmented_pcl_cloud, 0);

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
        viz_manager_->id_ = 0;
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

        walls_node->object_cloud = walls_cloud;
        walls_node->object_type = PerceptionObject::WALL;

        // 2) Extract a floor plane => "floor" (assuming only one floor)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_cloud = extractFloorPlane(working_cloud, /*min_inliers=*/200);

        auto floor_node = std::make_shared<SegmentTreeNode>();

        floor_node->object_cloud = floor_cloud;
        floor_node->object_type = PerceptionObject::FLOOR;

        // 3) Remainder => "object"
        // The leftover in working_cloud is all the stuff that isn't walls or floor
        auto object_node = std::make_shared<SegmentTreeNode>();

        object_node->object_cloud = working_cloud;
        object_node->object_type = PerceptionObject::UNKNOWN_OBJECT;

        root->children.push_back(walls_node);
        root->children.push_back(floor_node);
        root->children.push_back(object_node);

        walls_node->children.push_back(null_child);
        floor_node->children.push_back(null_child);

        // 4) Subdivide the "object" node into smaller sub-objects (3rd level)
        subdivideObjectNode(object_node);
        for (auto child_node : object_node->children)
        {
            std::cout << "3rd Level" << std::endl;
            segmentTableLegsConditionalWithCleaning(child_node);
        }
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
        seg.setDistanceThreshold(0.0175f);
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
        ec.setMinClusterSize(25);       // tune
        ec.setMaxClusterSize(100000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(object_node->object_cloud);
        ec.extract(cluster_indices);
        // std::cout << "number of cluster extracted = " << cluster_indices.size() << std::endl;

        int sub_id = 0;
        for (auto &c_indices : cluster_indices)
        {
            // cout << "sub_id " << sub_id << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            cluster_cloud->points.reserve(c_indices.indices.size());
            for (auto idx : c_indices.indices)
            {
                cluster_cloud->points.push_back(object_node->object_cloud->points[idx]);
            }

            auto child_node = std::make_shared<SegmentTreeNode>();
            child_node->object_cloud = cluster_cloud;
            child_node->object_type = PerceptionObject::SUB_UNKNOWN_OBJECT;

            object_node->children.push_back(child_node);
            sub_id++;
        }

        RCLCPP_INFO(node_->get_logger(), "Found %lu sub-objects in leftover 'object' cloud.",
                    cluster_indices.size());
    }

    void Environment3DPerception::segmentTableLegsConditionalWithCleaning(std::shared_ptr<SegmentTreeNode> &object_node)
    {
        // ------------------ 1) Normal Estimation (OMP) ------------------ //
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*object_node->object_cloud));
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
            return;
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
        auto null_child = std::make_shared<SegmentTreeNode>();
        null_child = nullptr;
        int cluster_id = 0;
        std::cout << "number of cluster level 4 " << cluster_indices.size() << std::endl;
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::Normal>::Ptr object_normals(new pcl::PointCloud<pcl::Normal>());
            // cluster i
            std::cout << "cluster_id = " << cluster_id << std::endl;
            std::shared_ptr<SegmentTreeNode> child = std::make_shared<SegmentTreeNode>();
            child->object_cloud->points.reserve(indices.indices.size());
            for (int idx : indices.indices)
            {
                pcl::PointXYZRGB point;
                point.x = cleaned->points[idx].x;
                point.y = cleaned->points[idx].y;
                point.z = cleaned->points[idx].z;
                point.r = colour[0][0];
                point.g = colour[0][1];
                point.b = colour[0][2];
                child->object_cloud->points.push_back(point);

                pcl::Normal normal_;
                normal_.normal_x = cleaned->points[idx].normal_x;
                normal_.normal_y = cleaned->points[idx].normal_y;
                normal_.normal_z = cleaned->points[idx].normal_z;
                object_normals->points.push_back(normal_);
            }
            double r0 = colour[0][0] / 255;
            double g0 = colour[0][1] / 255;
            double b0 = colour[0][2] / 255;
            auto shape = shape_extraction->fitBestBoundingBox(child->object_cloud, object_normals, r0, g0, b0);
            cluster_id++;
            colour.erase(colour.begin());
            child->object_type = PerceptionObject::SUB_UNKNOWN_OBJECT;
            child->children.push_back(null_child);
            object_node->children.push_back(child);
        }
    }

    bool SegmentationCondition::tableLegCondition(const pcl::PointXYZRGBNormal &p1, const pcl::PointXYZRGBNormal &p2, float squared_distance)
    {
        // Some thresholds:
        const float distance_thresh = 0.0175;
        const float orientation_thresh = 0.15; // how different normal_z can be

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
