#include "pc_processing/octomap_generator.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace octoMapGenerator
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    geometry_msgs::msg::TransformStamped map_to_base_transform;

    PointCloudStitcher::PointCloudStitcher(rclcpp::Node::SharedPtr node, const std::string &map_frame, std::mutex &m,
                                           std::condition_variable &cv, bool &callback_triggered_flag, bool stitch,
                                           rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr octomap_pub___)
        : node_(node),
          map_frame_(map_frame),
          voxel_grid_leaf_size_(0.015f),
          std_dev_mul_thresh_(1.0),
          mean_k_(50),
          mutex_(m),
          cv_(cv),
          callback_triggered_(callback_triggered_flag),
          start_stiching(stitch)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        octomap_pub_ = octomap_pub___;
        pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/arm_rgbd_camera/points", rclcpp::QoS(10),
            std::bind(&PointCloudStitcher::pointCloudSnapShotCallback, this, std::placeholders::_1));

        box6dposes_client_ = node_->create_client<custom_interfaces::srv::BoxposeEstimator>("six_d_pose_estimate_service");
    }

    void PointCloudStitcher::pointCloudSnapShotCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &snapshot_msg)
    {
        std::string camera_frame = snapshot_msg->header.frame_id;
        rclcpp::Time cloud_time = snapshot_msg->header.stamp;

        try
        {
            map_to_camera_transform = tf_buffer_->lookupTransform(
                map_frame_, camera_frame, cloud_time, tf2::durationFromSec(1.0));
        }
        catch (tf2::ExtrapolationException &ex)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "TF extrapolation error for time %.9f. Retrying with latest available transform.",
                        cloud_time.seconds());
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_snapshot_in_map = transformCloudToMap(*snapshot_msg);
        addCloudSnapshot(pcl_snapshot_in_map);

        if (start_stiching)
        {
            finalizeStitching(0.005f);
            sensor_msgs::msg::PointCloud2 octomap;
            pcl::toROSMsg(*accumulated_cloud_, octomap);
            octomap.header.stamp = node_->now();
            octomap.header.frame_id = map_frame_;
            octomap_pub_->publish(octomap);
            try
            {
                map_to_base_transform = tf_buffer_->lookupTransform("map", "base_link", cloud_time, tf2::durationFromSec(5.0));
                std::cout << "base at the time of snapshot " << map_to_base_transform.transform.translation.x << " "
                          << map_to_base_transform.transform.translation.y << " "
                          << map_to_base_transform.transform.translation.z << " "
                          << map_to_base_transform.transform.rotation.x << " "
                          << map_to_base_transform.transform.rotation.y << " "
                          << map_to_base_transform.transform.rotation.z << " "
                          << map_to_base_transform.transform.rotation.w << " " << std::endl;
            }
            catch (tf2::ExtrapolationException &ex)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "No Base.",
                            cloud_time.seconds());
                return;
            }
            SendRequestFor6DPoseEstimation(0.1, octomap);
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            callback_triggered_ = true;
        }
        cv_.notify_one();
        pointcloud_sub_.reset();
    }

    // source of error - robot is at A1 when snapshot taken, but in TF, robot is at A1'
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudStitcher::transformCloudToMap(const sensor_msgs::msg::PointCloud2 &msg)
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

    void PointCloudStitcher::addCloudSnapshot(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &snapshot)
    {
        *accumulated_cloud_ += *snapshot;
    }

    void PointCloudStitcher::finalizeStitching(float leaf_size)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(accumulated_cloud_);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*accumulated_cloud_);
    }

    Eigen::Affine3d PointCloudStitcher::poseToEigen(const geometry_msgs::msg::Transform &transform)
    {
        Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
        Eigen::Translation3d t(transform.translation.x, transform.translation.y, transform.translation.z);
        return t * q;
    }

    void PointCloudStitcher::SendRequestFor6DPoseEstimation(double delta, const sensor_msgs::msg::PointCloud2 &cloud)
    {
        if (!box6dposes_client_->wait_for_service(5s)) // Wait for 5 second
        {
            RCLCPP_ERROR(node_->get_logger(), "Service '6d_pose_estimate_service' not available.");
            return;
        }

        auto request = std::make_shared<custom_interfaces::srv::BoxposeEstimator::Request>();

        request->delta = delta;
        request->cloud = cloud;
        auto future = box6dposes_client_->async_send_request(request, std::bind(&PointCloudStitcher::HandleResponse, this, std::placeholders::_1));
    }

    void PointCloudStitcher::HandleResponse(rclcpp::Client<custom_interfaces::srv::BoxposeEstimator>::SharedFuture future)
    {
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node_->get_logger(), "Service call succeeded: ");
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Service call failed: ");
        }
    }

    OctoMapGenerator::OctoMapGenerator(const rclcpp::Node::SharedPtr &node, const std::shared_ptr<cMRKinematics::ArmKinematicsSolver> &kinematics_solver)
        : node_(node), kinematics_solver_(kinematics_solver)
    {
        colision_free_planner_server_ = node_->create_service<custom_interfaces::srv::GoalPoseVector>(
            "colision_free_planner_service", std::bind(&OctoMapGenerator::ServerCallbackForColisionFreePlanning,
                                                       this, std::placeholders::_1, std::placeholders::_2));
        octree_ = nullptr;

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void OctoMapGenerator::ServerCallbackForColisionFreePlanning(const std::shared_ptr<custom_interfaces::srv::GoalPoseVector::Request> request,
                                                                 std::shared_ptr<custom_interfaces::srv::GoalPoseVector::Response> response)
    {
        buildOctomap(accumulated_cloud_);
        auto ompl_planner = std::make_unique<collision_free_planning::CollisionFreePlanner>(node_, octree_, kinematics_solver_, map_to_base_transform);

        std::vector<std::vector<double>> joint_states_vector = ompl_planner->planPath(request->goal_poses_for_arm.poses[0]);
        std::cout << "ompl planning done" << std::endl;
        // auto visualizer = std::make_unique<visualization::VisualizationManager>(node_);
        // visualizer->publishPathWithOrientations(colision_free_path);
        std_msgs::msg::Float64MultiArray float_array_msg;
        for (const auto &joint_states : joint_states_vector)
        {
            float_array_msg.data = joint_states;
            response->joint_states_vector.push_back(float_array_msg);
        }
        response->reply = true;
        accumulated_cloud_->points.clear();
    }

    void OctoMapGenerator::buildOctomap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, double resolution)
    {
        // Create an OcTree
        octree_ = std::make_shared<octomap::OcTree>(resolution);

        // Insert points
        for (const auto &pt : pcl_cloud->points)
        {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
            {
                octomap::point3d p(pt.x, pt.y, pt.z);
                octree_->updateNode(p, true);
            }
        }
        octree_->updateInnerOccupancy();

        RCLCPP_INFO(node_->get_logger(),
                    "[CollisionFreePlanner] OctoMap built: %zu pts, resolution=%.3f",
                    pcl_cloud->size(), resolution);
    }
}