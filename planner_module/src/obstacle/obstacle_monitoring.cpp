#include "obstacle/obstacle_monitoring.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

namespace obstacle_monitoring
{
    ObstacleMonitor::ObstacleMonitor()
        : Node("obstacle_monitor_node"),
          operating_mode_(OperatingMode::Idle),
          robot_base_frame_("base_link"),
          global_frame_("map"),
          laser_scan_buffer_size_(5),
          scan_difference_threshold_(0.1),
          detection_radius_(2.0),
          destination_x_(0.0),
          destination_y_(0.0),
          destination_radius_(1.0),
          is_global_costmap_initialized_(false),
          is_local_costmap_initialized_(false),
          new_obstacle_detected_(false),
          laser_subscribed_(false),
          activate_costmap_comparison_(false)
    {
        // Declare parameters
        this->declare_parameter("robot_base_frame", robot_base_frame_);
        this->declare_parameter("global_frame", global_frame_);
        this->declare_parameter("laser_scan_buffer_size", laser_scan_buffer_size_);
        this->declare_parameter("scan_difference_threshold", scan_difference_threshold_);
        this->declare_parameter("detection_radius", detection_radius_);
        this->declare_parameter("destination_radius", destination_radius_);
        this->declare_parameter("activate_costmap_comparison", false);
        this->declare_parameter("destination_x", destination_x_);
        this->declare_parameter("destination_y", destination_y_);

        // Get parameters
        bool temp_activate_costmap_comparison;
        this->get_parameter("robot_base_frame", robot_base_frame_);
        this->get_parameter("global_frame", global_frame_);
        this->get_parameter("laser_scan_buffer_size", laser_scan_buffer_size_);
        this->get_parameter("scan_difference_threshold", scan_difference_threshold_);
        this->get_parameter("detection_radius", detection_radius_);
        this->get_parameter("destination_radius", destination_radius_);
        this->get_parameter("activate_costmap_comparison", temp_activate_costmap_comparison);
        activate_costmap_comparison_ = temp_activate_costmap_comparison;
        this->get_parameter("destination_x", destination_x_);
        this->get_parameter("destination_y", destination_y_);

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer for updating operating mode
        // mode_update_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(500),
        //     std::bind(&ObstacleMonitor::updateOperatingMode, this));

        // Parameter callback
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ObstacleMonitor::parameterCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Obstacle Monitor Node Initialized in Idle mode.");
    }

    void ObstacleMonitor::switchOperatingMode(OperatingMode mode)
    {
        if (operating_mode_ == mode)
            return;

        operating_mode_ = mode;

        std::string mode_str = (mode == OperatingMode::Idle) ? "Idle" : (mode == OperatingMode::Stationary) ? "Stationary"
                                                                                                            : "Moving";

        RCLCPP_INFO(this->get_logger(), "Switching to %s mode.", mode_str.c_str());

        switch (operating_mode_)
        {
        case OperatingMode::Stationary:
            // laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            //     "/scan", rclcpp::QoS(10),
            //     std::bind(&ObstacleMonitor::laserScanCallback, this, std::placeholders::_1));
            if (global_costmap_sub_)
                global_costmap_sub_.reset();
            if (local_costmap_sub_)
                local_costmap_sub_.reset();
            break;
        case OperatingMode::Idle:
            if (laser_scan_sub_)
                laser_scan_sub_.reset();
            if (global_costmap_sub_)
                global_costmap_sub_.reset();
            if (local_costmap_sub_)
                local_costmap_sub_.reset();
            break;
        case OperatingMode::Moving:
            if (laser_scan_sub_)
                laser_scan_sub_.reset();
            if (global_costmap_sub_)
                global_costmap_sub_.reset();
            if (local_costmap_sub_)
                local_costmap_sub_.reset();
            break;
        }
    }

    void ObstacleMonitor::updateOperatingMode()
    {
        // Logic to update operating mode can be added here
        // For now, we keep it simple
    }

    // void ObstacleMonitor::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    // {
    //     laser_scan_buffer_.push_back(msg);

    //     if (static_cast<int>(laser_scan_buffer_.size()) >= laser_scan_buffer_size_)
    //     {
    //         detectMovingObstacles();
    //         laser_scan_buffer_.clear();
    //     }
    // }

    // void ObstacleMonitor::detectMovingObstacles()
    // {
    //     auto current_scan = laser_scan_buffer_.back();
    //     auto previous_scan = laser_scan_buffer_.front();
    //     int obs_counter = 0;

    //     // Ensure scans are compatible
    //     if (current_scan->ranges.size() != previous_scan->ranges.size())
    //     {
    //         RCLCPP_WARN(this->get_logger(), "Laser scans have different sizes.");
    //         return;
    //     }

    //     // Get robot's current position in map frame -> put this code in stationary state transition
    //     geometry_msgs::msg::TransformStamped robot_transform;
    //     try
    //     {
    //         robot_transform = tf_buffer_->lookupTransform(
    //             global_frame_, robot_base_frame_, current_scan->header.stamp, tf2::durationFromSec(0.1));
    //     }
    //     catch (tf2::TransformException &ex)
    //     {
    //         RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
    //         return;
    //     }

    //     double robot_x = robot_transform.transform.translation.x;
    //     double robot_y = robot_transform.transform.translation.y;

    //     // std::vector<MovingObstacle> new_obstacles;

    //     // Compare scans
    //     for (size_t i = 0; i < current_scan->ranges.size(); ++i)
    //     {
    //         double range_current = current_scan->ranges[i];
    //         double range_previous = previous_scan->ranges[i];

    //         if (std::isfinite(range_current) && std::isfinite(range_previous))
    //         {
    //             double range_difference = std::abs(range_current - range_previous);
    //             if (range_difference > scan_difference_threshold_)
    //             {
    //                 // Potential moving obstacle detected but it may be far away
    //                 double angle = current_scan->angle_min + i * current_scan->angle_increment;
    //                 double x = range_current * cos(angle);
    //                 double y = range_current * sin(angle);
    //                 double dis_from_lidar = std::hypot(x, y);

    //                 if (dis_from_lidar < 2 * detection_radius_)
    //                 {
    //                     // Transform to global frame
    //                     geometry_msgs::msg::PointStamped local_point, global_point;
    //                     local_point.header.frame_id = current_scan->header.frame_id;
    //                     local_point.header.stamp = current_scan->header.stamp;
    //                     local_point.point.x = x;
    //                     local_point.point.y = y;
    //                     local_point.point.z = 0.0;

    //                     try
    //                     {
    //                         tf_buffer_->transform(local_point, global_point, global_frame_, tf2::durationFromSec(0.1));

    //                         // Calculate distance between obstacle and robot
    //                         double dx = global_point.point.x - robot_x;
    //                         double dy = global_point.point.y - robot_y;
    //                         double distance_obs_from_robot = std::hypot(dx, dy);

    //                         if (distance_obs_from_robot <= detection_radius_)
    //                         {
    //                             // moving obs detected -> will stop the task as long as the moving obs are around
    //                             obs_counter++;
    //                             new_obstacle_detected_ = true;
    //                             break;
    //                         }
    //                     }
    //                     catch (tf2::TransformException &ex)
    //                     {
    //                         RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    //                         continue;
    //                     }
    //                 }
    //             }
    //         }
    //     }

    //     if (obs_counter > 0)
    //     {
    //         // moving obs are around
    //     }
    //     else if (obs_counter == 0 && new_obstacle_detected_)
    //     {
    //         // there were moving obs around but at present they are not around -> resume the paused task
    //     }
    //     else if (obs_counter == 0 && !new_obstacle_detected_)
    //     {
    //         // ignore this condition
    //     }
    // }

    void ObstacleMonitor::globalCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        // Initialize global costmap
        // if (!is_global_costmap_initialized_)
        // {
        std::cout << "trying reading global costmap" << std::endl;
        global_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
            msg->metadata.size_x, msg->metadata.size_y,
            msg->metadata.resolution,
            msg->metadata.origin.position.x,
            msg->metadata.origin.position.y);

        // Copy data to global costmap
        unsigned char *global_char_map = global_costmap_->getCharMap();
        std::copy(msg->data.begin(), msg->data.end(), global_char_map);

        is_global_costmap_initialized_ = true;
        // global_costmap_sub_.reset();
        // std::cout << "unsubscribed from global costmap" << std::endl;
        // }
    }

    // void ObstacleMonitor::globalCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    // {
    //     std::lock_guard<std::mutex> lock(costmap_mutex_);
    //     if (!global_costmap_)
    //     {
    //         global_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
    //             msg->metadata.size_x, msg->metadata.size_y,
    //             msg->metadata.resolution,
    //             msg->metadata.origin.position.x,
    //             msg->metadata.origin.position.y);
    //     }
    //     else
    //     {
    //         // Check if size, resolution, or origin has changed
    //         if (global_costmap_->getSizeInCellsX() != msg->metadata.size_x ||
    //             global_costmap_->getSizeInCellsY() != msg->metadata.size_y ||
    //             global_costmap_->getResolution() != msg->metadata.resolution ||
    //             global_costmap_->getOriginX() != msg->metadata.origin.position.x ||
    //             global_costmap_->getOriginY() != msg->metadata.origin.position.y)
    //         {
    //             RCLCPP_WARN(this->get_logger(), "Global costmap metadata has changed. Reinitializing costmap.");
    //             global_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
    //                 msg->metadata.size_x, msg->metadata.size_y,
    //                 msg->metadata.resolution,
    //                 msg->metadata.origin.position.x,
    //                 msg->metadata.origin.position.y);
    //         }
    //     }

    //     // Update cost data
    //     unsigned char *global_char_map = global_costmap_->getCharMap();
    //     std::copy(msg->data.begin(), msg->data.end(), global_char_map);

    //     is_global_costmap_initialized_ = true;
    // }

    // void ObstacleMonitor::localCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    // {
    //     std::lock_guard<std::mutex> lock(costmap_mutex_);
    //     if (!is_global_costmap_initialized_)
    //     {
    //         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Global costmap not initialized yet.");
    //         return;
    //     }

    //     // Initialize or update local costmap
    //     if (!local_costmap_)
    //     {
    //         local_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
    //             msg->metadata.size_x, msg->metadata.size_y,
    //             msg->metadata.resolution,
    //             msg->metadata.origin.position.x,
    //             msg->metadata.origin.position.y);
    //     }
    //     else
    //     {
    //         // Check if size or resolution has changed
    //         if (local_costmap_->getSizeInCellsX() != msg->metadata.size_x ||
    //             local_costmap_->getSizeInCellsY() != msg->metadata.size_y ||
    //             local_costmap_->getResolution() != msg->metadata.resolution)
    //         {
    //             RCLCPP_WARN(this->get_logger(), "Local costmap size or resolution has changed. Reinitializing costmap.");
    //             local_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
    //                 msg->metadata.size_x, msg->metadata.size_y,
    //                 msg->metadata.resolution,
    //                 msg->metadata.origin.position.x,
    //                 msg->metadata.origin.position.y);
    //         }
    //         else
    //         {
    //             // Update origin
    //             local_costmap_->updateOrigin(msg->metadata.origin.position.x, msg->metadata.origin.position.y);
    //         }
    //     }

    //     // Update cost data
    //     unsigned char *local_char_map = local_costmap_->getCharMap();
    //     std::copy(msg->data.begin(), msg->data.end(), local_char_map);

    //     is_local_costmap_initialized_ = true;

    //     // Get robot's position in the local costmap frame
    //     geometry_msgs::msg::TransformStamped robot_transform;
    //     try
    //     {
    //         robot_transform = tf_buffer_->lookupTransform(
    //             msg->header.frame_id, robot_base_frame_, tf2::TimePointZero);
    //     }
    //     catch (tf2::TransformException &ex)
    //     {
    //         RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
    //         return;
    //     }

    //     double robot_x = robot_transform.transform.translation.x;
    //     double robot_y = robot_transform.transform.translation.y;

    //     // Identify new obstacles
    //     std::vector<std::pair<double, double>> new_obstacle_positions;

    //     // Iterate over the local costmap
    //     for (unsigned int mx = 0; mx < local_costmap_->getSizeInCellsX(); ++mx)
    //     {
    //         for (unsigned int my = 0; my < local_costmap_->getSizeInCellsY(); ++my)
    //         {
    //             unsigned char local_cost = local_costmap_->getCost(mx, my);
    //             if (local_cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
    //             {
    //                 double wx, wy;
    //                 local_costmap_->mapToWorld(mx, my, wx, wy);

    //                 // Check if the obstacle is within a safe radius around the robot
    //                 double distance_to_robot = std::hypot(wx - robot_x, wy - robot_y);
    //                 if (distance_to_robot < 1.0) // Adjust this threshold as needed
    //                 {
    //                     std::cout << "skipping" << std::endl;
    //                     continue; // Skip obstacles near the robot
    //                 }

    //                 // Transform point to global frame
    //                 geometry_msgs::msg::PointStamped local_point, global_point;
    //                 local_point.header.frame_id = msg->header.frame_id;
    //                 local_point.header.stamp = msg->header.stamp;
    //                 local_point.point.x = wx;
    //                 local_point.point.y = wy;
    //                 local_point.point.z = 0.0;

    //                 try
    //                 {
    //                     tf_buffer_->transform(local_point, global_point, global_frame_, tf2::durationFromSec(0.1));

    //                     // Convert world coordinates to global costmap indices
    //                     unsigned int gmx, gmy;
    //                     if (global_costmap_->worldToMap(global_point.point.x, global_point.point.y, gmx, gmy))
    //                     {
    //                         unsigned char global_cost = global_costmap_->getCost(gmx, gmy);
    //                         if (global_cost < nav2_costmap_2d::LETHAL_OBSTACLE)
    //                         {
    //                             // This is a new obstacle
    //                             new_obstacle_positions.emplace_back(global_point.point.x, global_point.point.y);
    //                         }
    //                     }
    //                 }
    //                 catch (tf2::TransformException &ex)
    //                 {
    //                     RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    //                     continue;
    //                 }
    //             }
    //         }
    //     }

    //     // Process new obstacles
    //     if (!new_obstacle_positions.empty())
    //     {
    //         RCLCPP_INFO(this->get_logger(), "Detected %zu new obstacles.", new_obstacle_positions.size());
    //         // Handle detected obstacles
    //         // For example, you might want to publish them or update a map
    //     }
    // }

    void ObstacleMonitor::localCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        std::cout << "-------------------------------------------------------------" << std::endl;
        if (!is_global_costmap_initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "Global costmap not initialized yet.");
            return;
        }

        // std::lock_guard<std::mutex> lock(costmap_mutex_);
        // Initialize local costmap
        local_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
            msg->metadata.size_x, msg->metadata.size_y,
            msg->metadata.resolution,
            msg->metadata.origin.position.x,
            msg->metadata.origin.position.y);

        // Copy data to local costmap
        unsigned char *local_char_map = local_costmap_->getCharMap();
        std::copy(msg->data.begin(), msg->data.end(), local_char_map);
        std::cout << "comparing local costmap with global costmap" << std::endl;

        is_local_costmap_initialized_ = true;

        // Identify new obstacles
        std::vector<std::pair<double, double>> new_obstacle_positions;

        // Iterate over the local costmap
        std::cout << "num of points " << local_costmap_->getSizeInCellsX() * local_costmap_->getSizeInCellsY() << std::endl;
        int cells = 0;
        for (unsigned int mx = 0; mx < local_costmap_->getSizeInCellsX(); ++mx)
        {
            for (unsigned int my = 0; my < local_costmap_->getSizeInCellsY(); ++my)
            {
                unsigned char local_cost = local_costmap_->getCost(mx, my);
                if (local_cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
                {
                    double wx, wy;
                    local_costmap_->mapToWorld(mx, my, wx, wy);

                    // Transform point to global frame
                    geometry_msgs::msg::PointStamped local_point, global_point;
                    local_point.header.frame_id = msg->header.frame_id; // Typically 'odom'
                    local_point.header.stamp = msg->header.stamp;
                    local_point.point.x = wx;
                    local_point.point.y = wy;
                    local_point.point.z = 0.0;

                    try
                    {
                        tf_buffer_->transform(local_point, global_point, global_frame_, tf2::durationFromSec(0.1));

                        // Convert world coordinates to global costmap indices
                        unsigned int gmx, gmy;
                        if (global_costmap_->worldToMap(global_point.point.x, global_point.point.y, gmx, gmy))
                        {
                            unsigned char global_cost = global_costmap_->getCost(gmx, gmy);
                            if (global_cost < nav2_costmap_2d::LETHAL_OBSTACLE)
                            {
                                // This is a new obstacle
                                std::cout << "a new dynamic obstacle has been detected" << std::endl;
                                cells++;
                                new_obstacle_positions.emplace_back(global_point.point.x, global_point.point.y);
                            }
                        }
                    }
                    catch (tf2::TransformException &ex)
                    {
                        RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
                        continue;
                    }
                }
            }
        }
        std::cout << "num of cells " << cells << std::endl;
        // Process new obstacles
        if (!new_obstacle_positions.empty())
        {
            // Find the obstacle closest to the destination
            auto closest_obstacle_it = std::min_element(
                new_obstacle_positions.begin(), new_obstacle_positions.end(),
                [this](const std::pair<double, double> &a, const std::pair<double, double> &b)
                {
                    double dist_a = std::hypot(a.first - destination_x_, a.second - destination_y_);
                    double dist_b = std::hypot(b.first - destination_x_, b.second - destination_y_);
                    return dist_a < dist_b;
                });

            // processNewObstacle(*closest_obstacle_it, msg->header.stamp);
        }
    }

    void ObstacleMonitor::processNewObstacle(const std::pair<double, double> &obstacle_position, const rclcpp::Time &timestamp)
    {
        RCLCPP_INFO(this->get_logger(), "New obstacle detected near the destination. Processing...");

        // Stop the robot -> send the cancel request
        stopRobot();

        // Create a MovingObstacle instance
        MovingObstacle obstacle;
        obstacle.x = obstacle_position.first;
        obstacle.y = obstacle_position.second;
        obstacle.vx = 0.0; // Velocity is unknown here
        obstacle.vy = 0.0;
        obstacle.last_seen = timestamp;

        // Rotate towards the obstacle
        rotateTowardsObstacle(obstacle);
    }

    void ObstacleMonitor::rotateTowardsObstacle(const MovingObstacle &obstacle)
    {
        // Get robot's current pose
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
            return;
        }

        double robot_x = transform_stamped.transform.translation.x;
        double robot_y = transform_stamped.transform.translation.y;
        double robot_yaw = tf2::getYaw(transform_stamped.transform.rotation);

        // Compute angle to obstacle
        double dx = obstacle.x - robot_x;
        double dy = obstacle.y - robot_y;
        double target_angle = std::atan2(dy, dx);

        double angle_difference = target_angle - robot_yaw;

        // Normalize angle -> principle value (-pi to +pi)
        while (angle_difference > M_PI)
            angle_difference -= 2 * M_PI;
        while (angle_difference < -M_PI)
            angle_difference += 2 * M_PI;

        // Rotate robot
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.angular.z = (angle_difference > 0) ? 0.5 : -0.5;

        rclcpp::Rate rate(10);
        double rotated_angle = 0.0;
        rclcpp::Time start_time = this->now();

        while (std::abs(rotated_angle) < std::abs(angle_difference) && rclcpp::ok())
        {
            cmd_vel_pub_->publish(cmd_vel);
            rate.sleep();

            // Update rotated angle
            rclcpp::Time current_time = this->now();
            double dt = (current_time - start_time).seconds();
            rotated_angle = cmd_vel.angular.z * dt;
        }

        // Stop rotation
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    void ObstacleMonitor::stopRobot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    rcl_interfaces::msg::SetParametersResult ObstacleMonitor::parameterCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for (const auto &param : parameters)
        {
            if (param.get_name() == "activate_costmap_comparison")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
                {
                    activate_costmap_comparison_ = param.as_bool();

                    if (activate_costmap_comparison_)
                    {
                        // Subscribe to costmap topics
                        global_costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
                            "/global_costmap/costmap_raw", rclcpp::QoS(10),
                            std::bind(&ObstacleMonitor::globalCostmapCallback, this, std::placeholders::_1));

                        local_costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
                            "/local_costmap/costmap_raw", rclcpp::QoS(10),
                            std::bind(&ObstacleMonitor::localCostmapCallback, this, std::placeholders::_1));

                        RCLCPP_INFO(this->get_logger(), "Subscribed to costmap topics for comparison.");
                    }
                    else
                    {
                        // Unsubscribe from costmap topics
                        if (global_costmap_sub_)
                            global_costmap_sub_.reset();
                        if (local_costmap_sub_)
                            local_costmap_sub_.reset();
                        RCLCPP_INFO(this->get_logger(), "Unsubscribed from costmap topics.");
                    }
                }
                else
                {
                    result.successful = false;
                    result.reason = "activate_costmap_comparison must be a boolean.";
                    return result;
                }
                break;
            }
        }
        result.successful = true;
        result.reason = "Parameters set successfully.";
        return result;
    }

} // namespace obstacle_monitoring
