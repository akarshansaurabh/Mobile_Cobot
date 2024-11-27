#include "obstacle/dynamic_obstacle_monitoring.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace obstacle_monitoring
{
    ObstacleMonitor::ObstacleMonitor()
        : Node("obstacle_monitor_node"),
          robot_base_frame_("base_link"),
          global_frame_("map"),
          local_frame_("odom"),
          detection_radius_(2.0),
          operating_mode_("moving_robot"), // Default mode
          is_global_costmap_initialized_(false),
          is_reference_local_costmap_initialized_(false)
    {
        // Declare and get parameters
        this->declare_parameter("operating_mode", operating_mode_);
        this->get_parameter("operating_mode", operating_mode_);

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to costmap topics based on operating mode
        if (operating_mode_ == "moving_robot")
        {
            // Subscribe to global and local costmaps
            global_costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
                "/global_costmap/costmap_raw", rclcpp::QoS(10),
                std::bind(&ObstacleMonitor::globalCostmapCallback, this, std::placeholders::_1));

            local_costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
                "/local_costmap/costmap_raw", rclcpp::QoS(10),
                std::bind(&ObstacleMonitor::localCostmapCallback, this, std::placeholders::_1));
        }
        else if (operating_mode_ == "stationary_robot")
        {
            // Subscribe only to local costmap
            local_costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
                "/local_costmap/costmap_raw", rclcpp::QoS(10),
                std::bind(&ObstacleMonitor::localCostmapCallback, this, std::placeholders::_1));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid operating mode: %s", operating_mode_.c_str());
            throw std::runtime_error("Invalid operating mode");
        }

        RCLCPP_INFO(this->get_logger(), "Obstacle Monitor Node Initialized in %s mode.", operating_mode_.c_str());
    }

    void ObstacleMonitor::globalCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        // Initialize global costmap
        global_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
            msg->metadata.size_x, msg->metadata.size_y,
            msg->metadata.resolution,
            msg->metadata.origin.position.x,
            msg->metadata.origin.position.y);

        // Copy data to global costmap
        unsigned char *global_char_map = global_costmap_->getCharMap();
        std::copy(msg->data.begin(), msg->data.end(), global_char_map);

        is_global_costmap_initialized_ = true;
    }

    void ObstacleMonitor::localCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        if (operating_mode_ == "moving_robot")
        {
            if (!is_global_costmap_initialized_)
            {
                RCLCPP_WARN(this->get_logger(), "Global costmap not initialized yet.");
                return;
            }

            // Initialize current local costmap
            current_local_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
                msg->metadata.size_x, msg->metadata.size_y,
                msg->metadata.resolution,
                msg->metadata.origin.position.x,
                msg->metadata.origin.position.y);

            // Copy data to current local costmap
            unsigned char *local_char_map = current_local_costmap_->getCharMap();
            std::copy(msg->data.begin(), msg->data.end(), local_char_map);

            // Identify new obstacles
            std::vector<std::pair<double, double>> new_obstacle_positions;

            // Iterate over the current local costmap
            for (unsigned int mx = 0; mx < current_local_costmap_->getSizeInCellsX(); ++mx)
            {
                for (unsigned int my = 0; my < current_local_costmap_->getSizeInCellsY(); ++my)
                {
                    unsigned char cost = current_local_costmap_->getCost(mx, my);
                    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
                    {
                        double wx, wy;
                        current_local_costmap_->mapToWorld(mx, my, wx, wy);

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

            // Process new obstacles
            if (!new_obstacle_positions.empty())
            {
                processNewObstacles(new_obstacle_positions, msg->header.stamp);
            }
        }
        else if (operating_mode_ == "stationary_robot")
        {
            // Initialize reference local costmap if not already done
            if (!is_reference_local_costmap_initialized_)
            {
                reference_local_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
                    msg->metadata.size_x, msg->metadata.size_y,
                    msg->metadata.resolution,
                    msg->metadata.origin.position.x,
                    msg->metadata.origin.position.y);

                // Copy data to reference local costmap
                unsigned char *ref_char_map = reference_local_costmap_->getCharMap();
                std::copy(msg->data.begin(), msg->data.end(), ref_char_map);

                is_reference_local_costmap_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Reference local costmap initialized.");
                return;
            }

            // Initialize current local costmap
            current_local_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
                msg->metadata.size_x, msg->metadata.size_y,
                msg->metadata.resolution,
                msg->metadata.origin.position.x,
                msg->metadata.origin.position.y);

            // Copy data to current local costmap
            unsigned char *current_char_map = current_local_costmap_->getCharMap();
            std::copy(msg->data.begin(), msg->data.end(), current_char_map);

            // Identify new obstacles
            std::vector<std::pair<double, double>> new_obstacle_positions;

            unsigned char *ref_char_map = reference_local_costmap_->getCharMap();

            // Iterate over the current local costmap
            for (unsigned int index = 0; index < current_local_costmap_->getSizeInCellsX() * current_local_costmap_->getSizeInCellsY(); ++index)
            {
                unsigned char current_cost = current_char_map[index];
                unsigned char ref_cost = ref_char_map[index];

                if ((current_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) &&
                    (ref_cost < nav2_costmap_2d::LETHAL_OBSTACLE))
                {
                    unsigned int mx = index % current_local_costmap_->getSizeInCellsX();
                    unsigned int my = index / current_local_costmap_->getSizeInCellsX();

                    double wx, wy;
                    current_local_costmap_->mapToWorld(mx, my, wx, wy);

                    // No need to transform since the robot is stationary
                    new_obstacle_positions.emplace_back(wx, wy);
                }
            }

            // Process new obstacles
            if (!new_obstacle_positions.empty())
            {
                processNewObstacles(new_obstacle_positions, msg->header.stamp);
            }
        }
    }

    void ObstacleMonitor::processNewObstacles(const std::vector<std::pair<double, double>> &new_obstacle_positions, const rclcpp::Time &stamp)
    {
        // Get robot's current position
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, stamp, tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
            return;
        }

        double robot_x = transform_stamped.transform.translation.x;
        double robot_y = transform_stamped.transform.translation.y;

        double detection_radius_squared = detection_radius_ * detection_radius_;

        // Check if any new obstacle is within the detection radius
        for (const auto &pos : new_obstacle_positions)
        {
            double dx = pos.first - robot_x;
            double dy = pos.second - robot_y;
            double distance_squared = dx * dx + dy * dy;

            if (distance_squared <= detection_radius_squared)
            {
                // Trigger depth camera processing
                RCLCPP_INFO(this->get_logger(), "New obstacle detected within radius. Triggering depth camera.");
                // TODO: Implement depth camera activation and processing logic here

                // Early exit after triggering
                return;
            }
        }
    }
} // namespace obstacle_monitoring














// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************












// #ifndef OBSTACLE_MONITOR_HPP
// #define OBSTACLE_MONITOR_HPP

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <deque>
// #include <vector>
// #include <string>

// namespace obstacle_monitoring {

// enum class OperatingMode { Idle, Stationary, Moving };

// struct MovingObstacle {
//     double x;
//     double y;
//     double vx;
//     double vy;
//     rclcpp::Time last_seen;
// };

// class ObstacleMonitor : public rclcpp::Node {
// public:
//     ObstacleMonitor();

// private:
//     // Operating mode
//     OperatingMode operating_mode_;

//     // TF
//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
//     std::string robot_base_frame_;
//     std::string global_frame_;

//     // Subscribers
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_status_sub_;

//     // Publishers
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
//     rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

//     // Laser scan processing
//     void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
//     void detectMovingObstacles();
//     std::deque<sensor_msgs::msg::LaserScan::SharedPtr> laser_scan_buffer_;
//     size_t laser_scan_buffer_size_;
//     double scan_difference_threshold_;
//     double detection_radius_;
//     std::vector<MovingObstacle> detected_obstacles_;

//     // Odometry processing
//     void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
//     double movement_threshold_;
//     double angular_movement_threshold_;

//     // Goal processing
//     void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
//     geometry_msgs::msg::PoseStamped current_goal_;
//     double destination_radius_;

//     // Goal status processing
//     void goalStatusCallback(const std_msgs::msg::String::SharedPtr msg);
//     bool isObstacleDetectedByNav2();

//     // Operating mode management
//     void switchOperatingMode(OperatingMode mode);
//     void updateOperatingMode();

//     // Timer for mode updates
//     rclcpp::TimerBase::SharedPtr mode_update_timer_;

//     // Helper functions
//     bool isObstacleNearDestination(const MovingObstacle& obstacle);
//     void processObstacle(const MovingObstacle& obstacle);
//     void rotateTowardsObstacle(const MovingObstacle& obstacle);
//     void stopRobot();

//     // Robot state
//     bool robot_moving_;
//     rclcpp::Time last_movement_time_;
//     rclcpp::Time last_obstacle_time_;
//     double idle_timeout_;
// };

// } // namespace obstacle_monitoring

// #endif // OBSTACLE_MONITOR_HPP












// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************
// ******************************************************************************************************************************





// #include "obstacle_monitor.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <cmath>

// namespace obstacle_monitoring {

// ObstacleMonitor::ObstacleMonitor()
//     : Node("obstacle_monitor_node"),
//       operating_mode_(OperatingMode::Idle),
//       robot_base_frame_("base_link"),
//       global_frame_("map"),
//       laser_scan_buffer_size_(5),
//       scan_difference_threshold_(0.1),
//       detection_radius_(2.0),
//       movement_threshold_(0.1),
//       angular_movement_threshold_(0.1),
//       destination_radius_(1.0),
//       robot_moving_(false),
//       idle_timeout_(5.0)
// {
//     // Declare parameters
//     this->declare_parameter("robot_base_frame", robot_base_frame_);
//     this->declare_parameter("global_frame", global_frame_);
//     this->declare_parameter("laser_scan_buffer_size", laser_scan_buffer_size_);
//     this->declare_parameter("scan_difference_threshold", scan_difference_threshold_);
//     this->declare_parameter("detection_radius", detection_radius_);
//     this->declare_parameter("movement_threshold", movement_threshold_);
//     this->declare_parameter("angular_movement_threshold", angular_movement_threshold_);
//     this->declare_parameter("destination_radius", destination_radius_);
//     this->declare_parameter("idle_timeout", idle_timeout_);

//     // Get parameters
//     this->get_parameter("robot_base_frame", robot_base_frame_);
//     this->get_parameter("global_frame", global_frame_);
//     this->get_parameter("laser_scan_buffer_size", laser_scan_buffer_size_);
//     this->get_parameter("scan_difference_threshold", scan_difference_threshold_);
//     this->get_parameter("detection_radius", detection_radius_);
//     this->get_parameter("movement_threshold", movement_threshold_);
//     this->get_parameter("angular_movement_threshold", angular_movement_threshold_);
//     this->get_parameter("destination_radius", destination_radius_);
//     this->get_parameter("idle_timeout", idle_timeout_);

//     // Initialize TF buffer and listener
//     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//     // Publishers
//     cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//     goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

//     // Subscribers
//     goal_status_sub_ = create_subscription<std_msgs::msg::String>(
//         "/goal_status", rclcpp::QoS(10),
//         std::bind(&ObstacleMonitor::goalStatusCallback, this, std::placeholders::_1));

//     goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//         "/goal_pose", rclcpp::QoS(10),
//         std::bind(&ObstacleMonitor::goalCallback, this, std::placeholders::_1));

//     // Timer for updating operating mode
//     mode_update_timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(500),
//         std::bind(&ObstacleMonitor::updateOperatingMode, this));

//     RCLCPP_INFO(this->get_logger(), "Obstacle Monitor Node Initialized in Idle mode.");
// }

// void ObstacleMonitor::switchOperatingMode(OperatingMode mode) {
//     if (operating_mode_ == mode) {
//         return;
//     }

//     // Unsubscribe from existing topics to stop callbacks
//     laser_scan_sub_.reset();
//     odometry_sub_.reset();

//     operating_mode_ = mode;

//     std::string mode_str = (mode == OperatingMode::Idle) ? "Idle" :
//                            (mode == OperatingMode::Stationary) ? "Stationary" : "Moving";

//     RCLCPP_INFO(this->get_logger(), "Switching to %s mode.", mode_str.c_str());

//     if (operating_mode_ == OperatingMode::Stationary) {
//         // Subscribe to topics needed in Stationary mode
//         laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", rclcpp::QoS(10),
//             std::bind(&ObstacleMonitor::laserScanCallback, this, std::placeholders::_1));

//         odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
//             "/odom", rclcpp::QoS(10),
//             std::bind(&ObstacleMonitor::odometryCallback, this, std::placeholders::_1));
//     }

//     // Reset relevant data
//     detected_obstacles_.clear();
//     laser_scan_buffer_.clear();
// }

// void ObstacleMonitor::updateOperatingMode() {
//     rclcpp::Time now = this->now();

//     // Operating mode is managed based on goal status and obstacle detection
//     // No automatic transitions here
//     // Additional logic can be added if needed
// }

// void ObstacleMonitor::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//     laser_scan_buffer_.push_back(msg);

//     if (laser_scan_buffer_.size() >= laser_scan_buffer_size_) {
//         detectMovingObstacles();
//         laser_scan_buffer_.pop_front();
//     }
// }

// void ObstacleMonitor::detectMovingObstacles() {
//     auto current_scan = laser_scan_buffer_.back();
//     auto previous_scan = laser_scan_buffer_.front();

//     // Ensure scans are compatible
//     if (current_scan->ranges.size() != previous_scan->ranges.size()) {
//         RCLCPP_WARN(this->get_logger(), "Laser scans have different sizes.");
//         return;
//     }

//     // Get robot's current position in map frame
//     geometry_msgs::msg::TransformStamped robot_transform;
//     try {
//         robot_transform = tf_buffer_->lookupTransform(
//             global_frame_, robot_base_frame_, current_scan->header.stamp, tf2::durationFromSec(0.1));
//     } catch (tf2::TransformException &ex) {
//         RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
//         return;
//     }

//     double robot_x = robot_transform.transform.translation.x;
//     double robot_y = robot_transform.transform.translation.y;

//     std::vector<MovingObstacle> new_obstacles;

//     // Compare scans
//     for (size_t i = 0; i < current_scan->ranges.size(); ++i) {
//         double range_current = current_scan->ranges[i];
//         double range_previous = previous_scan->ranges[i];

//         if (std::isfinite(range_current) && std::isfinite(range_previous)) {
//             double range_difference = std::abs(range_current - range_previous);
//             if (range_difference > scan_difference_threshold_) {
//                 // Potential moving obstacle detected
//                 double angle = current_scan->angle_min + i * current_scan->angle_increment;
//                 double x = range_current * cos(angle);
//                 double y = range_current * sin(angle);

//                 // Transform to global frame
//                 geometry_msgs::msg::PointStamped local_point, global_point;
//                 local_point.header.frame_id = current_scan->header.frame_id;
//                 local_point.header.stamp = current_scan->header.stamp;
//                 local_point.point.x = x;
//                 local_point.point.y = y;
//                 local_point.point.z = 0.0;

//                 try {
//                     tf_buffer_->transform(local_point, global_point, global_frame_, tf2::durationFromSec(0.1));

//                     // Calculate distance between obstacle and robot
//                     double dx = global_point.point.x - robot_x;
//                     double dy = global_point.point.y - robot_y;
//                     double distance = std::hypot(dx, dy);

//                     if (distance <= detection_radius_) {
//                         MovingObstacle obstacle;
//                         obstacle.x = global_point.point.x;
//                         obstacle.y = global_point.point.y;
//                         obstacle.last_seen = this->now();

//                         // Estimate velocity
//                         double time_diff = (current_scan->header.stamp - previous_scan->header.stamp).seconds();
//                         obstacle.vx = (range_current * cos(angle) - range_previous * cos(angle)) / time_diff;
//                         obstacle.vy = (range_current * sin(angle) - range_previous * sin(angle)) / time_diff;

//                         new_obstacles.push_back(obstacle);
//                     }
//                 } catch (tf2::TransformException &ex) {
//                     RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
//                     continue;
//                 }
//             }
//         }
//     }

//     if (!new_obstacles.empty()) {
//         detected_obstacles_ = new_obstacles;
//         last_obstacle_time_ = this->now();

//         // Process obstacles near destination
//         std::vector<MovingObstacle> obstacles_near_destination;
//         for (const auto& obstacle : detected_obstacles_) {
//             if (isObstacleNearDestination(obstacle)) {
//                 obstacles_near_destination.push_back(obstacle);
//             }
//         }

//         if (!obstacles_near_destination.empty()) {
//             // Find obstacle closest to the destination
//             auto closest_obstacle = std::min_element(
//                 obstacles_near_destination.begin(), obstacles_near_destination.end(),
//                 [this](const MovingObstacle& a, const MovingObstacle& b) {
//                     double da = std::hypot(a.x - current_goal_.pose.position.x, a.y - current_goal_.pose.position.y);
//                     double db = std::hypot(b.x - current_goal_.pose.position.x, b.y - current_goal_.pose.position.y);
//                     return da < db;
//                 });

//             processObstacle(*closest_obstacle);
//         } else {
//             // No obstacles near destination, re-send previous goal
//             RCLCPP_INFO(this->get_logger(), "No obstacles near destination. Resuming navigation.");
//             goal_pub_->publish(current_goal_);
//             switchOperatingMode(OperatingMode::Moving);
//         }
//     } else {
//         // No moving obstacles detected
//     }
// }

// void ObstacleMonitor::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     double linear_velocity = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
//     double angular_velocity = std::abs(msg->twist.twist.angular.z);

//     if (linear_velocity > movement_threshold_ || angular_velocity > angular_movement_threshold_) {
//         robot_moving_ = true;
//         last_movement_time_ = this->now();
//     } else {
//         robot_moving_ = false;
//     }
// }

// void ObstacleMonitor::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//     current_goal_ = *msg;
// }

// void ObstacleMonitor::goalStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
//     if (msg->data == "ACTIVE") {
//         switchOperatingMode(OperatingMode::Moving);
//     } else if (msg->data == "SUCCEEDED" || msg->data == "CANCELED") {
//         switchOperatingMode(OperatingMode::Idle);
//     } else if (msg->data == "ABORTED") {
//         // Determine if the abort was due to a dynamic obstacle
//         if (isObstacleDetectedByNav2()) {
//             // Cancel the goal via action client (assumed to be handled externally)
//             switchOperatingMode(OperatingMode::Stationary);
//         } else {
//             switchOperatingMode(OperatingMode::Idle);
//         }
//     }
// }

// bool ObstacleMonitor::isObstacleDetectedByNav2() {
//     // Implement logic to determine if Nav2 detected an obstacle causing the goal to abort
//     // This could involve checking feedback messages or status codes
//     // For now, we'll assume any abort is due to a dynamic obstacle
//     return true;
// }

// bool ObstacleMonitor::isObstacleNearDestination(const MovingObstacle& obstacle) {
//     double dx = obstacle.x - current_goal_.pose.position.x;
//     double dy = obstacle.y - current_goal_.pose.position.y;
//     double distance = std::hypot(dx, dy);
//     return distance <= destination_radius_;
// }

// void ObstacleMonitor::processObstacle(const MovingObstacle& obstacle) {
//     RCLCPP_INFO(this->get_logger(), "Obstacle near destination detected. Processing...");

//     // Stop the robot
//     stopRobot();

//     // Rotate towards the obstacle
//     rotateTowardsObstacle(obstacle);

//     // Activate depth camera (placeholder)
//     RCLCPP_INFO(this->get_logger(), "Activating depth camera for obstacle at (%f, %f)", obstacle.x, obstacle.y);

//     // Further processing can be added here
// }

// void ObstacleMonitor::rotateTowardsObstacle(const MovingObstacle& obstacle) {
//     // Get robot's current pose
//     geometry_msgs::msg::TransformStamped transform_stamped;
//     try {
//         transform_stamped = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
//     } catch (tf2::TransformException &ex) {
//         RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
//         return;
//     }

//     double robot_x = transform_stamped.transform.translation.x;
//     double robot_y = transform_stamped.transform.translation.y;
//     double robot_yaw = tf2::getYaw(transform_stamped.transform.rotation);

//     // Compute angle to obstacle
//     double dx = obstacle.x - robot_x;
//     double dy = obstacle.y - robot_y;
//     double target_angle = std::atan2(dy, dx);

//     double angle_difference = target_angle - robot_yaw;

//     // Normalize angle
//     while (angle_difference > M_PI) angle_difference -= 2 * M_PI;
//     while (angle_difference < -M_PI) angle_difference += 2 * M_PI;

//     // Rotate robot
//     geometry_msgs::msg::Twist cmd_vel;
//     cmd_vel.angular.z = (angle_difference > 0) ? 0.5 : -0.5;

//     rclcpp::Rate rate(10);
//     double rotated_angle = 0.0;
//     rclcpp::Time start_time = this->now();

//     while (std::abs(rotated_angle) < std::abs(angle_difference) && rclcpp::ok()) {
//         cmd_vel_pub_->publish(cmd_vel);
//         rate.sleep();

//         // Update rotated angle
//         rclcpp::Time current_time = this->now();
//         double dt = (current_time - start_time).seconds();
//         rotated_angle = cmd_vel.angular.z * dt;
//     }

//     // Stop rotation
//     cmd_vel.angular.z = 0.0;
//     cmd_vel_pub_->publish(cmd_vel);
// }

// void ObstacleMonitor::stopRobot() {
//     geometry_msgs::msg::Twist cmd_vel;
//     cmd_vel.linear.x = 0.0;
//     cmd_vel.angular.z = 0.0;
//     cmd_vel_pub_->publish(cmd_vel);
// }

// } // namespace obstacle_monitoring

