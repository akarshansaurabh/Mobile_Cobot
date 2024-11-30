#ifndef OBSTACLE_MONITOR_HPP_
#define OBSTACLE_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <deque>
#include <vector>
#include <atomic>
#include <condition_variable>
#include <algorithm>
#include <cmath>

namespace obstacle_monitoring
{

    enum class OperatingMode
    {
        Idle,
        Stationary,
        Moving
    };

    struct MovingObstacle
    {
        double x;
        double y;
        double vx;
        double vy;
        rclcpp::Time last_seen;
    };

    class ObstacleMonitor : public rclcpp::Node
    {
    public:
        ObstacleMonitor();

    private:
        // Operating mode management
        OperatingMode operating_mode_;
        void switchOperatingMode(OperatingMode mode);
        void updateOperatingMode();

        // TF buffer and listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::string robot_base_frame_;
        std::string global_frame_;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
        rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_costmap_sub_;
        rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_sub_;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        // Costmap data
        std::unique_ptr<nav2_costmap_2d::Costmap2D> global_costmap_;
        std::unique_ptr<nav2_costmap_2d::Costmap2D> local_costmap_;
        std::atomic<bool> is_global_costmap_initialized_;
        bool is_local_costmap_initialized_;

        // Laser scan processing
        void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void detectMovingObstacles();
        std::deque<sensor_msgs::msg::LaserScan::SharedPtr> laser_scan_buffer_;
        int laser_scan_buffer_size_;
        double scan_difference_threshold_;
        double detection_radius_;
        std::vector<MovingObstacle> detected_obstacles_;
        std::mutex laser_mutex_;
        std::atomic<bool> new_obstacle_detected_;

        // Costmap callbacks
        void globalCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
        void localCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
        void processNewObstacle(const std::pair<double, double> &obstacle_position, const rclcpp::Time &timestamp);

        // Parameter callback
        OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
        rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters);
        std::atomic<bool> activate_costmap_comparison_;

        // Timer for mode updates
        rclcpp::TimerBase::SharedPtr mode_update_timer_;

        // Laser scan subscription management
        void manageLaserSubscription();
        std::atomic<bool> laser_subscribed_;
        std::mutex laser_sub_mutex_;
        std::condition_variable laser_sub_cv_;
        rclcpp::TimerBase::SharedPtr laser_timer_;

        // Helper functions
        void stopRobot();
        void rotateTowardsObstacle(const MovingObstacle &obstacle);

        // Destination position
        double destination_x_;
        double destination_y_;

        // Destination radius
        double destination_radius_;

        // Thread safety
        std::mutex costmap_mutex_;
    };

} // namespace obstacle_monitoring

#endif // OBSTACLE_MONITOR_HPP_
