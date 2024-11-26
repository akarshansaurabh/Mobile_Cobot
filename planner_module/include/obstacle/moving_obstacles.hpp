#ifndef OBSTACLE_MONITOR_HPP
#define OBSTACLE_MONITOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp> // for LETHAL_OBSTACLE

namespace obstacleMonitoring
{
    class ObstacleMonitor : public rclcpp::Node
    {
    public:
        ObstacleMonitor();
        ~ObstacleMonitor() = default;

    private:
        // Callback for costmap updates
        void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);

        // Method to process new obstacles
        void processNewObstacles(const std::vector<std::pair<unsigned int, unsigned int>> &new_obstacle_cells, const rclcpp::Time &stamp, const std::string &target_frame_id);

        // ROS interfaces
        rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;

        // Transform buffer and listener
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Costmaps
        std::unique_ptr<nav2_costmap_2d::Costmap2D> static_costmap_;
        std::unique_ptr<nav2_costmap_2d::Costmap2D> obstacle_costmap_;
        

        // Robot's frame
        std::string robot_base_frame_;

        // Radius threshold
        double detection_radius_;

        // Flag to indicate if the static costmap is initialized
        bool is_static_costmap_initialized_;

        // Size variables
        unsigned int size_x_;
        unsigned int size_y_;
    };
}
#endif
