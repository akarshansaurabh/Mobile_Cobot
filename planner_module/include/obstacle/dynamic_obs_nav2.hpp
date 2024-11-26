#ifndef OBSTACLE_MONITOR_HPP
#define OBSTACLE_MONITOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <vector>

namespace obstacle_monitoring
{
class ObstacleMonitor : public rclcpp::Node
{
public:
    ObstacleMonitor();
    ~ObstacleMonitor() = default;

private:
    // Callbacks for costmap updates
    void globalCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
    void localCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);

    // Method to process new obstacles
    void processNewObstacles(const std::vector<std::pair<double, double>> &new_obstacle_positions, const rclcpp::Time &stamp);

    // ROS interfaces
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_costmap_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_sub_;

    // Transform buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Costmaps
    std::unique_ptr<nav2_costmap_2d::Costmap2D> global_costmap_;
    std::unique_ptr<nav2_costmap_2d::Costmap2D> local_costmap_;

    // Robot's frame
    std::string robot_base_frame_;

    // Frames
    std::string global_frame_;
    std::string local_frame_;

    // Radius threshold
    double detection_radius_;

    // Flags to indicate if costmaps are initialized
    bool is_global_costmap_initialized_;
    bool is_local_costmap_initialized_;
};
} // namespace obstacle_monitoring

#endif // OBSTACLE_MONITOR_HPP
