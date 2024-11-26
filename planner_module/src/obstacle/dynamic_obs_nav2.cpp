#include "obstacle/dynamic_obs_nav2.hpp"
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
      is_global_costmap_initialized_(false),
      is_local_costmap_initialized_(false)
{
    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to the global and local costmap topics
    global_costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        "/global_costmap/costmap_raw", rclcpp::QoS(10),
        std::bind(&ObstacleMonitor::globalCostmapCallback, this, std::placeholders::_1));

    local_costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
        "/local_costmap/costmap_raw", rclcpp::QoS(10),
        std::bind(&ObstacleMonitor::localCostmapCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Obstacle Monitor Node Initialized.");
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
    if (!is_global_costmap_initialized_)
    {
        RCLCPP_WARN(this->get_logger(), "Global costmap not initialized yet.");
        return;
    }

    // Initialize local costmap
    local_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
        msg->metadata.size_x, msg->metadata.size_y,
        msg->metadata.resolution,
        msg->metadata.origin.position.x,
        msg->metadata.origin.position.y);

    // Copy data to local costmap
    unsigned char *local_char_map = local_costmap_->getCharMap();
    std::copy(msg->data.begin(), msg->data.end(), local_char_map);

    is_local_costmap_initialized_ = true;

    // Identify new obstacles
    std::vector<std::pair<double, double>> new_obstacle_positions;

    // Iterate over the local costmap
    for (unsigned int mx = 0; mx < local_costmap_->getSizeInCellsX(); ++mx)
    {
        for (unsigned int my = 0; my < local_costmap_->getSizeInCellsY(); ++my)
        {
            unsigned char cost = local_costmap_->getCost(mx, my);
            if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
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

void ObstacleMonitor::processNewObstacles(const std::vector<std::pair<double, double>> &new_obstacle_positions, const rclcpp::Time &stamp)
{
    // Get robot's current position in the global frame
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
