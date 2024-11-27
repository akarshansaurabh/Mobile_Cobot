#include "obstacle/moving_obstacles.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace obstacleMonitoring
{
    ObstacleMonitor::ObstacleMonitor()
        : Node("obstacle_monitor_node"),
          robot_base_frame_("base_link"),
          detection_radius_(2.0),
          is_static_costmap_initialized_(false),
          size_x_(0),
          size_y_(0)
    {
        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to the local costmap topic
        costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
            "/local_costmap/costmap_raw", rclcpp::QoS(10),
            std::bind(&ObstacleMonitor::costmapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Obstacle Monitor Node Initialized.");
    }

    /*
        populate the static and obstacle cost map using the nav2_msgs::msg::Costmap
        Find new obstacle by comparing the cost of each cell in static and obstacle cost map
        Find robot position and obstacle position in map frame
        If object is in the ROI, activate depth cam
    */
    void ObstacleMonitor::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        // Initialize static and obstacle costmaps
        if (!is_static_costmap_initialized_)
        {
            size_x_ = msg->metadata.size_x;
            size_y_ = msg->metadata.size_y;

            // Initialize static costmap
            static_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
                size_x_, size_y_,
                msg->metadata.resolution,
                msg->metadata.origin.position.x,
                msg->metadata.origin.position.y);

            // Copy data to static costmap
            unsigned int size = msg->data.size();
            unsigned char *static_char_map = static_costmap_->getCharMap();
            std::copy(msg->data.begin(), msg->data.end(), static_char_map);

            is_static_costmap_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Static costmap initialized.");
        }
        else
        {
            // Initialize obstacle costmap
            obstacle_costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
                size_x_, size_y_,
                msg->metadata.resolution,
                msg->metadata.origin.position.x,
                msg->metadata.origin.position.y);

            // Copy data to obstacle costmap
            unsigned int size = msg->data.size();
            unsigned char *obstacle_char_map = obstacle_costmap_->getCharMap();
            std::copy(msg->data.begin(), msg->data.end(), obstacle_char_map);

            // Identify new obstacles using a single loop
            std::vector<std::pair<unsigned int, unsigned int>> new_obstacle_cells;
            new_obstacle_cells.reserve(size / 10); // Reserve space assuming sparse obstacles

            unsigned char *static_char_map = static_costmap_->getCharMap();

            for (unsigned int index = 0; index < size; ++index)
            {
                unsigned char static_cost = static_char_map[index];
                unsigned char obstacle_cost = obstacle_char_map[index];
                // nav2_costmap_2d::Costmap2D::
                // Check if the cell is an obstacle in the obstacle layer but not in the static layer

                if ((obstacle_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) &&
                    (static_cost < nav2_costmap_2d::LETHAL_OBSTACLE))
                {
                    unsigned int mx = index % size_x_;
                    unsigned int my = index / size_x_;
                    new_obstacle_cells.emplace_back(mx, my);
                }
            }

            // Process new obstacles
            if (!new_obstacle_cells.empty())
            {
                processNewObstacles(new_obstacle_cells, msg->header.stamp, msg->header.frame_id);
            }
        }
    }

    void ObstacleMonitor::processNewObstacles(const std::vector<std::pair<unsigned int, unsigned int>> &new_obstacle_cells, const rclcpp::Time &stamp, const std::string &target_frame_id)
    {
        // Get robot's current position at the time of the costmap
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            // target, source, time
            // finds the transformation matrix of the source frame wrt target frame at given time stamp
            transform_stamped = tf_buffer_->lookupTransform(
                target_frame_id, robot_base_frame_, stamp,
                rclcpp::Duration::from_seconds(0.1));
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
        for (const auto &cell : new_obstacle_cells)
        {
            unsigned int mx = cell.first;
            unsigned int my = cell.second;

            double wx, wy;
            obstacle_costmap_->mapToWorld(mx, my, wx, wy);

            double dx = wx - robot_x;
            double dy = wy - robot_y;
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
}