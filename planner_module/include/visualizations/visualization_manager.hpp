#ifndef VISUALIZATION_MANAGER_HPP
#define VISUALIZATION_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <Eigen/Dense>

namespace visualization
{
    class VisualizationManager
    {
    public:
        explicit VisualizationManager(rclcpp::Node::SharedPtr node);
        ~VisualizationManager() = default;

        void publishMarkerArray(const std::vector<geometry_msgs::msg::Pose> &poses);
        void publishTableVertices(const std::vector<geometry_msgs::msg::Point> &vertices);
        void publishPathWithOrientations(const std::vector<geometry_msgs::msg::Pose> &path);

    private:
        void visualizeVector(const geometry_msgs::msg::Point &origin, const geometry_msgs::msg::Vector3 &vector, int id,
                             visualization_msgs::msg::MarkerArray &marker_array, float r, float g, float b);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        std::string target_frame_;
    };
} // namespace visualization

#endif // VISUALIZATION_MANAGER_HPP
