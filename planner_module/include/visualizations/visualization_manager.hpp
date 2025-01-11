#ifndef VISUALIZATION_MANAGER_HPP
#define VISUALIZATION_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <Eigen/Dense>

// FCL
#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>

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
        visualization_msgs::msg::MarkerArray createTriangleMarker(const std::shared_ptr<fcl::CollisionObjectf> &collision_object,
                                                                  int id, const std::string &frame_id);
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    private:
        void visualizeVector(const geometry_msgs::msg::Point &origin, const geometry_msgs::msg::Vector3 &vector, int id,
                             visualization_msgs::msg::MarkerArray &marker_array, float r, float g, float b);

        rclcpp::Node::SharedPtr node_;
        std::string target_frame_;
    };
} // namespace visualization

#endif // VISUALIZATION_MANAGER_HPP
