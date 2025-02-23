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
    enum class Shapes
    {
        CUBOID,
        CYLINDER
    };

    struct Shape3D
    {
        Shapes shape3d;
        geometry_msgs::msg::Pose pose;
        float r, g, b;
        double L, B, H, R, h;
    };

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
        void deleteMarkersInRange(int start_id, int end_id, const std::string &marker_namespace);
        void publishCuboidMarker(const Shape3D &shape_3d);
        void publishFilledPolygon(const std::vector<geometry_msgs::msg::Point> &polygon_points,
                                  float r, float g, float b, float alpha = 1.0f);
        void publishCylinderMarker(const Shape3D &shape_3d);
        void publishPeripheryLineStrip(const std::vector<geometry_msgs::msg::Point> &hull_points,
                                       float r, float g, float b);
        int id_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    private:
        void visualizeVector(const geometry_msgs::msg::Point &origin, const geometry_msgs::msg::Vector3 &vector, int id,
                             visualization_msgs::msg::MarkerArray &marker_array, float r, float g, float b);

        rclcpp::Node::SharedPtr node_;
        std::string target_frame_;
    };
}

#endif