#include "visualizations/visualization_manager.hpp"

namespace visualization
{
    VisualizationManager::VisualizationManager(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/box_marker_array", rclcpp::QoS(10));

        node_->get_parameter_or<std::string>("target_frame_", target_frame_, "map");
    }

    void VisualizationManager::publishMarkerArray(const std::vector<geometry_msgs::msg::Pose> &poses)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int current_marker_id = 0;

        for (const auto &pose : poses)
        {
            // Convert quaternion to rotation matrix
            Eigen::Quaternionf quat(pose.orientation.w,
                                    pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z);
            Eigen::Matrix3f rot_matrix = quat.toRotationMatrix();

            // Origin of the vectors
            geometry_msgs::msg::Point origin = pose.position;

            // Define vectors based on rotation matrix columns
            geometry_msgs::msg::Vector3 x_vector;
            x_vector.x = rot_matrix(0, 0);
            x_vector.y = rot_matrix(1, 0);
            x_vector.z = rot_matrix(2, 0);

            geometry_msgs::msg::Vector3 y_vector;
            y_vector.x = rot_matrix(0, 1);
            y_vector.y = rot_matrix(1, 1);
            y_vector.z = rot_matrix(2, 1);

            geometry_msgs::msg::Vector3 z_vector;
            z_vector.x = rot_matrix(0, 2);
            z_vector.y = rot_matrix(1, 2);
            z_vector.z = rot_matrix(2, 2);

            // X-axis (Red)
            visualizeVector(origin, x_vector, current_marker_id++, marker_array, 1.0f, 0.0f, 0.0f);

            // Y-axis (Green)
            visualizeVector(origin, y_vector, current_marker_id++, marker_array, 0.0f, 1.0f, 0.0f);

            // Z-axis (Blue)
            visualizeVector(origin, z_vector, current_marker_id++, marker_array, 0.0f, 0.0f, 1.0f);
        }

        // Publish the MarkerArray
        marker_pub_->publish(marker_array);
    }

    void VisualizationManager::visualizeVector(const geometry_msgs::msg::Point &origin, const geometry_msgs::msg::Vector3 &vector, int id,
                                               visualization_msgs::msg::MarkerArray &marker_array, float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = target_frame_;
        marker.header.stamp = node_->now();
        marker.ns = "vectors";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.points.resize(2);
        marker.points[0] = origin;
        marker.points[1].x = origin.x + vector.x * 0.4f;
        marker.points[1].y = origin.y + vector.y * 0.4f;
        marker.points[1].z = origin.z + vector.z * 0.4f;

        marker.scale.x = 0.05f;
        marker.scale.y = 0.1f;
        marker.scale.z = 0.1f;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration(0, 0);
        marker_array.markers.push_back(marker);
    }

    void VisualizationManager::publishTableVertices(const std::vector<geometry_msgs::msg::Point> &vertices)
    {
        visualization_msgs::msg::Marker table_marker;
        table_marker.header.frame_id = target_frame_;
        table_marker.header.stamp = node_->now();
        table_marker.ns = "table_vertices";
        table_marker.id = 0;
        table_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        table_marker.action = visualization_msgs::msg::Marker::ADD;

        table_marker.points.resize(vertices.size() + 1);
        for (int i = 0; i < vertices.size(); ++i)
            table_marker.points[i] = vertices[i];
        table_marker.points[vertices.size()] = vertices[0];

        table_marker.scale.x = 0.02f;

        table_marker.color.r = 0.0f;
        table_marker.color.g = 0.0f;
        table_marker.color.b = 1.0f;
        table_marker.color.a = 1.0f;

        table_marker.lifetime = rclcpp::Duration(0, 0);

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(table_marker);

        marker_pub_->publish(marker_array);
        RCLCPP_INFO(node_->get_logger(), "Published table vertices.");
    }
} // namespace visualization
