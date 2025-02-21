#include "visualizations/visualization_manager.hpp"
#include <chrono>

namespace visualization
{
    VisualizationManager::VisualizationManager(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/box_marker_array", rclcpp::QoS(10));

        node_->get_parameter_or<std::string>("target_frame_", target_frame_, "map");
        this->id_ = 0;
    }

    void VisualizationManager::publishMarkerArray(const std::vector<geometry_msgs::msg::Pose> &poses)
    {
        deleteMarkersInRange(0, 75, "vectors");
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

            visualizeVector(origin, x_vector, current_marker_id++, marker_array, 1.0f, 0.0f, 0.0f);
            visualizeVector(origin, y_vector, current_marker_id++, marker_array, 0.0f, 1.0f, 0.0f);
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
        marker.points[1].x = origin.x + vector.x * 0.1;
        marker.points[1].y = origin.y + vector.y * 0.1;
        marker.points[1].z = origin.z + vector.z * 0.1;

        marker.scale.x = 0.0125f;
        marker.scale.y = 0.025f;
        marker.scale.z = 0.025f;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration(0, 0);
        marker_array.markers.push_back(marker);
    }

    void VisualizationManager::deleteMarkersInRange(int start_id, int end_id, const std::string &marker_namespace)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (int id = start_id; id <= end_id; ++id)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = target_frame_;
            marker.header.stamp = node_->now();

            // Must match the same namespace and ID used when creating them (e.g. "vectors")
            marker.ns = marker_namespace;
            marker.id = id;
            marker.action = visualization_msgs::msg::Marker::DELETE;

            // The type doesn't strictly matter for deletion, but you can set it to the same type
            // you used when creating. If unknown, you can leave it at 0 or any valid type.
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker_array.markers.push_back(marker);
        }

        // Publish all the delete commands
        marker_pub_->publish(marker_array);

        // RCLCPP_INFO(node_->get_logger(),
        //             "Deleted markers in namespace '%s' with IDs [%d..%d].",
        //             marker_namespace.c_str(), start_id, end_id);
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

    visualization_msgs::msg::MarkerArray VisualizationManager::createTriangleMarker(const std::shared_ptr<fcl::CollisionObjectf> &collision_object,
                                                                                    int id, const std::string &frame_id)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        auto start = std::chrono::high_resolution_clock::now();
        // 1) Get a *const* pointer to the underlying geometry
        const fcl::CollisionGeometryf *geometry = collision_object->getCollisionGeometry(); // returns const

        // 2) dynamic_cast to a *const* BVHModel
        //    because geometry is const
        auto bvh = dynamic_cast<const fcl::BVHModel<fcl::OBBRSS<float>> *>(geometry);

        if (!bvh)
        {
            // If it's not a BVHModel, we can't extract triangle data
            RCLCPP_WARN(node_->get_logger(),
                        "Collision geometry is not a BVHModel<fcl::OBBRSS<float>>. Skipping visualization.");
            return marker_array;
        }

        // 2) Create a Marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.ns = "fcl_debug";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        const fcl::Transform3f &transform = collision_object->getTransform();

        // Convert fcl::Transform3f to geometry_msgs::msg::Pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = transform.translation().x();
        pose.position.y = transform.translation().y();
        pose.position.z = transform.translation().z();
        Eigen::Matrix3f eigen_rot = transform.rotation();

        Eigen::Quaternionf quat(eigen_rot);
        quat.normalize(); // Ensure the quaternion is normalized

        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();

        marker.pose = pose;

        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // full opacity

        // 3) Fill in the geometry as triangles
        //    For TRIANGLE_LIST, every 3 points = 1 triangle
        marker.points.reserve(bvh->num_tris * 3);

        for (int f = 0; f < bvh->num_tris; f++)
        {
            auto idx0 = bvh->tri_indices[f][0];
            auto idx1 = bvh->tri_indices[f][1];
            auto idx2 = bvh->tri_indices[f][2];

            auto makePoint = [&](int i)
            {
                geometry_msgs::msg::Point p;
                p.x = bvh->vertices[i][0];
                p.y = bvh->vertices[i][1];
                p.z = bvh->vertices[i][2];
                return p;
            };

            marker.points.push_back(makePoint(idx0));
            marker.points.push_back(makePoint(idx1));
            marker.points.push_back(makePoint(idx2));
        }

        // 4) Wrap in a MarkerArray for publishing
        marker_array.markers.push_back(marker);
        // marker_pub_->publish(marker_array);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        // Output the duration in milliseconds
        // std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
        // RCLCPP_INFO(node_->get_logger(),
        //             "Published collision object for link with ID=%d.", id);
        return marker_array;
    }

    void VisualizationManager::publishPathWithOrientations(const std::vector<geometry_msgs::msg::Pose> &path)
    {
        if (path.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Empty path. Nothing to visualize.");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = target_frame_;
        line_strip.header.stamp = node_->now();
        line_strip.ns = "path_line_strip";
        line_strip.id = 100;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;

        for (auto &p : path)
        {
            geometry_msgs::msg::Point pt;
            pt.x = p.position.x;
            pt.y = p.position.y;
            pt.z = p.position.z;
            line_strip.points.push_back(pt);
        }

        line_strip.scale.x = 0.02f; // thickness of line
        line_strip.color.r = 1.0f;
        line_strip.color.g = 1.0f;
        line_strip.color.b = 0.0f; // yellow
        line_strip.color.a = 1.0f;

        line_strip.lifetime = rclcpp::Duration(0, 0);
        marker_array.markers.push_back(line_strip);

        int marker_id = 101; // next marker ID
        for (auto &pose : path)
        {
            // Convert pose.orientation to Eigen::Matrix3f
            Eigen::Quaternionf quat(pose.orientation.w,
                                    pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z);
            Eigen::Matrix3f rot_matrix = quat.toRotationMatrix();

            geometry_msgs::msg::Point origin = pose.position;
            // X-axis
            geometry_msgs::msg::Vector3 x_vec;
            x_vec.x = rot_matrix(0, 0);
            x_vec.y = rot_matrix(1, 0);
            x_vec.z = rot_matrix(2, 0);

            // Y-axis
            geometry_msgs::msg::Vector3 y_vec;
            y_vec.x = rot_matrix(0, 1);
            y_vec.y = rot_matrix(1, 1);
            y_vec.z = rot_matrix(2, 1);

            // Z-axis
            geometry_msgs::msg::Vector3 z_vec;
            z_vec.x = rot_matrix(0, 2);
            z_vec.y = rot_matrix(1, 2);
            z_vec.z = rot_matrix(2, 2);

            visualizeVector(origin, x_vec, marker_id++, marker_array, 1.0f, 0.0f, 0.0f);
            visualizeVector(origin, y_vec, marker_id++, marker_array, 0.0f, 1.0f, 0.0f);
            visualizeVector(origin, z_vec, marker_id++, marker_array, 0.0f, 0.0f, 1.0f);
        }

        marker_pub_->publish(marker_array);
        RCLCPP_INFO(node_->get_logger(), "Published path line strip and orientation frames.");
    }

    void VisualizationManager::publishCuboidMarker(const Shape3D &shape_3d)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = target_frame_;
        marker.header.stamp = node_->now();
        marker.ns = "cuboid";
        marker.id = this->id_;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = shape_3d.pose;
        marker.scale.x = shape_3d.L;
        marker.scale.y = shape_3d.B;
        marker.scale.z = shape_3d.H;
        marker.color.r = shape_3d.r;
        marker.color.g = shape_3d.g;
        marker.color.b = shape_3d.b;
        marker.color.a = 1.0;                     // Fully opaque
        marker.lifetime = rclcpp::Duration(0, 0); // Marker persists indefinitely

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
        this->id_++;
    }

    void VisualizationManager::publishCylinderMarker(const Shape3D &shape_3d)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = target_frame_;
        marker.header.stamp = node_->now();
        marker.ns = "cylinder";
        marker.id = this->id_;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = shape_3d.pose;
        marker.scale.x = (2.0 * shape_3d.R);
        marker.scale.y = (2.0 * shape_3d.R);
        marker.scale.z = shape_3d.h;
        marker.color.r = shape_3d.r;
        marker.color.g = shape_3d.g;
        marker.color.b = shape_3d.b;
        marker.color.a = 1.0;                     // Fully opaque
        marker.lifetime = rclcpp::Duration(0, 0); // Marker persists indefinitely

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
        this->id_++;
    }

}
