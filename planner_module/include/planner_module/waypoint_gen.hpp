#ifndef DOOR_PATH_ANALYSIS_HPP
#define DOOR_PATH_ANALYSIS_HPP

#include <vector>
#include <string>
#include <tuple>
#include <cmath>
#include <limits>
#include <algorithm>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace waypointGen
{
    struct Door
    {
        std::string name;
        geometry_msgs::msg::Point position;
    };

    class DoorPathAnalyzer
    {
    public:
        explicit DoorPathAnalyzer(double threshold, const std::vector<geometry_msgs::msg::Pose> &path);
        // Compute the sequence of doors that the path passes through.
        std::vector<std::pair<std::string, int>> computePassingOrder();
        std::vector<Door> doors_;

    private:
        double threshold_;
        std::vector<geometry_msgs::msg::Pose> path_;

        std::tuple<double, int> computeMinDistanceToPath(const geometry_msgs::msg::Point &p);

        inline double distToWaypoint(const geometry_msgs::msg::Point &door, const geometry_msgs::msg::Pose &waypoint) const
        {
            double dx = door.x - waypoint.position.x;
            double dy = door.y - waypoint.position.y;
            double dz = door.z - waypoint.position.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        std::vector<Door> loadDoorsFromYaml(const std::string &filename);
        std::tuple<double, int> refineMinDistanceInRange(const geometry_msgs::msg::Point &p, int left, int right);
    };

    class TablePathAnalyzer
    {
    private:
        std::vector<geometry_msgs::msg::PoseStamped> table_corners;

    public:
        geometry_msgs::msg::PoseStamped P1, P2;
        TablePathAnalyzer(const std::string &filename, double dis);
    };

    class TableWayPointGen
    {
    private:
        inline double distToWaypoint(const geometry_msgs::msg::Point &door, const geometry_msgs::msg::Point &waypoint) const
        {
            double dx = door.x - waypoint.x;
            double dy = door.y - waypoint.y;
            double dz = door.z - waypoint.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

    public:
        TableWayPointGen();
        geometry_msgs::msg::Vector3 ComputeDesiredForwardDistance(const std::vector<geometry_msgs::msg::Point> &table_vertices,
                                             const geometry_msgs::msg::Point &robot_position);
    };
}

#endif
