#include "planner_module/waypoint_gen.hpp"
#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace std;

namespace waypointGen
{
    DoorPathAnalyzer::DoorPathAnalyzer(double threshold, const std::vector<geometry_msgs::msg::Pose> &path)
        : threshold_(threshold), path_(path)
    {
        std::string filename = "/home/akarshan/mobile_cobot_ws/src/r1d1_description/config/locations.yaml";
        doors_ = loadDoorsFromYaml(filename);
    }

    std::vector<std::pair<std::string, int>> DoorPathAnalyzer::computePassingOrder()
    {
        std::vector<std::pair<std::string, int>> passingDoors;

        for (const auto &d : doors_)
        {
            auto [minDist, bestIdx] = computeMinDistanceToPath(d.position);
            if (minDist <= threshold_)
                passingDoors.emplace_back(d.name, bestIdx);
        }

        // Sort by bestIdx so that we know the sequence in which doors are encountered
        std::sort(passingDoors.begin(), passingDoors.end(),
                  [](const auto &d1, const auto &d2)
                  {
                      return d1.second < d2.second;
                  });

        return passingDoors;
    }

    std::tuple<double, int> DoorPathAnalyzer::computeMinDistanceToPath(const geometry_msgs::msg::Point &p)
    {
        if (path_.empty())
            return {std::numeric_limits<double>::infinity(), -1};
        if (path_.size() == 1)
        {
            double d = distToWaypoint(p, path_[0]);
            return {d, 0};
        }

        int left = 0;
        int right = static_cast<int>(path_.size()) - 1;

        while (right - left > 2)
        {
            int mid = (left + right) / 2;
            double distLeft = distToWaypoint(p, path_[left]);
            double distMid = distToWaypoint(p, path_[mid]);
            double distRight = distToWaypoint(p, path_[right]);

            if ((distLeft + distMid) < (distMid + distRight))
                right = mid;
            else
                left = mid;
        }

        // Now [left, right] is a small range (at most 3 elements)
        auto [finalDist, finalIdx] = refineMinDistanceInRange(p, left, right);
        return {finalDist, finalIdx};
    }

    std::tuple<double, int> DoorPathAnalyzer::refineMinDistanceInRange(const geometry_msgs::msg::Point &p, int left, int right)
    {
        double minDist = std::numeric_limits<double>::infinity();
        int bestIdx = -1;

        if (left == right)
        {
            double d = distToWaypoint(p, path_[left]);
            return {d, left};
        }

        for (int i = left; i < right; i++)
        {
            double d = distToWaypoint(p, path_[i]);
            if (d < minDist)
            {
                minDist = d;
                bestIdx = i;
            }
        }
        return {minDist, bestIdx};
    }

    std::vector<Door> DoorPathAnalyzer::loadDoorsFromYaml(const std::string &filename)
    {
        std::vector<waypointGen::Door> doors;

        YAML::Node config = YAML::LoadFile(filename);

        if (!config["locations"])
            return doors;

        YAML::Node locations = config["locations"];
        const std::vector<std::string> doorNames = {"door_A", "door_B", "door_C", "door_D", "door_E"};
        Door door;
        for (const auto &doorName : doorNames)
        {
            YAML::Node doorNode = locations[doorName];

            door.name = doorName;
            door.position.x = doorNode["position"]["x"].as<double>();
            door.position.y = doorNode["position"]["y"].as<double>();
            door.position.z = doorNode["position"]["z"].as<double>();
            doors.push_back(std::move(door));
        }
        return doors;
    }

    TablePathAnalyzer::TablePathAnalyzer(const std::string &filename, double dis)
    {
        // populate table_corners
        YAML::Node config = YAML::LoadFile(filename);
        geometry_msgs::msg::PoseStamped pose;

        YAML::Node locations = config["locations"];
        const std::vector<std::string> tableCorners = {"table_A", "table_B", "table_C", "table_D"};

        for (const auto &corner : tableCorners)
        {
            YAML::Node cornerNode = locations[corner];

            pose.pose.position.x = cornerNode["position"]["x"].as<double>();
            pose.pose.position.y = cornerNode["position"]["y"].as<double>();
            pose.pose.position.z = cornerNode["position"]["z"].as<double>();
            pose.pose.orientation.x = cornerNode["orientation"]["x"].as<double>();
            pose.pose.orientation.y = cornerNode["orientation"]["y"].as<double>();
            pose.pose.orientation.z = cornerNode["orientation"]["z"].as<double>();
            pose.pose.orientation.w = cornerNode["orientation"]["w"].as<double>();
            table_corners.push_back(std::move(pose));
        }
        geometry_msgs::msg::PoseStamped M1, M2;
        // compute P1 and P2
        P1.pose.position.x = M1.pose.position.x = (table_corners[0].pose.position.x + table_corners[1].pose.position.x) / 2.0;
        P1.pose.position.y = M1.pose.position.y = (table_corners[0].pose.position.y + table_corners[1].pose.position.y) / 2.0;
        P1.pose.position.z = M1.pose.position.z = (table_corners[0].pose.position.z + table_corners[1].pose.position.z) / 2.0;
        P2.pose.position.x = M2.pose.position.x = (table_corners[2].pose.position.x + table_corners[3].pose.position.x) / 2.0;
        P2.pose.position.y = M2.pose.position.y = (table_corners[2].pose.position.y + table_corners[3].pose.position.y) / 2.0;
        P2.pose.position.z = M2.pose.position.z = (table_corners[2].pose.position.z + table_corners[3].pose.position.z) / 2.0;

        P1.pose.position.x -= dis;
        P2.pose.position.y -= dis;

        P1.pose.orientation.x = 0.0;
        P1.pose.orientation.y = 0.0;
        P1.pose.orientation.z = 0.0;
        P1.pose.orientation.w = 1.0;

        P2.pose.orientation.x = 0.0;
        P2.pose.orientation.y = 0.0;
        P2.pose.orientation.z = 0.707;
        P2.pose.orientation.w = 0.707;

        P1.header.frame_id = P2.header.frame_id = "map";
    }

    geometry_msgs::msg::Vector3 TableWayPointGen::ComputeDesiredForwardDistance(const std::vector<geometry_msgs::msg::Point> &table_vertices,
                                                                                const geometry_msgs::msg::Point &robot_position)
    {
        geometry_msgs::msg::Point mid_point;
        geometry_msgs::msg::Vector3 ans;
        double min_dis = 10000.0;
        Eigen::Vector3d dis_vector, direction;
        for (int i = 0; i < 4; i++)
        {
            mid_point.x = (table_vertices[i].x + table_vertices[(i == 3) ? 0 : i + 1].x) / 2.0;
            mid_point.y = (table_vertices[i].y + table_vertices[(i == 3) ? 0 : i + 1].y) / 2.0;
            mid_point.z = (table_vertices[i].z + table_vertices[(i == 3) ? 0 : i + 1].z) / 2.0;
            double dis = distToWaypoint(mid_point, robot_position);
            if (dis < min_dis)
            {
                min_dis = dis;
                dis_vector << (mid_point.x - robot_position.x),
                    (mid_point.y - robot_position.y),
                    (mid_point.z - robot_position.z);
            }
        }
        direction = dis_vector / dis_vector.norm();
        ans.x = 0.85 * min_dis * direction.x();
        ans.y = 0.85 * min_dis * direction.y();
        ans.z = 0.85 * min_dis * direction.z();
        std::cout << ans.x << " " << ans.y << " " << ans.z << std::endl;
        return ans;
    }

    TableWayPointGen::TableWayPointGen() {}
}