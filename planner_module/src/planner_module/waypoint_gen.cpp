#include "planner_module/waypoint_gen.hpp"

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
        // Refine by checking segments within this range precisely
        auto [finalDist, finalIdx] = refineMinDistanceInRange(p, left, right);
        return {finalDist, finalIdx};
    }

    std::tuple<double, int> DoorPathAnalyzer::refineMinDistanceInRange(const geometry_msgs::msg::Point &p, int left, int right)
    {
        // In this small range, we check all segments [i, i+1] and also the waypoints themselves
        // to find the true minimal distance. The minimal point might be inside a segment.

        double minDist = std::numeric_limits<double>::infinity();
        int bestIdx = -1;

        if (left == right)
        {
            double d = distToWaypoint(p, path_[left]);
            return {d, left};
        }

        // Check line segments between [left ... right]
        // Also consider that if left==right, no segment, just a single waypoint check
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

    std::vector<Door> loadDoorsFromYaml(const std::string &filename)
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
        pose.header.frame_id = "map";
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
        P1.pose.position.x = M1.pose.position.x = (table_corners[0].pose.position.x + table_corners[1].pose.position.x) / 2;
        P1.pose.position.y = M1.pose.position.y = (table_corners[0].pose.position.y + table_corners[1].pose.position.y) / 2;
        P1.pose.position.z = M1.pose.position.z = (table_corners[0].pose.position.z + table_corners[1].pose.position.z) / 2;
        P2.pose.position.x = M2.pose.position.x = (table_corners[2].pose.position.x + table_corners[3].pose.position.x) / 2;
        P2.pose.position.y = M2.pose.position.y = (table_corners[2].pose.position.y + table_corners[3].pose.position.y) / 2;
        P2.pose.position.z = M2.pose.position.z = (table_corners[2].pose.position.z + table_corners[3].pose.position.z) / 2;

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
    }
}