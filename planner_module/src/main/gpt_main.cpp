#include "obstacle/gpt.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<obstacle_monitoring::ObstacleMonitor>());
    rclcpp::shutdown();
    return 0;
}
