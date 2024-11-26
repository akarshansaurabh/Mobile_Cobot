#include "obstacle/moving_obstacles.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<obstacleMonitoring::ObstacleMonitor>());
    rclcpp::shutdown();
    return 0;
}
