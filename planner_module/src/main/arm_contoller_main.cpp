#include "planner_module/arm_contoller.hpp"

using namespace arm_planner;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("custom_arm_controller_node");
    arm_planner::ArmController action_client_obj(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
