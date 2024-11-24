#include "planner_module/arm_contoller.hpp"

using namespace arm_planner;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto action_client_obj = std::make_shared<ArmController>();

    // Start spinning the action_client_obj in a separate thread
    // std::thread spinner([action_client_obj]()
    //                     { rclcpp::spin(action_client_obj); });
    // spinner.join();

    rclcpp::spin(action_client_obj);
    rclcpp::shutdown();
    return 0;
}
