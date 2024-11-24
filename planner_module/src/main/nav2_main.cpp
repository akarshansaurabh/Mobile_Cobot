#include "navigation/nav2_client.hpp"

using namespace custom_Nav2ActionClient;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto action_client_obj = std::make_shared<Nav2ActionClient>();

    NavigateToPose::Goal goal_msg;
    goal_msg.pose.pose.position.x = 1.0;
    goal_msg.pose.pose.position.y = 1.4;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = action_client_obj->now();

    // Start spinning the action_client_obj in a separate thread
    std::thread spinner([action_client_obj]()
                        { rclcpp::spin(action_client_obj); });

    // Wait briefly to ensure the action_client_obj is spinning
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Now, call SetCostmapParameters
    // action_client_obj->SetCostmapParameters();
    action_client_obj->SetNav2Parameters();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    spinner.join();

    rclcpp::shutdown();
    return 0;
}
