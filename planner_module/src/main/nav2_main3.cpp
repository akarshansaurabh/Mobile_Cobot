#include "navigation/nav2_client3.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("nav2_client3_node");

    // Instantiate action clients and utilities
    custom_nav2_action_client2::NavigateToPoseClient navigate_to_pose_client(node);
    custom_nav2_action_client2::ComputePathToPoseClient path_to_pose_client(node);
    custom_nav2_action_client2::NavigateThroughPosesClient navigate_to_poses_client(node);
    navigate_to_pose_client.initialize();

    custom_nav2_action_client2::Nav2Utilities nav2_utils(node, navigate_to_pose_client,
                                                         path_to_pose_client, navigate_to_poses_client);

    // Create a MultiThreadedExecutor with multiple threads (e.g., 4)
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);

    // Now we can use a timer or future blocking without halting other callbacks
    // We can perform param configuration asynchronously
    // The executor.spin() will run in this main thread and handle all callbacks
    std::thread param_setting_thread([&nav2_utils]()
                                     {
        // Sleep a little to ensure the node and executor are up
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        nav2_utils.SetNav2Parameters();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); });

    executor.spin(); // This will process callbacks in multiple threads

    param_setting_thread.join();
    rclcpp::shutdown();
    return 0;
}
