#include "navigation/nav2_client2.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("nav2_client2_node");
    std::cout << "new code " << std::endl;

    // Instantiate NavigateToPose action client
    custom_nav2_action_client::NavigateToPoseClient navigate_to_pose_client(node);
    custom_nav2_action_client::ComputePathToPoseClient path_to_pose_client(node);
    custom_nav2_action_client::NavigateThroughPosesClient navigate_to_poses_client(node);
    // Instantiate utilities
    custom_nav2_action_client::Nav2Utilities nav2_utils(node, navigate_to_pose_client,
                                                        path_to_pose_client, navigate_to_poses_client);

    std::thread spinner([node]()
                        { rclcpp::spin(node); });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    nav2_utils.SetNav2Parameters();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    spinner.join();
    rclcpp::shutdown();
    return 0;
}
