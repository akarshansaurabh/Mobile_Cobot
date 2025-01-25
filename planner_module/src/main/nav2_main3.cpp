#include "navigation/nav2_client3.hpp"
#include "planner_module/manager.hpp"

// #include "planner_module/arm_contoller.hpp"
// #include "planner_module/multithreaded_fcl_loader.hpp"

// using namespace arm_planner;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("nav2_client3_node");
    auto pc_detection_tracker_ = std::make_shared<planner_correction::DetectionTracker>();

    // std::string urdf = "robot_description";
    // std::string root_link = "base_link";
    // std::string tip_link = "ee_link";
    // auto kinematics_solver = std::make_shared<cMRKinematics::ArmKinematicsSolver<7>>(node, urdf, root_link, tip_link);
    // auto fcl_ = std::make_shared<multi_fcl_loader::MultiThreadedFCLLoader>(node);
    // auto collision_objs = fcl_->buildFCLCollisionObjectsInParallel();
    // arm_planner::ArmController action_client_obj(node, kinematics_solver, collision_objs);

    custom_nav2_action_client2::NavigateToPoseClient navigate_to_pose_client(node, pc_detection_tracker_);
    custom_nav2_action_client2::ComputePathToPoseClient path_to_pose_client(node);
    custom_nav2_action_client2::NavigateThroughPosesClient navigate_to_poses_client(node);
    custom_nav2_action_client2::Nav2Utilities nav2_utils(node, navigate_to_pose_client,
                                                         path_to_pose_client, navigate_to_poses_client);
    
    manager::Manager manager_(node, navigate_to_pose_client, navigate_to_poses_client, nav2_utils);
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
