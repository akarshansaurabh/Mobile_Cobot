#include "planner_module/arm_contoller.hpp"
#include "planner_module/multithreaded_fcl_loader.hpp"

using namespace arm_planner;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("custom_arm_controller_node");
    std::string urdf = "robot_description";
    std::string root_link = "base_link";
    std::string tip_link = "ee_link";
    auto kinematics_solver = std::make_shared<cMRKinematics::ArmKinematicsSolver<7>>(node, urdf, root_link, tip_link);
    auto fcl_ = std::make_shared<multi_fcl_loader::MultiThreadedFCLLoader>(node);
    auto collision_objs = fcl_->buildFCLCollisionObjectsInParallel();
    arm_planner::ArmController action_client_obj(node, kinematics_solver, collision_objs);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
