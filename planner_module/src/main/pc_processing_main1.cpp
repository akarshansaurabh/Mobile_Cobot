#include "pc_processing/pointcloud_processor.hpp"
#include "pc_processing/threeD_perception.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pointcloud_processor_node");
    pointcloud_processing::PointCloudProcessor pc_processor(node);
    environment3DPerception::Environment3DPerception three_d_perception(node);

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
