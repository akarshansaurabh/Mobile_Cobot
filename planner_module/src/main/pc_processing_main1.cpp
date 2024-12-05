#include "pc_processing/pointcloud_processor.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_processing::PointCloudProcessor>();
    node->initialize();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
