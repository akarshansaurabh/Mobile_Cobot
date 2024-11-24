#include "pc_processing/pc_processing_1.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pointcloud_transformer::PointCloudTransformer>());
    rclcpp::shutdown();
    return 0;
}
