#ifndef MULTITHREADED_FCL_LOADER_HPP
#define MULTITHREADED_FCL_LOADER_HPP

// Standard C++ & STL
#include <string>
#include <vector>
#include <memory>
#include <future>
#include <iostream>

// Assimp
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// FCL
#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>

// ROS 2 (optional, for logging via rclcpp::Node)
#include <rclcpp/rclcpp.hpp>

namespace multi_fcl_loader
{
    class MultiThreadedFCLLoader
    {
    public:
        explicit MultiThreadedFCLLoader(const std::shared_ptr<rclcpp::Node> &node = nullptr);
        std::vector<std::shared_ptr<fcl::CollisionObjectf>>
        buildFCLCollisionObjectsInParallel(const std::vector<std::string> &link_mesh_paths);

    private:
        bool loadMeshFromSTL(const std::string &path,
                             fcl::BVHModel<fcl::OBBRSS<float>> &model);
        std::shared_ptr<fcl::CollisionObjectf> Create_FCL_CollisionObj(const std::string &mesh_path);

    private:
        std::shared_ptr<rclcpp::Node> node_;
    };
}

#endif
