#ifndef MULTITHREADED_FCL_LOADER_HPP
#define MULTITHREADED_FCL_LOADER_HPP

// Standard C++ & STL
#include <string>
#include <vector>
#include <memory>
#include <future>
#include <iostream>
#include <unordered_map>

// Assimp
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// FCL
#include <fcl/fcl.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>

#include <rclcpp/rclcpp.hpp>
#include "visualizations/visualization_manager.hpp"

namespace multi_fcl_loader
{
    struct LinkOffset
    {
        double x_off, y_off, z_off;
        float roll_off, pitch_off, yaw_off;
        int link_index;
    };

    class MultiThreadedFCLLoader
    {
    public:
        explicit MultiThreadedFCLLoader(const std::shared_ptr<rclcpp::Node> &node = nullptr);
        std::vector<std::shared_ptr<fcl::CollisionObjectf>>
        buildFCLCollisionObjectsInParallel();

    private:
        bool loadMeshFromSTL(const std::string &path,
                             fcl::BVHModel<fcl::OBBRSS<float>> &model);
        std::shared_ptr<fcl::CollisionObjectf> Create_FCL_CollisionObj(const std::string &mesh_path);

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::unordered_map<std::string, LinkOffset> link_offsets_;
        std::shared_ptr<visualization::VisualizationManager> viz_manager_;
    };
}

#endif
