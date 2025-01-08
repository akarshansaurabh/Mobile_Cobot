#include "planner_module/multithreaded_fcl_loader.hpp"

namespace multi_fcl_loader
{
    MultiThreadedFCLLoader::MultiThreadedFCLLoader(const std::shared_ptr<rclcpp::Node> &node)
        : node_(node)
    {
        // Nothing special here; just store the node pointer
    }

    std::vector<std::shared_ptr<fcl::CollisionObjectf>>
    MultiThreadedFCLLoader::buildFCLCollisionObjectsInParallel(const std::vector<std::string> &link_mesh_paths)
    {
        // We'll store futures, each representing a single STL load
        std::vector<std::future<std::shared_ptr<fcl::CollisionObjectf>>> futures;
        futures.reserve(link_mesh_paths.size());

        // For each path, launch a thread with std::async
        for (const auto &path : link_mesh_paths)
            futures.push_back(std::async(std::launch::async, [this, path]()
                                         { return this->Create_FCL_CollisionObj(path); }));

        // Wait for all futures and gather results
        std::vector<std::shared_ptr<fcl::CollisionObjectf>> collision_objects;
        collision_objects.reserve(link_mesh_paths.size());

        for (size_t i = 0; i < futures.size(); i++)
        {
            std::shared_ptr<fcl::CollisionObjectf> obj = futures[i].get();
            if (!obj)
                RCLCPP_ERROR(node_->get_logger(), "STL load failed for path: %s", link_mesh_paths[i].c_str());
            else
                collision_objects.push_back(obj);
        }

        RCLCPP_INFO(node_->get_logger(), "Loaded %zu CollisionObjects in parallel.", collision_objects.size());
        return collision_objects;
    }

    std::shared_ptr<fcl::CollisionObjectf>
    MultiThreadedFCLLoader::Create_FCL_CollisionObj(const std::string &mesh_path)
    {
        // 1) Create a BVHModel for OBBRSS
        typedef fcl::BVHModel<fcl::OBBRSS<float>> MyBVHModel;
        auto mesh_model = std::make_shared<MyBVHModel>();

        // 2) Attempt to load the STL into mesh_model
        bool ok = loadMeshFromSTL(mesh_path, *mesh_model);
        if (!ok)
        {
            // Return null on failure
            return nullptr;
        }

        // 3) Create a CollisionObjectf
        std::shared_ptr<fcl::CollisionGeometryf> link_geometry = mesh_model;
        auto link_obj = std::make_shared<fcl::CollisionObjectf>(link_geometry);

        // By default, set identity transform
        link_obj->setTranslation(fcl::Vector3f(0.f, 0.f, 0.f));
        link_obj->setRotation(fcl::Matrix3f::Identity());

        return link_obj;
    }

    bool MultiThreadedFCLLoader::loadMeshFromSTL(const std::string &path,
                                                 fcl::BVHModel<fcl::OBBRSS<float>> &model)
    {
        // 1) Use Assimp to load the file
        Assimp::Importer importer;
        const aiScene *scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

        if (!scene || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) || !scene->mRootNode)
        {
            RCLCPP_ERROR(node_->get_logger(), "Assimp error loading %s: %s",
                         path.c_str(), importer.GetErrorString());
            return false;
        }

        // 2) Build the BVH
        model.beginModel();

        // For each mesh in the scene
        for (unsigned int i = 0; i < scene->mNumMeshes; i++)
        {
            const aiMesh *mesh = scene->mMeshes[i];
            if (!mesh)
                continue;

            // For each face
            for (unsigned int f = 0; f < mesh->mNumFaces; f++)
            {
                const aiFace &face = mesh->mFaces[f];
                if (face.mNumIndices != 3)
                {
                    RCLCPP_WARN(node_->get_logger(), "Non-triangular face in %s. Skipping.", path.c_str());
                    continue;
                }
                fcl::Vector3f v0(mesh->mVertices[face.mIndices[0]].x, mesh->mVertices[face.mIndices[0]].y, mesh->mVertices[face.mIndices[0]].z);
                fcl::Vector3f v1(mesh->mVertices[face.mIndices[1]].x, mesh->mVertices[face.mIndices[1]].y, mesh->mVertices[face.mIndices[1]].z);
                fcl::Vector3f v2(mesh->mVertices[face.mIndices[2]].x, mesh->mVertices[face.mIndices[2]].y, mesh->mVertices[face.mIndices[2]].z);
                model.addTriangle(v0, v1, v2);
            }
        }

        // 3) Finalize the BVH
        model.endModel();

        RCLCPP_INFO(node_->get_logger(), "Successfully loaded STL: %s", path.c_str());
        return true;
    }

}
