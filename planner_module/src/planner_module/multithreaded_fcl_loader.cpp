#include "planner_module/multithreaded_fcl_loader.hpp"

namespace multi_fcl_loader
{
    MultiThreadedFCLLoader::MultiThreadedFCLLoader(const std::shared_ptr<rclcpp::Node> &node)
        : node_(node)
    {
        link_offsets_ = {{"/home/akarshan/mobile_cobot_ws/src/r1d1_description/meshes/base_link.stl",
                          {0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f, 0}},
                         {"/home/akarshan/mobile_cobot_ws/src/r1d1_description/meshes/linear_1.stl",
                          {-0.005999, -0.0445, -1.067874,
                           0.0f, 0.0f, 0.0f, 1}},
                         {"/home/akarshan/mobile_cobot_ws/src/r1d1_description/meshes/arm_base_link_1.stl",
                          {-0.018999, 0.0005, -1.09902,
                           0.0f, 0.0f, 0.0f, 2}},
                         {"/home/akarshan/mobile_cobot_ws/src/r1d1_description/meshes/shoulder_Link_1.stl",
                          {-0.094999, 0.0005, -1.09902,
                           0.0f, 0.0f, 0.0f, 3}},
                         {"/home/akarshan/mobile_cobot_ws/src/r1d1_description/meshes/upperarm_Link_1.stl",
                          {-0.158999, -0.009342, -1.049998,
                           0.0f, 0.0f, 0.0f, 4}},
                         {"/home/akarshan/mobile_cobot_ws/src/r1d1_description/meshes/forearm_Link_1.stl",
                          {-0.37078, -0.188921, -1.086052,
                           0.0f, 0.0f, 0.0f, 5}},
                         {"/home/akarshan/mobile_cobot_ws/src/r1d1_description/meshes/wrist1_Link_1.stl",
                          {-0.552363, -0.344177, -1.110082,
                           0.0f, 0.0f, 0.0f, 6}},
                         {"/home/akarshan/mobile_cobot_ws/src/r1d1_description/meshes/wrist2_Link_1.stl",
                          {-0.504715, -0.383705, -1.07212,
                           0.0f, 0.0f, 0.0f, 7}},
                         {"/home/akarshan/mobile_cobot_ws/src/r1d1_description/meshes/wrist3_Link_1.stl",
                          {-0.494322, -0.443048, -1.112671,
                           0.0f, 0.0f, 0.0f, 8}}};
    }

    std::vector<std::shared_ptr<fcl::CollisionObjectf>>
    MultiThreadedFCLLoader::buildFCLCollisionObjectsInParallel()
    {
        // Vector to store futures, each returns a pair of mesh path and collision object
        std::vector<std::future<std::pair<std::string, std::shared_ptr<fcl::CollisionObjectf>>>> futures;
        futures.reserve(link_offsets_.size());

        for (const auto &pair : link_offsets_)
        {
            const std::string &path = pair.first;
            futures.emplace_back(std::async(std::launch::async, [this, path]() -> std::pair<std::string, std::shared_ptr<fcl::CollisionObjectf>>
                                            {
                auto obj = this->Create_FCL_CollisionObj(path);
                return {path, obj}; }));
        }
        std::vector<std::pair<int, std::shared_ptr<fcl::CollisionObjectf>>> temp_objects;
        temp_objects.reserve(link_offsets_.size());

        for (auto &fut : futures)
        {
            auto [path, obj] = fut.get();
            if (!obj)
            {
                RCLCPP_ERROR(node_->get_logger(), "STL load failed for path: %s", path.c_str());
            }
            else
            {
                auto it = link_offsets_.find(path);
                if (it != link_offsets_.end())
                {
                    temp_objects.emplace_back(it->second.link_index, obj);
                }
            }
        }

        // Sort the temporary vector based on link_index in ascending order
        std::sort(temp_objects.begin(), temp_objects.end(),
                  [](const std::pair<int, std::shared_ptr<fcl::CollisionObjectf>> &a,
                     const std::pair<int, std::shared_ptr<fcl::CollisionObjectf>> &b) -> bool
                  {
                      return a.first < b.first;
                  });
        std::vector<std::shared_ptr<fcl::CollisionObjectf>> collision_objects;
        collision_objects.reserve(temp_objects.size());
        for (const auto &pair : temp_objects)
        {
            collision_objects.push_back(pair.second);
        }

        RCLCPP_INFO(node_->get_logger(), "Loaded %zu CollisionObjects in parallel and sorted by link_index.", collision_objects.size());
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

        // 2) Get local offsets
        double x_off, y_off, z_off;
        float roll_off, pitch_off, yaw_off;
        auto it = link_offsets_.find(path);
        if (it != link_offsets_.end())
        {
            x_off = it->second.x_off;
            y_off = it->second.y_off;
            z_off = it->second.z_off;
            roll_off = it->second.roll_off;
            pitch_off = it->second.pitch_off;
            yaw_off = it->second.yaw_off;
        }

        // 3) Build the BVH
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
                if (f % 2 == 0)
                {
                    if (path == "/home/akarshan/mobile_cobot_ws/src/r1d1_description/meshes/linear_1.stl")
                    {
                        std::cout << "offset " << x_off << " " << y_off << " " << z_off << std::endl;
                    }
                    fcl::Vector3f v0((mesh->mVertices[face.mIndices[0]].x * 0.001) + x_off,
                                     (mesh->mVertices[face.mIndices[0]].y * 0.001) + y_off,
                                     (mesh->mVertices[face.mIndices[0]].z * 0.001) + z_off);

                    fcl::Vector3f v1((mesh->mVertices[face.mIndices[1]].x * 0.001) + x_off,
                                     (mesh->mVertices[face.mIndices[1]].y * 0.001) + y_off,
                                     (mesh->mVertices[face.mIndices[1]].z * 0.001) + z_off);

                    fcl::Vector3f v2((mesh->mVertices[face.mIndices[2]].x * 0.001) + x_off,
                                     (mesh->mVertices[face.mIndices[2]].y * 0.001) + y_off,
                                     (mesh->mVertices[face.mIndices[2]].z * 0.001) + z_off);
                    model.addTriangle(v0, v1, v2);
                }
            }
        }

        // 4) Finalize the BVH
        model.endModel();

        RCLCPP_INFO(node_->get_logger(), "Successfully loaded STL: %s", path.c_str());
        return true;
    }

}
