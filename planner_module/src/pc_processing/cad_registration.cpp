#include "pc_processing/cad_registration.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cmath>
#include <thread>
#include <future>
#include <vector>

// Constructor / Destructor
CADRegistrationPipeline::CADRegistrationPipeline()
    : m_partialCloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
      m_cadCloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
      m_finalPose(Eigen::Matrix4f::Identity()),
      m_finalCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
{
}

CADRegistrationPipeline::~CADRegistrationPipeline()
{
}

void CADRegistrationPipeline::setPartialCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &partial)
{
    m_partialCloud = partial;
}

void CADRegistrationPipeline::setCadCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cad)
{
    m_cadCloud = cad;
}

// Fixed: use sensor_msgs::msg::PointCloud2
void CADRegistrationPipeline::loadPartialFromROS(const sensor_msgs::msg::PointCloud2 &rosCloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(rosCloud, *temp);
    m_partialCloud = temp;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CADRegistrationPipeline::loadCADAsXYZRGB(const std::string &cad_file_path)
{
    // Create a PolygonMesh to hold the loaded mesh
    pcl::PolygonMesh mesh;

    // Attempt to load the mesh from the STL file
    // (For OBJ, use: pcl::io::loadPolygonFileOBJ)
    if (pcl::io::loadPolygonFileSTL(cad_file_path, mesh) == 0)
    {
        std::cerr << "[loadChairCADAsXYZRGB] ERROR: Could not load STL file: "
                  << cad_file_path << std::endl;
        // Return an empty cloud on failure
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    }

    // Convert the mesh's cloud (a PCLPointCloud2) to a temporary PointXYZ cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempXYZ(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh.cloud, *tempXYZ);

    // Create a new XYZRGB cloud to hold the final output
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    outputCloud->reserve(tempXYZ->size());

    // Assign a default color to each point (e.g., gray)
    const uint8_t defaultR = 128;
    const uint8_t defaultG = 128;
    const uint8_t defaultB = 128;

    // Copy XYZ data and assign color
    for (const auto &pt : tempXYZ->points)
    {
        pcl::PointXYZRGB rgbPt;
        rgbPt.x = pt.x;
        rgbPt.y = pt.y;
        rgbPt.z = pt.z;
        rgbPt.r = defaultR;
        rgbPt.g = defaultG;
        rgbPt.b = defaultB;
        outputCloud->push_back(rgbPt);
    }

    std::cout << "[loadChairCADAsXYZRGB] Loaded " << outputCloud->size()
              << " vertices from file: " << cad_file_path << std::endl;

    return outputCloud;
}

void CADRegistrationPipeline::computeRegistration()
{
    // Step 1) Voxel Downsample partial + CAD in parallel using a vector of futures
    std::vector<std::future<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> voxelFutures;
    voxelFutures.push_back(std::async(std::launch::async, [this]()
                                      { return voxelDownsample(m_partialCloud, 0.015f /*leaf size in meters*/); }));
    voxelFutures.push_back(std::async(std::launch::async, [this]()
                                      { return voxelDownsample(m_cadCloud, 0.015f); }));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr partialDown = voxelFutures[0].get();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cadDown = voxelFutures[1].get();

    // Step 2) Normal + Feature extraction in parallel
    std::vector<std::future<pcl::PointCloud<pcl::Normal>::Ptr>> normalFutures;
    normalFutures.push_back(std::async(std::launch::async, [this, partialDown]()
                                       { return estimateNormalsKNN(partialDown, 20); }));
    normalFutures.push_back(std::async(std::launch::async, [this, cadDown]()
                                       { return estimateNormalsKNN(cadDown, 20); }));

    pcl::PointCloud<pcl::Normal>::Ptr partialNormals = normalFutures[0].get();
    pcl::PointCloud<pcl::Normal>::Ptr cadNormals = normalFutures[1].get();

    // pipeline 1
    // // Now for FPFH feature extraction, again in parallel
    // std::vector<std::future<pcl::PointCloud<pcl::FPFHSignature33>::Ptr>> fpfhFutures;
    // fpfhFutures.push_back(std::async(std::launch::async, [this, partialDown, partialNormals]()
    // {
    //     return computeFPFH(partialDown, partialNormals, 0.025f /*feature radius*/);
    // }));
    // fpfhFutures.push_back(std::async(std::launch::async, [this, cadDown, cadNormals]()
    // {
    //     return computeFPFH(cadDown, cadNormals, 0.025f);
    // }));
    // pcl::PointCloud<pcl::FPFHSignature33>::Ptr partialFeatures = fpfhFutures[0].get();
    // pcl::PointCloud<pcl::FPFHSignature33>::Ptr cadFeatures     = fpfhFutures[1].get();
    // // Step 3) Global alignment (SampleConsensusPrerejective)
    // Eigen::Matrix4f coarseTransform = globalAlignment(
    //     partialDown, partialNormals, partialFeatures,
    //     cadDown,     cadNormals,     cadFeatures);

    // pipeline 2
    Eigen::Matrix4f coarseTransform = bestGlobalAlignmentPPF(partialDown, cadDown, partialNormals, cadNormals);

    // Fixed: use partialDown instead of IpartialDown
    Eigen::Matrix4f refinedTransform = refineAlignmentGICP(partialDown, cadDown, coarseTransform);

    m_finalPose = refinedTransform;

    // Apply final pose to the *full-resolution* CAD
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedCad(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*m_cadCloud, *alignedCad, m_finalPose);

    // Step 5) Merge partial + aligned CAD (full resolution partial)
    m_finalCloud = mergeClouds(m_partialCloud, alignedCad);

    // Step 6) Convert final to sensor_msgs::msg::PointCloud2 for RViz
    pcl::toROSMsg(*m_finalCloud, m_finalRosCloud);
    m_finalRosCloud.header.frame_id = "map"; // or any frame you prefer
}

Eigen::Matrix4f CADRegistrationPipeline::getFinalPose() const
{
    return m_finalPose;
}

sensor_msgs::msg::PointCloud2 CADRegistrationPipeline::getFinalRosCloud() const
{
    return m_finalRosCloud;
}

//----------------------------------------------------------
// Private utility methods
//----------------------------------------------------------

// 1) Voxel downsample
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CADRegistrationPipeline::voxelDownsample(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
    float leafSize) const
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*output);
    return output;
}

// 2) Normal Estimation (KNN-based)
pcl::PointCloud<pcl::Normal>::Ptr CADRegistrationPipeline::estimateNormalsKNN(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
    int k) const
{
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(input);
    ne.setNumberOfThreads(4);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    ne.setKSearch(k);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.compute(*normals);

    return normals;
}

// 3) FPFH feature extraction
pcl::PointCloud<pcl::FPFHSignature33>::Ptr CADRegistrationPipeline::computeFPFH(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    float radius) const
{
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setNumberOfThreads(4);
    fpfh.setInputCloud(input);
    fpfh.setInputNormals(normals);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(radius);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh.compute(*features);
    return features;
}

// 4) Global alignment with SampleConsensusPrerejective
Eigen::Matrix4f CADRegistrationPipeline::globalAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &partial,
                                                         const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &partialFeatures,
                                                         const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cad,
                                                         const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &cadFeatures) const
{
    pcl::SampleConsensusPrerejective<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac;
    sac.setInputSource(cad);
    sac.setSourceFeatures(cadFeatures);
    sac.setInputTarget(partial);
    sac.setTargetFeatures(partialFeatures);

    // Some typical hyperparameters:
    sac.setMaximumIterations(50000);         // RANSAC iterations
    sac.setNumberOfSamples(3);               // Points to sample
    sac.setCorrespondenceRandomness(5);      // Nearest features
    sac.setSimilarityThreshold(0.9f);        // Edge length similarity
    sac.setMaxCorrespondenceDistance(0.05f); // Meters
    sac.setInlierFraction(0.25f);

    pcl::PointCloud<pcl::PointXYZRGB> sacResult;
    sac.align(sacResult);

    if (!sac.hasConverged())
    {
        std::cerr << "[globalAlignment] SampleConsensusPrerejective FAILED to converge!\n";
        return Eigen::Matrix4f::Identity();
    }
    else
    {
        std::cout << "[globalAlignment] Succeeded. Score: "
                  << sac.getFitnessScore() << std::endl;
        return sac.getFinalTransformation();
    }
}

// 4B) PPF-based alignment with concurrency for normal estimation
Eigen::Matrix4f CADRegistrationPipeline::bestGlobalAlignmentPPF(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &partial,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cad,
    const pcl::PointCloud<pcl::Normal>::Ptr &partialNormals,
    const pcl::PointCloud<pcl::Normal>::Ptr &cadNormals)
{
    // Combine clouds with normals into PointXYZRGBNormal
    auto partialWithNormals = combineCloudAndNormals(partial, partialNormals);
    auto cadWithNormals = combineCloudAndNormals(cad, cadNormals);
    if (!partialWithNormals || !cadWithNormals)
    {
        std::cerr << "[bestGlobalAlignmentPPF] Failed to combine clouds with normals.\n";
        return Eigen::Matrix4f::Identity();
    }

    // 2) Compute PPF features on the CAD
    pcl::PointCloud<pcl::PPFSignature>::Ptr cadPPF(new pcl::PointCloud<pcl::PPFSignature>());
    {
        pcl::PPFEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PPFSignature> ppfEst;
        ppfEst.setInputCloud(cadWithNormals);
        ppfEst.setInputNormals(cadWithNormals); // PointXYZRGBNormal includes normals
        ppfEst.compute(*cadPPF);
    }
    std::cout << "[PPF] CAD PPF features computed. Size: " << cadPPF->size() << std::endl;

    // 3) Build a PPF HashMap for the CAD
    pcl::PPFHashMapSearch::Ptr hashMapSearch(new pcl::PPFHashMapSearch(10.0f / 180.0f * static_cast<float>(M_PI), 0.03f));
    hashMapSearch->setInputFeatureCloud(cadPPF);
    std::cout << "[PPF] HashMap prepared.\n";

    // 4) Register partial (scene) to CAD (model)
    pcl::PPFRegistration<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ppfReg;
    ppfReg.setInputTarget(partialWithNormals);
    ppfReg.setInputSource(cadWithNormals);
    ppfReg.setSearchMethod(hashMapSearch);

    pcl::PointCloud<pcl::PointXYZRGBNormal> alignedCAD;
    ppfReg.align(alignedCAD);

    Eigen::Matrix4f transform = ppfReg.getFinalTransformation();
    std::cout << "[PPF] Registration completed. Score: "
              << ppfReg.getFitnessScore() << std::endl;
    std::cout << "[PPF] Final transform:\n"
              << transform << std::endl;

    return transform;
}

// 5) Fine alignment with Generalized ICP
Eigen::Matrix4f CADRegistrationPipeline::refineAlignmentGICP(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &partial,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cad,
    const Eigen::Matrix4f &initialGuess) const
{
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
    gicp.setMaximumIterations(100);
    gicp.setMaxCorrespondenceDistance(0.02); // meters
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setInputTarget(partial);
    gicp.setInputSource(cad);

    pcl::PointCloud<pcl::PointXYZRGB> finalOutput;
    gicp.align(finalOutput, initialGuess);

    if (!gicp.hasConverged())
    {
        std::cerr << "[refineAlignmentGICP] GICP failed to converge. Returning initial guess.\n";
        return initialGuess;
    }
    else
    {
        std::cout << "[refineAlignmentGICP] GICP converged. Score: "
                  << gicp.getFitnessScore() << std::endl;
        return gicp.getFinalTransformation();
    }
}

// 6) Merge partial + aligned CAD
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CADRegistrationPipeline::mergeClouds(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &partial,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &alignedCad) const
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGB>());
    merged->reserve(partial->size() + alignedCad->size());
    *merged += *partial;
    *merged += *alignedCad;
    return merged;
}

// Utility to combine PointXYZRGB and Normal into PointXYZRGBNormal
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CADRegistrationPipeline::combineCloudAndNormals(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals) const
{
    if (cloud->size() != normals->size())
    {
        std::cerr << "[combineCloudAndNormals] ERROR: Cloud and normals have different sizes!\n";
        return nullptr;
    }
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr combined(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    combined->resize(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        combined->points[i].x = cloud->points[i].x;
        combined->points[i].y = cloud->points[i].y;
        combined->points[i].z = cloud->points[i].z;
        combined->points[i].rgb = cloud->points[i].rgb;
        combined->points[i].normal_x = normals->points[i].normal_x;
        combined->points[i].normal_y = normals->points[i].normal_y;
        combined->points[i].normal_z = normals->points[i].normal_z;
        combined->points[i].curvature = normals->points[i].curvature;
    }
    return combined;
}