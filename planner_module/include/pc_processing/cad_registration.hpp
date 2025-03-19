#ifndef CAD_REGISTRATION_PIPELINE_H
#define CAD_REGISTRATION_PIPELINE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ppf_registration.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/ppf.h>

#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/PolygonMesh.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <future>
#include <thread>
#include <vector>
#include <string>
#include <iostream>

class CADRegistrationPipeline
{
public:
    CADRegistrationPipeline();
    ~CADRegistrationPipeline();

    // Sets the partial scene cloud (with multiple objects, including the chair).
    void setPartialCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &partial);

    // Sets the CAD model cloud
    void setCadCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cad);

    // Convert from sensor_msgs::msg::PointCloud2 to PCL
    void loadPartialFromROS(const sensor_msgs::msg::PointCloud2 &rosCloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadCADAsXYZRGB(const std::string &cad_file_path);

    // Main pipeline:
    //  1) Voxel downsample (parallel via a vector of futures)
    //  2) Normal + Feature extraction (also in parallel)
    //  3) Global alignment (SampleConsensusPrerejective)
    //  4) GICP refinement
    //  5) Merge final
    //  6) Convert to ROS
    void computeRegistration();

    // Retrieve final pose
    Eigen::Matrix4f getFinalPose() const;

    // Retrieve final merged cloud as a sensor_msgs for RViz
    sensor_msgs::msg::PointCloud2 getFinalRosCloud() const;

private:
    // Utility methods

    // 1) Voxel Downsample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelDownsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                                                           float leafSize) const;

    // 2) Normal Estimation with k-nearest neighbors
    pcl::PointCloud<pcl::Normal>::Ptr estimateNormalsKNN(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                                                         int k) const;

    // 3) FPFH feature extraction
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                                                           const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                                           float radius) const;

    // 4) Global alignment with SampleConsensusPrerejective
    Eigen::Matrix4f globalAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &partial,
                                    const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &partialFeatures,
                                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cad,
                                    const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &cadFeatures) const;

    // 4B) Alternative global alignment with PPF (multi-threaded normal estimation)
    Eigen::Matrix4f bestGlobalAlignmentPPF(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &partial,
                                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cad,
                                           const pcl::PointCloud<pcl::Normal>::Ptr &partialNormals,
                                           const pcl::PointCloud<pcl::Normal>::Ptr &cadNormals);

    // 5) Fine alignment with GICP
    Eigen::Matrix4f refineAlignmentGICP(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &partial,
                                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cad,
                                        const Eigen::Matrix4f &initialGuess) const;

    // 6) Merge partial + aligned CAD
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergeClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &partial,
                                                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &alignedCad) const;

    // Utility to combine PointXYZRGB and Normal into PointXYZRGBNormal
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr combineCloudAndNormals(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals) const;

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_partialCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cadCloud;

    Eigen::Matrix4f m_finalPose;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_finalCloud;
    sensor_msgs::msg::PointCloud2 m_finalRosCloud;
};

#endif // CAD_REGISTRATION_PIPELINE_H