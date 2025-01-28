#ifndef COMMONMATHSSOLVER_HPP
#define COMMONMATHSSOLVER_HPP

#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace std;

const double epsilon = 0.01;

namespace CommonMathsSolver
{
    namespace Vectors3D
    {
        Eigen::Vector3d FindUnitNormal(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3);
    }
    namespace OrientationNTransformaton
    {
        Eigen::Matrix3d Compute_Rx(double t);
        Eigen::Matrix3d Compute_Ry(double t);
        Eigen::Matrix3d Compute_Rz(double t);
        Eigen::Matrix3d ComputeR(const Eigen::Vector3d &rpy);
        vector<Eigen::Quaterniond> GenerateQuaternions(Eigen::Quaterniond &qi, Eigen::Quaterniond &qf, int num);
    }
    namespace Geometry
    {
        double Distance2P(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2);
    }
    namespace MaxMin
    {
        double FindMax(const vector<double> &vec);
        double FindMin(const vector<double> &vec);
    }
}

#endif

// header = 1170 + 
