#include "maths/commonmathssolver.hpp"

namespace CommonMathsSolver
{
    Eigen::Matrix3d OrientationNTransformaton::Compute_Rx(double t)
    {
        Eigen::Matrix3d rx;
        rx << 1, 0, 0,
            0, cos(t), -sin(t),
            0, sin(t), cos(t);
        return rx;
    }

    Eigen::Matrix3d OrientationNTransformaton::Compute_Ry(double t)
    {
        Eigen::Matrix3d ry;
        ry << cos(t), 0, sin(t),
            0, 1, 0,
            -sin(t), 0, cos(t);
        return ry;
    }

    Eigen::Matrix3d OrientationNTransformaton::Compute_Rz(double t)
    {
        Eigen::Matrix3d rz;
        rz << cos(t), -sin(t), 0,
            sin(t), cos(t), 0,
            0, 0, 1;
        return rz;
    }

    Eigen::Matrix3d OrientationNTransformaton::ComputeR(const Eigen::Vector3d &rpy)
    {
        double a = rpy(0), b = rpy(1), y = rpy(2);
        Eigen::Matrix3d target_rot;
        target_rot << (cos(b) * cos(y)), ((-cos(a) * sin(y)) + (sin(a) * sin(b) * cos(y))), ((sin(a) * sin(y)) + (cos(a) * sin(b) * cos(y))),
            (cos(b) * sin(y)), ((cos(a) * cos(y)) + (sin(a) * sin(b) * sin(y))), ((-sin(a) * cos(y)) + (cos(a) * sin(b) * sin(y))),
            (-sin(b)), ((cos(b) * sin(a))), ((cos(a) * cos(b)));
        return target_rot;
    }

    vector<Eigen::Quaterniond> OrientationNTransformaton::GenerateQuaternions(Eigen::Quaterniond &qi, Eigen::Quaterniond &qf, int num)
    {
        vector<Eigen::Quaterniond> quaternions;
        qi.normalize();
        qf.normalize();
        double angle = 2 * acos(qi.dot(qf));
        double dt = angle / (num - 1);

        for (int i = 0; i < num; i++)
            if (fabs(qf.x() - qi.x()) <= 0.00001 && fabs(qf.y() - qi.y()) <= 0.00001 && fabs(qf.z() - qi.z()) <= 0.00001 && fabs(qf.w() - qi.w()) <= 0.00001)
                quaternions.push_back(qf);
            else
                quaternions.push_back(qi.slerp((i * dt / angle), qf));
        return quaternions;
    }

    double Geometry::Distance2P(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2)
    {
        return sqrt(pow(P2(0) - P1(0), 2) + pow(P2(1) - P1(1), 2) + pow(P2(2) - P1(2), 2));
    }

    double MaxMin::FindMax(const vector<double> &vec)
    {
        double max1 = vec[0];
        for (int i = 1; i < vec.size(); i++)
            max1 = max(max1, vec[i]);
        return max1;
    }

    double MaxMin::FindMin(const vector<double> &vec)
    {
        double min1 = vec[0];
        for (int i = 1; i < vec.size(); i++)
            min1 = min(min1, vec[i]);
        return min1;
    }

    Eigen::Vector3d Vectors3D::FindUnitNormal(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3)
    {
        Eigen::Vector3d p2p1 = p1 - p2;
        Eigen::Vector3d p2p3 = p3 - p2;
        Eigen::Vector3d cp = p2p1.cross(p2p3);
        return (cp / cp.norm());
    }
}
