#include <iostream>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "maths/commonmathssolver.hpp"
#include "maths/curves.hpp"

using namespace std;

namespace Curves
{
    Line::Line(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2)
    {
        this->P1 = P1;
        this->P2 = P2;
        path_lenght = CommonMathsSolver::Geometry::Distance2P(P1, P2);
    }

    Eigen::Vector4d Line::GenerateSinglePoint(double param_t)
    {
        double i = param_t / path_lenght;
        if (path_lenght == 0)
            i = 0.0;
        return (P1 + (i * (P2 - P1)));
    }

    vector<Eigen::Vector4d> Line::Generate3DPoints(int &num)
    {
        double precision = path_lenght / num;

        vector<Eigen::Vector4d> line_points;
        for (int i = 0; i <= num; i++)
            line_points.push_back(GenerateSinglePoint(i * precision));
        return line_points;
    }

    vector<Eigen::Vector4d> Line::Generate3DPoints(double &precision)
    {
        int num = path_lenght / precision;
        if (num > 0)
            precision = path_lenght / num;
        else if (path_lenght > 0)
            precision = path_lenght;
        num = path_lenght / precision;
        if (path_lenght <= 0.01)
        {
            precision = 0.0;
            num = 0;
        }

        vector<Eigen::Vector4d> line_points;
        for (int i = 0; i <= num; i++)
            line_points.push_back(GenerateSinglePoint(i * precision));
        return line_points;
    }

    vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> Line::Generate6DPoints(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, double &precision)
    {
        vector<Eigen::Vector4d> line_xyzs = Generate3DPoints(precision);
        Eigen::Quaterniond Q1(pose1.block<3, 3>(0, 0)), Q2(pose2.block<3, 3>(0, 0));
        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> ans;
        vector<Eigen::Quaterniond> line_qs;

        if (line_xyzs.size() > 1)
        {
            line_qs = CommonMathsSolver::OrientationNTransformaton::GenerateQuaternions(Q1, Q2, line_xyzs.size());
            for (int i = 0; i < line_xyzs.size(); i++)
                ans.push_back(make_pair(line_xyzs[i], line_qs[i]));
        }
        else if (line_xyzs.size() == 1 && (fabs(Q2.x() - Q1.x()) > 0.01 || fabs(Q2.y() - Q1.y()) > 0.01 || fabs(Q2.z() - Q1.z()) > 0.01 || fabs(Q2.w() - Q1.w()) > 0.01))
        {
            line_qs = CommonMathsSolver::OrientationNTransformaton::GenerateQuaternions(Q1, Q2, 100);
            for (int i = 0; i < 100; i++)
                ans.push_back(make_pair(pose1.block<4, 1>(0, 3), line_qs[i]));
        }
        return ans;
    }

}
