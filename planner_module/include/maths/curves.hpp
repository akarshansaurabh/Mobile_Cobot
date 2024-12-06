#ifndef CURVES_HPP
#define CURVES_HPP

#include <iostream>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "commonmathssolver.hpp"

using namespace std;

namespace Curves
{
    enum class CurveType
    {
        Line,
        Arc
    };

    class Curves
    {
    public:
        double path_lenght;
        virtual Eigen::Vector4d GenerateSinglePoint(double param_t) = 0;
        virtual vector<Eigen::Vector4d> Generate3DPoints(int &num) = 0;
    };

    class Line : public Curves
    {
    protected:
        Eigen::Vector4d P1, P2;

    public:
        Line(const Eigen::Vector4d &P1, const Eigen::Vector4d &P2);
        Eigen::Vector4d GenerateSinglePoint(double param_t) override;
        vector<Eigen::Vector4d> Generate3DPoints(int &num) override;
        vector<Eigen::Vector4d> Generate3DPoints(double &precision);
        vector<pair<Eigen::Vector4d, Eigen::Quaterniond>> Generate6DPoints(const Eigen::Matrix4d &pose1, const Eigen::Matrix4d &pose2, double &precision);
    };
}

#endif