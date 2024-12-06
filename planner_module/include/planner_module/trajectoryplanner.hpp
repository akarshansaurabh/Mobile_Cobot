#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "maths/commonmathssolver.hpp"
#include "kinematics.hpp"
#include <ruckig/ruckig.hpp>
#include <array>
#include <deque>

using namespace std;
using namespace ruckig;

namespace cMRTrajectoryPlanner
{
    struct JointTrajectoryPoint
    {
        std::array<double, 6> position, velocity, acceleration;
        double time;
    };

    struct Limits
    {
        std::array<double, 6> v_max, a_max;
    };

    enum class TrajectoryValidity
    {
        OUTSIDE_WORKSPACE,
        SINGULARITY_IS_PRESENT,
        TRAJECTORY_IS_VALID,
        SINGULARITY_WILL_BE_PRESENT
    };

    extern Limits joint_max_limits, ee_max_limits;

    class RuckigWrapper
    {
    private:
        Ruckig<6> traj_solver_;
        InputParameter<6> GetRuckigInputs(const std::array<double, 6> &current_positions, const std::array<double, 6> &target_positions, Limits &limits);

    public:
        double sampling_time_;
        // InputParameter<6> input_;
        deque<JointTrajectoryPoint> traj_queue_;
        RuckigWrapper(double sampling_time);
        void GenerateTrajectory(const std::array<double, 6> &current_positions, const std::array<double, 6> &target_positions, Limits &limits, double &t0);
    };
}

#endif