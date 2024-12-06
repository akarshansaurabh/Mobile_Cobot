#include "planner_module/trajectoryplanner.hpp"

namespace cMRTrajectoryPlanner
{
    Limits joint_max_limits, ee_max_limits;

    RuckigWrapper::RuckigWrapper(double sampling_time) : traj_solver_(sampling_time), sampling_time_(sampling_time) {}

    InputParameter<6> RuckigWrapper::GetRuckigInputs(const std::array<double, 6> &current_positions, const std::array<double, 6> &target_positions, Limits &limits)
    {
        InputParameter<6> input;
        input.current_position = {current_positions[0], current_positions[1], current_positions[2],
                                  current_positions[3], current_positions[4], current_positions[5]};
        input.current_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        input.target_position = {target_positions[0], target_positions[1], target_positions[2],
                                 target_positions[3], target_positions[4], target_positions[5]};
        input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        input.max_velocity = {limits.v_max[0], limits.v_max[1], limits.v_max[2],
                              limits.v_max[3], limits.v_max[4], limits.v_max[5]};
        input.max_acceleration = {limits.a_max[0], limits.a_max[1], limits.a_max[2],
                                  limits.a_max[3], limits.a_max[4], limits.a_max[5]};
        input.max_jerk = {1.0, 1, 1, 1, 1, 1};
        return input;
    }

    void RuckigWrapper::GenerateTrajectory(const std::array<double, 6> &current_positions, const std::array<double, 6> &target_positions, Limits &limits, double &t0)
    {
        JointTrajectoryPoint js_tp;

        InputParameter<6> input = GetRuckigInputs(current_positions, target_positions, limits);
        if (!this->traj_queue_.empty())
            this->traj_queue_.clear();

        OutputParameter<6> output;

        while (traj_solver_.update(input, output) == Result::Working)
        {
            // cout << "positions ";
            for (int i = 0; i < 6; i++)
            {
                js_tp.position[i] = output.new_position[i];
                js_tp.velocity[i] = output.new_velocity[i];
                js_tp.acceleration[i] = output.new_acceleration[i];
                // cout << js_tp.position[i] << " ";
            }
            // cout << endl;
            js_tp.time = t0 + output.time;
            this->traj_queue_.push_back(js_tp);
            output.pass_to_input(input);
        }
        t0 = this->traj_queue_.back().time;
    }
}