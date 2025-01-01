#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <iostream>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class OmplTestNode : public rclcpp::Node
{
public:
  OmplTestNode()
  : Node("ompl_test_node")
  {
    RCLCPP_INFO(this->get_logger(), "OMPL Test Node Started");

    // Here we run the exact code you had, but wrapped in a function
    runTest();
  }

private:
  void runTest()
  {
    // Define the state space (2D in this case)
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);

    // Set bounds for the space
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    // Create a simple setup
    og::SimpleSetup ss(space);

    // Define the start and goal states
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = -5;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = -5;

    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 5;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 5;

    ss.setStartAndGoalStates(start, goal);

    // Set the planner
    ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

    // Attempt to solve the planning problem
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
      // Print the path in the console
      ss.simplifySolution();
      RCLCPP_INFO(this->get_logger(), "Solution found. Path:");
      std::stringstream path_ss;
      ss.getSolutionPath().print(path_ss);
      RCLCPP_INFO(this->get_logger(), "%s", path_ss.str().c_str());
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No solution found.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OmplTestNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
