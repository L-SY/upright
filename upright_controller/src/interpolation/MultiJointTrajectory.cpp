//
// Created by lsy on 24-5-29.
//

#include "upright_controller/interpolation/MultiJointTrajectory.h"

namespace interpolation
{
MultiJointTrajectory::MultiJointTrajectory(const std::map<int, std::vector<CubicSpline::Node>>& jointTrajectories)
{
  createSplines(jointTrajectories);
}

void MultiJointTrajectory::createSplines(const std::map<int, std::vector<CubicSpline::Node>>& jointTrajectories)
{
  for (const auto& [jointId, nodes] : jointTrajectories)
  {
    if (nodes.size() < 2)
    {
      throw std::invalid_argument("Each joint must have at least two nodes to create splines.");
    }
    std::vector<CubicSpline> jointSplines;
    for (size_t i = 0; i < nodes.size() - 1; ++i)
    {
      jointSplines.emplace_back(nodes[i], nodes[i + 1]);
    }
    splines_[jointId] = jointSplines;
  }
}

std::vector<CubicSpline::Node> MultiJointTrajectory::getStateAtTime(scalar_t time) const
{
  std::vector<CubicSpline::Node> states;

  for (const auto& [jointId, jointSplines] : splines_)
  {
    for (const auto& spline : jointSplines)
    {
      if (time >= spline.startTime() && time <= spline.endTime())
      {
        CubicSpline::Node state{};
        state.time = time;
        state.position = spline.position(time);
        state.velocity = spline.velocity(time);
        states.push_back(state);
        break;
      }
    }
  }

  return states;
}
}  // namespace interpolation