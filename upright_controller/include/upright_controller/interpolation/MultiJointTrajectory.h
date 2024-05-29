//
// Created by lsy on 24-5-29.
//

#pragma once

#include "CubicSpline.h"
#include <vector>
#include <map>
#include <stdexcept>

namespace interpolation
{
class MultiJointTrajectory
{
public:
  // Constructor to initialize the class with joint trajectories
  MultiJointTrajectory(const std::map<int, std::vector<CubicSpline::Node>>& jointTrajectories);

  // Method to get the position, velocity, and acceleration at a given time
  std::vector<CubicSpline::Node> getStateAtTime(scalar_t time) const;

private:
  // Helper method to create splines for each joint
  void createSplines(const std::map<int, std::vector<CubicSpline::Node>>& jointTrajectories);

  std::map<int, std::vector<CubicSpline>> splines_;  // Map to hold splines for each joint
};
}  // namespace interpolation