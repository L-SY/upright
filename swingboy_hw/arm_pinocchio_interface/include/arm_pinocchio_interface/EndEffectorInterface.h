//
// Created by lsy on 24-5-23.
//

#pragma once

#include "arm_pinocchio_interface/PinocchioInterface.h"
#include <Eigen/Dense>
#include <memory>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <string>
#include <vector>

namespace arm_pinocchio {
template <typename SCALAR_T> struct DynamicsInterface {
  using matrix_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;

  matrix_t M; // 惯性矩阵
  vector_t C; // 科氏力和离心力
  vector_t G; // 重力
};

template <typename SCALAR_T> struct KinematicsInterface {
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using vector6_t = Eigen::Matrix<SCALAR_T, 6, 1>;
  using matrix3x_t = Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic>;

  std::vector<vector3_t> positions;
  std::vector<vector6_t> velocities;
  std::vector<matrix3x_t> jacobians;
};

template <typename SCALAR_T> class EndEffectorInterface {
public:
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3x_t = Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
  using quaternion_t = Eigen::Quaternion<SCALAR_T>;

  EndEffectorInterface(const PinocchioInterface &pinocchioInterface,
                       const std::vector<std::string> &endEffectorNames);

  void update(const vector_t &q, const vector_t &v);

  const DynamicsInterface<SCALAR_T> &getDynamics() const;
  const KinematicsInterface<SCALAR_T> &getKinematics() const;

  void setPinocchioInterface(const PinocchioInterface &pinocchioInterface) {
    pinocchioInterfacePtr_ = &pinocchioInterface;
  }

  PinocchioInterface getPinocchioInterface() { return *pinocchioInterfacePtr_; }

private:
  void updateDynamics(const vector_t &q, const vector_t &v);
  void updateKinematics();

  const PinocchioInterface *pinocchioInterfacePtr_;
  std::vector<std::string> endEffectorNames_;
  std::vector<pinocchio::FrameIndex> endEffectorFrameIds_;
  DynamicsInterface<SCALAR_T> dynamics_;
  KinematicsInterface<SCALAR_T> kinematics_;
};

} // namespace arm_pinocchio
