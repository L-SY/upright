//
// Created by lsy on 24-5-23.
//

#include "arm_pinocchio_interface/EndEffectorInterface.h"

namespace arm_pinocchio
{
template <typename SCALAR_T>
EndEffectorInterface<SCALAR_T>::EndEffectorInterface(const PinocchioInterface& pinocchioInterface,
                                                     const std::vector<std::string>& endEffectorNames)
  : pinocchioInterfacePtr_(nullptr), endEffectorNames_(endEffectorNames)
{
  for (const auto& name : endEffectorNames_)
  {
    endEffectorFrameIds_.push_back(pinocchioInterface.getModel().getFrameId(name));
  }
}

template <typename SCALAR_T>
void EndEffectorInterface<SCALAR_T>::update(const vector_t& q, const vector_t& v)
{
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }
  auto model = pinocchioInterfacePtr_->getModel();
  auto data = pinocchioInterfacePtr_->getData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateGlobalPlacements(model, data);

  updateDynamics(q, v);
  updateKinematics();
}

template <typename SCALAR_T>
void EndEffectorInterface<SCALAR_T>::updateDynamics(const vector_t& q, const vector_t& v)
{
  auto model = pinocchioInterfacePtr_->getModel();
  auto data = pinocchioInterfacePtr_->getData();
  pinocchio::crba(model, data, q);  // 计算惯性矩阵
  auto M = data.M;
  M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();

  dynamics_.M = data.M.template cast<SCALAR_T>();
  dynamics_.C = pinocchio::nonLinearEffects(model, data, q, v).template cast<SCALAR_T>();  // 计算科氏力和离心力
  dynamics_.G = pinocchio::computeGeneralizedGravity(model, data, q).template cast<SCALAR_T>();  // 计算重力
}

template <typename SCALAR_T>
void EndEffectorInterface<SCALAR_T>::updateKinematics()
{
  kinematics_.positions.clear();
  kinematics_.velocities.clear();
  kinematics_.jacobians.clear();

  auto model = pinocchioInterfacePtr_->getModel();
  auto data = pinocchioInterfacePtr_->getData();

  for (const auto& frameId : endEffectorFrameIds_)
  {
    pinocchio::MotionTpl<SCALAR_T> velocity =
        pinocchio::getFrameVelocity(model, data, frameId,
                                    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    kinematics_.velocities.push_back(velocity.toVector().template cast<SCALAR_T>());

    Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic> jacobian =
        Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, frameId,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
    kinematics_.jacobians.push_back(jacobian.template topRows<3>());
  }
}

template <typename SCALAR_T>
const DynamicsInterface<SCALAR_T>& EndEffectorInterface<SCALAR_T>::getDynamics() const
{
  return dynamics_;
}

template <typename SCALAR_T>
const KinematicsInterface<SCALAR_T>& EndEffectorInterface<SCALAR_T>::getKinematics() const
{
  return kinematics_;
}

}  // namespace arm_pinocchio
template class arm_pinocchio::EndEffectorInterface<double>;
