//
// Created by lsy on 24-5-23.
//

#include "arm_pinocchio_interface/EndEffectorInterface.h"

namespace arm_pinocchio {
template <typename SCALAR_T>
EndEffectorInterface<SCALAR_T>::EndEffectorInterface(
    const PinocchioInterface &pinocchioInterface,
    const std::vector<std::string> &endEffectorNames)
    : pinocchioInterfacePtr_(nullptr), endEffectorNames_(endEffectorNames) {
  for (const auto &name : endEffectorNames_) {
    endEffectorFrameIds_.push_back(
        pinocchioInterface.getModel().getFrameId(name));
  }
}

template <typename SCALAR_T>
void EndEffectorInterface<SCALAR_T>::update(const vector_t &q,
                                            const vector_t &v) {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error(
        "[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. "
        "Use setPinocchioInterface()");
  }
  auto model = pinocchioInterfacePtr_->getModel();
  auto data = pinocchioInterfacePtr_->getData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateGlobalPlacements(model, data);

  //  updateDynamics(q, v);
  //  updateKinematics();
}

template <typename SCALAR_T>
void EndEffectorInterface<SCALAR_T>::updateDynamics(const vector_t &q,
                                                    const vector_t &v) {
  auto model = pinocchioInterfacePtr_->getModel();
  auto data = pinocchioInterfacePtr_->getData();
  assert(q.size() == model.nq && "Dimension of q does not match model.nq");
  assert(v.size() == model.nv && "Dimension of v does not match model.nv");

  std::cout << "Inter update Dyanmics" << std::endl;
  // 打印调试信息
  std::cout << "Model nq: " << model.nq << std::endl;
  std::cout << "Model nv: " << model.nv << std::endl;
  std::cout << "Input q size: " << q.size() << std::endl;
  std::cout << "Input v size: " << v.size() << std::endl;

  // 打印关节名称和ID
  for (size_t i = 0; i < model.names.size(); ++i) {
    std::cout << "Joint " << i << ": " << model.names[i] << std::endl;
  }
  // 计算惯性矩阵
  try {
    pinocchio::crba(model, data, q);
    std::cout << "Inertia Matrix M:\n" << data.M << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Error during crba computation: " << e.what() << std::endl;
  }
  auto M = data.M;
  std::cout << "Inertia Matrix M before symmetry adjustment:\n"
            << M << std::endl;
  M.triangularView<Eigen::StrictlyLower>() =
      M.transpose().triangularView<Eigen::StrictlyLower>();
  std::cout << "Inertia Matrix M after symmetry adjustment:\n"
            << M << std::endl;

  dynamics_.M = data.M.template cast<SCALAR_T>();
  std::cout << "Dynamics Inertia Matrix M:\n" << dynamics_.M << std::endl;

  // 计算科氏力和离心力
  auto C = pinocchio::nonLinearEffects(model, data, q, v);
  assert(C.size() == model.nv && "Dimension of C does not match model.nv");
  dynamics_.C = C.template cast<SCALAR_T>();
  std::cout << "Coriolis and Centrifugal Forces C:\n"
            << dynamics_.C << std::endl;

  // 计算重力
  pinocchio::computeGeneralizedGravity(model, data, q);
  auto G = data.g;
  assert(G.size() == model.nv && "Dimension of G does not match model.nv");
  dynamics_.G = G.template cast<SCALAR_T>();
  std::cout << "Gravity Forces G:\n" << dynamics_.G << std::endl;
}

template <typename SCALAR_T>
void EndEffectorInterface<SCALAR_T>::updateKinematics() {
  kinematics_.positions.clear();
  kinematics_.velocities.clear();
  kinematics_.jacobians.clear();

  auto model = pinocchioInterfacePtr_->getModel();
  auto data = pinocchioInterfacePtr_->getData();

  for (const auto &frameId : endEffectorFrameIds_) {
    pinocchio::MotionTpl<SCALAR_T> velocity = pinocchio::getFrameVelocity(
        model, data, frameId, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    kinematics_.velocities.push_back(
        velocity.toVector().template cast<SCALAR_T>());

    Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic> jacobian =
        Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, frameId,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                jacobian);
    kinematics_.jacobians.push_back(jacobian.template topRows<3>());
  }
}

template <typename SCALAR_T>
const DynamicsInterface<SCALAR_T> &
EndEffectorInterface<SCALAR_T>::getDynamics() const {
  return dynamics_;
}

template <typename SCALAR_T>
const KinematicsInterface<SCALAR_T> &
EndEffectorInterface<SCALAR_T>::getKinematics() const {
  return kinematics_;
}

} // namespace arm_pinocchio
template class arm_pinocchio::EndEffectorInterface<double>;
