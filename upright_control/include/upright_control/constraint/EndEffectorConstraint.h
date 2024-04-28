/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <memory>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace upright
{

class EndEffectorConstraint final : public ocs2::StateConstraint
{
public:
  using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;
  using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;

  EndEffectorConstraint(const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                        const ocs2::ReferenceManager& referenceManager);
  ~EndEffectorConstraint() override = default;
  EndEffectorConstraint* clone() const override
  {
    return new EndEffectorConstraint(*endEffectorKinematicsPtr_, *referenceManagerPtr_);
  }

  size_t getNumConstraints(ocs2::scalar_t time) const override;
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
                          const ocs2::PreComputation& preComputation) const override;
  ocs2::VectorFunctionLinearApproximation
  getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                         const ocs2::PreComputation& preComputation) const override;

private:
  EndEffectorConstraint(const EndEffectorConstraint& other) = default;
  std::pair<ocs2::vector_t, quaternion_t> interpolateEndEffectorPose(ocs2::scalar_t time) const;

  /** Cached pointer to the pinocchio end effector kinematics. Is set to nullptr if not used. */
  ocs2::PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

  vector3_t eeDesiredPosition_;
  quaternion_t eeDesiredOrientation_;
  std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;
  const ocs2::ReferenceManager* referenceManagerPtr_;
};

}  // namespace upright