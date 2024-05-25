/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following
conditions are met:

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

#include <Eigen/Dense>
#include <iosfwd>
#include <memory>
#include <string>
#include <type_traits>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <urdf_model/model.h>

namespace arm_pinocchio {

class PinocchioInterface {
public:
  using Model = pinocchio::ModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;
  using Data = pinocchio::DataTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;
  using JointModel = pinocchio::JointModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>;

  /**
   * Construct from given pinocchio model
   * @param[in] model pinocchio model
   */
  explicit PinocchioInterface(const Model& model, const std::shared_ptr<const ::urdf::ModelInterface> urdfModelPtr = nullptr);

  /** Destructor */
  ~PinocchioInterface();

  /** Copy constructor */
  PinocchioInterface(const PinocchioInterface& rhs);

  /** Move constructor */
  PinocchioInterface(PinocchioInterface&& rhs) noexcept;

  /** Copy assignment operator */
  PinocchioInterface& operator=(const PinocchioInterface& rhs);

  /** Move assignment */
  PinocchioInterface& operator=(PinocchioInterface&& rhs) noexcept;

  /** Get the pinocchio model */
  const Model& getModel() const {
    return *robotModelPtr_;
  }

  /** Get the pinocchio data */
  Data& getData() {
    return *robotDataPtr_;
  }
  const Data& getData() const {
    return *robotDataPtr_;
  }

  /** Get the urdf model */
  const std::shared_ptr<const ::urdf::ModelInterface>& getUrdfModelPtr() const {
    return urdfModelPtr_;
  }

  friend std::ostream& operator<<(std::ostream& os, const PinocchioInterface& p);

private:
  std::shared_ptr<const Model> robotModelPtr_;
  std::unique_ptr<Data> robotDataPtr_;
  std::shared_ptr<const ::urdf::ModelInterface> urdfModelPtr_;
};

/** Print PinocchioInterface info to stream */
std::ostream& operator<<(std::ostream& os, const PinocchioInterface& p);

}  // namespace arm_pinocchio
