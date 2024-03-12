// NOTE: pinocchio needs to be included before other things to prevent the
// compiler fussing
#include "upright_control/MobileManipulatorInterface.h"

#include <hpp/fcl/shape/geometric_shapes.h>
//#include <ocs2_core/constraint/BoundConstraint.h>
#include <ocs2_core/constraint/LinearStateConstraint.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_core/penalties/penalties/DoubleSidedPenalty.h>
#include <ocs2_core/penalties/penalties/QuadraticPenalty.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/penalties/penalties/SquaredHingePenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>
#include <ocs2_sqp/MultipleShootingMpc.h>

#include <upright_control/constraint/BoundedBalancingConstraints.h>
#include <upright_control/constraint/EndEffectorBoxConstraint.h>
#include <upright_control/constraint/JointStateInputLimits.h>
#include <upright_control/constraint/ObstacleConstraint.h>
#include <upright_control/constraint/ProjectilePathConstraint.h>
#include <upright_control/constraint/ProjectilePlaneConstraint.h>
#include <upright_control/constraint/StateToStateInputConstraint.h>

#include <upright_control/cost/EndEffectorCost.h>
#include <upright_control/cost/QuadraticJointStateInputCost.h>

#include <upright_control/dynamics/BaseType.h>
#include <upright_control/dynamics/SystemDynamics.h>
#include <upright_control/dynamics/SystemPinocchioMapping.h>
#include <upright_control/dynamics/MobileManipulatorInfo.h>

#include <upright_control/InertialAlignment.h>
#include <upright_control/common/FactoryFunctions.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>