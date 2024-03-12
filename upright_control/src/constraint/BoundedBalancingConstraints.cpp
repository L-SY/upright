#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <upright_control/common/Types.h>
#include "upright_control/constraint/BoundedBalancingConstraints.h"

#include <upright_core/Bounded.h>
#include <upright_core/BoundedConstraints.h>
#include <upright_core/Contact.h>
#include <upright_core/ContactConstraints.h>
#include <upright_core/Nominal.h>

namespace upright {

    std::ostream& operator<<(std::ostream& out, const BalancingSettings& settings) {
        out << "enabled = " << settings.enabled << std::endl
            << "normal enabled = " << settings.constraints_enabled.normal
            << std::endl
            << "friction enabled = " << settings.constraints_enabled.friction
            << std::endl
            << "ZMP enabled = " << settings.constraints_enabled.zmp << std::endl
            << "num objects = " << settings.objects.size() << std::endl
            << "mu = " << settings.mu << std::endl
            << "delta = " << settings.delta << std::endl;
        return out;
    }

    RigidBodyState<ocs2::ad_scalar_t> get_rigid_body_state(
            const std::unique_ptr<ocs2::PinocchioEndEffectorKinematicsCppAd>&kinematics_ptr,
            const VecXad& state, const VecXad& input) {
        RigidBodyState<ocs2::ad_scalar_t> X;
        X.pose.position = kinematics_ptr->getPositionCppAd(state);
        X.pose.orientation = kinematics_ptr->getOrientationCppAd(state);

        X.velocity.linear = kinematics_ptr->getVelocityCppAd(state, input);
        X.velocity.angular = kinematics_ptr->getAngularVelocityCppAd(state, input);

        X.acceleration.linear = kinematics_ptr->getAccelerationCppAd(state, input);
        X.acceleration.angular =
                kinematics_ptr->getAngularAccelerationCppAd(state, input);
        return X;
    }

    NominalBalancingConstraints::NominalBalancingConstraints(
            const ocs2::PinocchioEndEffectorKinematicsCppAd& pinocchioEEKinematics,
            const BalancingSettings& settings, const Vec3d& gravity,
            const OptimizationDimensions& dims, bool recompileLibraries)
            : ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear),
              pinocchioEEKinPtr_(pinocchioEEKinematics.clone()),
              gravity_(gravity),
              settings_(settings),
              arrangement_(settings.objects, settings.constraints_enabled, gravity),
              dims_(dims) {
        if (pinocchioEEKinematics.getIds().size() != 1) {
            throw std::runtime_error(
                    "[TrayBalanaceConstraint] endEffectorKinematics has wrong "
                    "number of end effector IDs.");
        }

        // NOTE: workaround for CppADCodeGen slow compilation for single objects
        // if (arrangement_.objects.size() == 1) {
        //     auto it = arrangement_.objects.begin();
        //     arrangement_.objects.emplace("foo", it->second);
        // }

        // compile the CppAD library
        initialize(dims.x(), dims.u(), 0, "upright_nominal_balancing_constraints",
                   "/tmp/ocs2", recompileLibraries, true);
    }

    VecXad NominalBalancingConstraints::constraintFunction(
            ocs2::ad_scalar_t time, const VecXad& state, const VecXad& input,
            const VecXad& parameters) const {
        RigidBodyState<ocs2::ad_scalar_t> X =
                get_rigid_body_state(pinocchioEEKinPtr_, state, input);

        BalancedObjectArrangement<ocs2::ad_scalar_t> ad_arrangement =
                arrangement_.cast<ocs2::ad_scalar_t>();
        return ad_arrangement.balancing_constraints(X);
    }

    ContactForceBalancingConstraints::ContactForceBalancingConstraints(
            const ocs2::PinocchioEndEffectorKinematicsCppAd& pinocchioEEKinematics,
            const BalancingSettings& settings, const Vec3d& gravity,
            const OptimizationDimensions& dims, bool recompileLibraries)
            : ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear),
              pinocchioEEKinPtr_(pinocchioEEKinematics.clone()),
              gravity_(gravity),
              settings_(settings),
              dims_(dims) {
        if (pinocchioEEKinematics.getIds().size() != 1) {
            throw std::runtime_error(
                    "[TrayBalanaceConstraint] endEffectorKinematics has wrong "
                    "number of end effector IDs.");
        }

        // Important: this needs to come before the call to initialize, because it
        // is used in the constraintFunction which is called therein
        const bool frictionless = (dims.nf == 1);
        if (frictionless) {
            num_constraints_ = settings_.contacts.size();
        } else {
            num_constraints_ = settings_.contacts.size() *
                               NUM_LINEARIZED_FRICTION_CONSTRAINTS_PER_CONTACT;
        }

        // compile the CppAD library
        const std::string lib_name = "upright_contact_force_constraints";
        initialize(dims.x(), dims.u(), 0, lib_name, "/tmp/ocs2", recompileLibraries,
                   true);
    }

    VecXad ContactForceBalancingConstraints::constraintFunction(
            ocs2::ad_scalar_t time, const VecXad& state, const VecXad& input,
            const VecXad& parameters) const {
        // All forces are expressed in the EE frame
        VecXad forces = input.tail(dims_.f());

        std::vector<ContactPoint<ocs2::ad_scalar_t>> ad_contacts;
        for (auto& contact : settings_.contacts) {
            ad_contacts.push_back(contact.template cast<ocs2::ad_scalar_t>());
        }

        ocs2::ad_scalar_t n(ad_contacts.size());
        return compute_contact_force_constraints_linearized(ad_contacts, forces);
    }

    ObjectDynamicsConstraints::ObjectDynamicsConstraints(
            const ocs2::PinocchioEndEffectorKinematicsCppAd& pinocchioEEKinematics,
            const BalancingSettings& settings, const Vec3d& gravity,
            const OptimizationDimensions& dims, bool recompileLibraries)
            : ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear),
              pinocchioEEKinPtr_(pinocchioEEKinematics.clone()),
              gravity_(gravity),
              settings_(settings),
              dims_(dims) {
        if (pinocchioEEKinematics.getIds().size() != 1) {
            throw std::runtime_error(
                    "[TrayBalanaceConstraint] endEffectorKinematics has wrong "
                    "number of end effector IDs.");
        }

        // Six constraints per object: three linear and three rotational.
        num_constraints_ =
                settings_.objects.size() * NUM_DYNAMICS_CONSTRAINTS_PER_OBJECT;

        // compile the CppAD library
        const std::string lib_name = "upright_object_dynamics_constraints";
        initialize(dims.x(), dims.u(), 0, lib_name, "/tmp/ocs2", recompileLibraries,
                   true);
    }

    VecXad ObjectDynamicsConstraints::constraintFunction(
            ocs2::ad_scalar_t time, const VecXad& state, const VecXad& input,
            const VecXad& parameters) const {
        // All forces are expressed in the EE frame
        VecXad forces = input.tail(dims_.f());

        RigidBodyState<ocs2::ad_scalar_t> X =
                get_rigid_body_state(pinocchioEEKinPtr_, state, input);

        // Convert contact points to AD type
        std::vector<ContactPoint<ocs2::ad_scalar_t>> ad_contacts;
        for (const auto& contact : settings_.contacts) {
            ad_contacts.push_back(contact.template cast<ocs2::ad_scalar_t>());
        }

        // Convert objects to AD type
        std::map<std::string, BalancedObject<ocs2::ad_scalar_t>> ad_objects;
        for (const auto& kv : settings_.objects) {
            auto obj_ad = kv.second.template cast<ocs2::ad_scalar_t>();
            ad_objects.emplace(kv.first, obj_ad);
        }

        Vec3ad ad_gravity = gravity_.template cast<ocs2::ad_scalar_t>();

        // Normalizing by the number of constraints appears to improve the
        // convergence of the controller (cost landscape is better behaved)
        // TODO
        ocs2::ad_scalar_t n(sqrt(6 * ad_objects.size()));
        return compute_object_dynamics_constraints(ad_objects, ad_contacts, forces,
                                                   X, ad_gravity) / n;
    }

}  // namespace upright
