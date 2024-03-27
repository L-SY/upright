#pragma once

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <upright_control/dynamics/Dimensions.h>
#include <upright_control/common/Types.h>

namespace upright {

template <typename Scalar>
class TripleIntegratorPinocchioMapping final
        : public ocs2::PinocchioStateInputMapping<Scalar> {
public:
    using VecXs = VecX<Scalar>;
    using MatXs = MatX<Scalar>;

    TripleIntegratorPinocchioMapping(const RobotDimensions& dims) : dims_(dims) {}

    ~TripleIntegratorPinocchioMapping() override = default;

    TripleIntegratorPinocchioMapping<Scalar>* clone() const override {
        return new TripleIntegratorPinocchioMapping<Scalar>(*this);
    }

        VecXs getPinocchioJointPosition(const VecXs& state) const override {
            return state.head(dims_.q);
        }

        VecXs getPinocchioJointVelocity(const VecXs& state,
                                        const VecXs& input) const override {
            return state.segment(dims_.q, dims_.v);
        }

        VecXs getPinocchioJointAcceleration(const VecXs& state,
                                            const VecXs& input) const override {
            return state.tail(dims_.v);
        }

        // Maps the Jacobians of an arbitrary function f w.r.t q and v (generalized
        // positions and velocities), as provided by Pinocchio as Jq and Jv, to the
        // Jacobian of the state dfdx and Jacobian of the input dfdu.
        std::pair<MatXs, MatXs> getOcs2Jacobian(const VecXs& state, const MatXs& Jq,
                                                const MatXs& Jv) const override {
            const auto output_dim = Jq.rows();
            MatXs dfdx(output_dim, Jq.cols() + Jv.cols() + dims_.v);
            dfdx << Jq, Jv, MatXs::Zero(output_dim, dims_.v);

            // NOTE: this isn't used for collision avoidance (which is the only
            // place this method is called)
            MatXs dfdu = MatXs::Zero(output_dim, dims_.u);

            return {dfdx, dfdu};
        }

    private:
        RobotDimensions dims_;
    };

    template <typename Scalar>
    class NonholonomicPinocchioMapping final
            : public ocs2::PinocchioStateInputMapping<Scalar> {
    public:
        using VecXs = VecX<Scalar>;
        using MatXs = MatX<Scalar>;

        NonholonomicPinocchioMapping(const RobotDimensions& dims) : dims_(dims) {}

        ~NonholonomicPinocchioMapping() override = default;

        NonholonomicPinocchioMapping<Scalar>* clone() const override {
            return new NonholonomicPinocchioMapping<Scalar>(*this);
        }

        VecXs getPinocchioJointPosition(const VecXs& state) const override {
            std::cout << "JointPos" << std::endl;
            return state.head(dims_.q);
        }

        VecXs getPinocchioJointVelocity(const VecXs& state,const VecXs& input) const override {
            VecXs vPinocchio(dims_.q);
            const auto theta = state(2);
            const auto v = state(dims_.q);  // forward velocity in base frame
            const auto vLast = state.segment(dims_.q+1,dims_.v -1);
            vPinocchio << cos(theta) * v, sin(theta) * v, vLast;
            std::cout << "JointVel" << std::endl;
            return vPinocchio;
        }

        VecXs getPinocchioJointAcceleration(const VecXs& state,const VecXs& input) const override {
            std::cout << "~~~~~~~~~~~~~~~~" << std::endl;
            VecXs aPinocchio(dims_.q);
            const auto theta = state(2);
            const auto a = state(dims_.q + dims_.v);  // forward velocity in base frame
            const auto aLast = state.tail(dims_.v - 1);
            std::cout << aLast.size() << std::endl;
            aPinocchio << cos(theta) * a, sin(theta) * a, aLast;
            std::cout << "JointAcc" << std::endl;
            return aPinocchio;
        }

        // Maps the Jacobians of an arbitrary function f w.r.t q and v (generalized
        // positions and velocities), as provided by Pinocchio as Jq and Jv, to the
        // Jacobian of the state dfdx and Jacobian of the input dfdu.

        std::pair<MatXs, MatXs> getOcs2Jacobian(const VecXs& state, const MatXs& Jq,
                                                const MatXs& Jv) const override {

//        const auto output_dim = Jq.rows();
            std::cout << "!!!!!!!!!!!!!!!" << std::endl;
            const auto output_dim = Jv.rows();
            std::cout << "!!!!!!!!!!!!!!!" << std::endl;
            MatXs dfdx(output_dim, Jq.cols() + Jv.cols() + dims_.v);
            std::cout << "!!!!!!!!!!!!!!!" << std::endl;
            dfdx << Jq, Jv, MatXs::Zero(output_dim, dims_.v);
            std::cout << "!!!!!!!!!!!!!!!" << std::endl;
            // NOTE: this isn't used for collision avoidance (which is the only
            // place this method is called)
            MatXs dfdu = MatXs::Zero(output_dim, dims_.u);
            std::cout << "!!!!!!!!!!!!!!!" << std::endl;
            return {dfdx, dfdu};
        }
    private:
        RobotDimensions dims_;
    };


static const RobotDimensions OBSTACLE_DIMENSIONS{3, 3, 9, 0};

template <typename Scalar>
static const TripleIntegratorPinocchioMapping<Scalar> OBSTACLE_PINOCCHIO_MAPPING{
        OBSTACLE_DIMENSIONS};

template <typename Mapping, typename Scalar>
class SystemPinocchioMapping final
        : public ocs2::PinocchioStateInputMapping<Scalar> {
public:
    using VecXs = VecX<Scalar>;
    using MatXs = MatX<Scalar>;

    explicit SystemPinocchioMapping(const OptimizationDimensions& dims)
            : dims_(dims), robot_mapping_(dims.robot) {}

    ~SystemPinocchioMapping() override = default;

    SystemPinocchioMapping<Mapping, Scalar>* clone() const override {
        return new SystemPinocchioMapping<Mapping, Scalar>(*this);
    }

    VecXs getPinocchioJointPosition(const VecXs& state) const override {
        VecXs q_pin(dims_.q());

        // For now, we assume all obstacles go first in list of q, v
        for (int i = 0; i < dims_.o; ++i) {
            VecXs x_obs = state.segment(dims_.robot.x + i * 9, 9);
            q_pin.segment(i * 3, 3) =
                    OBSTACLE_PINOCCHIO_MAPPING<Scalar>.getPinocchioJointPosition(
                            x_obs);
        }

        // Then we add on the robot q
        VecXs x_robot = state.head(dims_.robot.x);
        q_pin.tail(dims_.robot.q) =
                robot_mapping_.getPinocchioJointPosition(x_robot);
        std::cout << "HERE!!!!!!!!!!!!!" << std::endl;
        return q_pin;
    }

    VecXs getPinocchioJointVelocity(const VecXs& state,
                                    const VecXs& input) const override {
        VecXs v_pin(dims_.v());
        VecXs u_obs = VecXs::Zero(3);  // Obstacles have no input

        for (int i = 0; i < dims_.o; ++i) {
            VecXs x_obs = state.segment(dims_.robot.x + i * 9, 9);
            v_pin.segment(i * 3, 3) =
                    OBSTACLE_PINOCCHIO_MAPPING<Scalar>.getPinocchioJointVelocity(
                            x_obs, u_obs);
        }

        // Then we add on the robot v
        VecXs x_robot = state.head(dims_.robot.x);
        VecXs u_robot = input.head(dims_.robot.u);
        v_pin.tail(dims_.robot.v) =
                robot_mapping_.getPinocchioJointVelocity(x_robot, u_robot);

        return v_pin;
    }

    VecXs getPinocchioJointAcceleration(const VecXs& state,
                                        const VecXs& input) const override {
        VecXs a_pin(dims_.v());
        VecXs u_obs = VecXs::Zero(3);  // Obstacles have no input

        for (int i = 0; i < dims_.o; ++i) {
            VecXs x_obs = state.segment(dims_.robot.x + i * 9, 9);
            a_pin.segment(i * 3, 3) =
                    OBSTACLE_PINOCCHIO_MAPPING<Scalar>.getPinocchioJointAcceleration(
                            x_obs, u_obs);
        }

        // Then we add on the robot a
        VecXs x_robot = state.head(dims_.robot.x);
        VecXs u_robot = input.head(dims_.robot.u);
        a_pin.tail(dims_.robot.v) =
                robot_mapping_.getPinocchioJointAcceleration(x_robot, u_robot);

        return a_pin;
    }

    // Maps the Jacobians of an arbitrary function f w.r.t q and v (generalized
    // positions and velocities), as provided by Pinocchio as Jq_pin and
    // Jv_pin, to the Jacobian of the state dfdx and Jacobian of the input
    // dfdu.
    std::pair<MatXs, MatXs> getOcs2Jacobian(
            const VecXs& state, const MatXs& Jq_pin,
            const MatXs& Jv_pin) const override {
        const auto output_dim = Jq_pin.rows();
        MatXs dfdx(output_dim, dims_.x());
        MatXs dfdu(output_dim, dims_.u());

        for (int i = 0; i < dims_.o; ++i) {
            VecXs x_obs = state.segment(dims_.robot.x + i * 9, 9);
            MatXs Jq_pin_obs = Jq_pin.middleCols(i * 3, 3);
            MatXs Jv_pin_obs = Jv_pin.middleCols(i * 3, 3);

            MatXs dfdx_obs;
            std::tie(dfdx_obs, std::ignore) =
                    OBSTACLE_PINOCCHIO_MAPPING<Scalar>.getOcs2Jacobian(
                            x_obs, Jq_pin_obs, Jv_pin_obs);

            // Obstacles have no input, so no dfdu
            dfdx.middleCols(dims_.robot.x + i * 9, 9) = dfdx_obs;
        }

        VecXs x_robot = state.head(dims_.robot.x);
        // Recall that robot q, v are after the obstacle q, v in Pinocchio
        MatXs Jq_pin_robot = Jq_pin.rightCols(dims_.robot.q);
        MatXs Jv_pin_robot = Jv_pin.rightCols(dims_.robot.v);
        MatXs dfdx_robot, dfdu_robot;
        std::tie(dfdx_robot, dfdu_robot) =
                robot_mapping_.getOcs2Jacobian(x_robot, Jq_pin_robot, Jv_pin_robot);

        dfdx.leftCols(dims_.robot.x) = dfdx_robot;
        dfdu.leftCols(dims_.robot.u) = dfdu_robot;

        return {dfdx, dfdu};
    }

private:
    OptimizationDimensions dims_;
    Mapping robot_mapping_;
};

}  // namespace upright
