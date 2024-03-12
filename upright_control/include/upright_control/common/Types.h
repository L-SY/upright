#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>

#include <upright_core/Types.h>

// Specialize the template types from upright_core with the scalars
// from OCS2.

namespace upright {

// Normal scalar (double)
// using Real = ocs2::scalar_t;

    using VecXd = VecX<ocs2::scalar_t>;
    using MatXd = MatX<ocs2::scalar_t>;
    using Vec2d = Vec2<ocs2::scalar_t>;
    using Mat2d = Mat2<ocs2::scalar_t>;
    using Vec3d = Vec3<ocs2::scalar_t>;
    using Mat3d = Mat3<ocs2::scalar_t>;
    using Mat23d = Mat23<ocs2::scalar_t>;
    using Quatd = Eigen::Quaternion<ocs2::scalar_t>;

// Auto-diff scalar
// using AD = ocs2::ad_scalar_t;

    using VecXad = VecX<ocs2::ad_scalar_t>;
    using MatXad = MatX<ocs2::ad_scalar_t>;
    using Vec2ad = Vec2<ocs2::ad_scalar_t>;
    using Mat2ad = Mat2<ocs2::ad_scalar_t>;
    using Vec3ad = Vec3<ocs2::ad_scalar_t>;
    using Mat3ad = Mat3<ocs2::ad_scalar_t>;
    using Mat23ad = Mat23<ocs2::ad_scalar_t>;
    using Quatad = Eigen::Quaternion<ocs2::ad_scalar_t>;

}  // namespace upright
