#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/common/value.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/kernel/mobod_impl.h"
#include "drake/multibody/kernel/revolute_mobod_data.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T, int axis>
class RevoluteMobod;

template <typename T, int axis>
struct MobodTraits<RevoluteMobod<T, axis>> {
  using Scalar = T;
  static constexpr int num_configurations = 1;
  static constexpr int num_velocities = 1;
  using ConfigVector = Eigen::Vector<T, num_configurations>;
  using VelocitiesVector = Eigen::Vector<T, num_configurations>;
  using MobodRigidTransform = RevoluteMobodRigidTransform<T, axis>;
  using MobodSpatialMotion = RevoluteMobodSpatialMotion<T, axis>;
  using PositionKinematicsData = RevoluteMobodPositionKinematicsData<T, axis>;
  using VelocityKinematicsData = RevoluteMobodVelocityKinematicsData<T, axis>;
  using AccelerationKinematicsData =
      RevoluteMobodAccelerationKinematicsData<T, axis>;
};

template <typename T, int axis>
class RevoluteMobod : public MobodImpl<RevoluteMobod<T, axis>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RevoluteMobod);

  // TODO: Consider placing these in a macro so that we can reuse this code in
  // several mobods.
  using Base = MobodImpl<RevoluteMobod<T, axis>>;
  using Traits = MobodTraits<RevoluteMobod>;
  static constexpr int num_configurations = Traits::num_configurations;
  static constexpr int num_velocities = Traits::num_velocities;
  using ConfigVector = typename Traits::ConfigVector;
  using VelocitiesVector = typename Traits::VelocitiesVector;
  using MobodSpatialMotion = typename Traits::MobodSpatialMotion;
  using PositionKinematicsData = typename Traits::PositionKinematicsData;
  using VelocityKinematicsData = typename Traits::VelocityKinematicsData;
  using AccelerationKinematicsData =
      typename Traits::AccelerationKinematicsData;

  // Inherit the constructor from the base class.
  // using Base::Base;

  RevoluteMobod() = default;

  explicit RevoluteMobod(MobodInfo info) : Base(std::move(info)){};

 private:
  /* Friend the base class so that MobodImpl::Foo() can reach its
   implementation, DoFoo(), in this class. */
  friend MobodImpl<RevoluteMobod>;

  void DoProjectSpatialForce(const SpatialForce<T>& F_BMo_M,
                             VelocitiesVector* tau) const {
    const Vector3<T>& t = F_BMo_M.rotational();
    (*tau)(0) = t(axis);
  }

  void DoCalcPositionKinematicsData(const ConfigVector& q,
                                    PositionKinematicsData* data) const {
    using std::cos;
    using std::sin;
    const T& theta = q[0];
    data->sin_q = sin(theta);
    data->cos_q = cos(theta);
    data->X_FM = RevoluteMobodRigidTransform<T, axis>(theta);
  }

  void DoCalcVelocityKinematicsData(
      const ConfigVector& q, const VelocitiesVector& v,
      const PositionKinematicsData& position_kinematics,
      VelocityKinematicsData* velocity_kinematics) const {
    unused(q);
    unused(position_kinematics);
    const T& theta_dot = v[0];
    velocity_kinematics->V_FM = RevoluteMobodSpatialMotion<T, axis>(theta_dot);
  }

  void DoCalcAccelerationKinematicsData(const ConfigVector&,
                                        const VelocitiesVector&,
                                        const VelocitiesVector& v_dot,
                                        const PositionKinematicsData&,
                                        const VelocityKinematicsData&,
                                        AccelerationKinematicsData* ak) const {
    const T& omega_dot = v_dot[0];
    ak->A_FM_M = RevoluteMobodSpatialMotion<T, axis>(omega_dot);
  }

  void DoAccumulateAccelerationBias(const Vector3<T>& w_WM_M,
                                    const MobodSpatialMotion& V_FM_M,
                                    SpatialAcceleration<T>* Ab_M) const {
    // In this case v_FM = 0, thus 2 * w_WP_M x v_FM_M = 0.
    Vector3<T>& alpha_bias = Ab_M->rotational();
    // alpha_bias += w_WM_M x w_FM_M ( = theta_dot ⋅ w_WM_M×eᵢ)
    CrossProductWithBasisVector<T, axis>::CalcAndAdd(w_WM_M, V_FM_M.theta_dot(),
                                                     &alpha_bias);
  }
};

template <typename T>
struct RevoluteMobodX : public RevoluteMobod<T, 0> {};

template <typename T>
struct RevoluteMobodY : public RevoluteMobod<T, 1> {};

template <typename T>
struct RevoluteMobodZ : public RevoluteMobod<T, 2> {};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
