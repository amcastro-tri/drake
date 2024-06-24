#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/common/value.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/kernel/mobod_impl.h"
#include "drake/multibody/kernel/quaternion_floating_mobod_data.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
class QuaternionFloatingMobod;

template <typename T>
struct MobodTraits<QuaternionFloatingMobod<T>> {
  using Scalar = T;
  static constexpr int num_configurations = 7;
  static constexpr int num_velocities = 6;
  using ConfigVector = Eigen::Vector<T, num_configurations>;
  using VelocitiesVector = Eigen::Vector<T, num_velocities>;
  using MobodRigidTransform = math::RigidTransform<T>;
  using MobodSpatialMotion = QuaternionFloatingMobodSpatialMotion<T>;
  using PositionKinematicsData =
      QuaternionFloatingMobodPositionKinematicsData<T>;
  using VelocityKinematicsData =
      QuaternionFloatingMobodVelocityKinematicsData<T>;
  using AccelerationKinematicsData =
      QuaternionFloatingMobodAccelerationKinematicsData<T>;
};

template <typename T>
class QuaternionFloatingMobod : public MobodImpl<QuaternionFloatingMobod<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(QuaternionFloatingMobod);

  // TODO: Consider placing these in a macro so that we can reuse this code in
  // several mobods.
  using Base = MobodImpl<QuaternionFloatingMobod<T>>;
  using Traits = MobodTraits<QuaternionFloatingMobod>;
  static constexpr int num_configurations = Traits::num_configurations;
  static constexpr int num_velocities = Traits::num_velocities;
  using ConfigVector = typename Traits::ConfigVector;
  using VelocitiesVector = typename Traits::VelocitiesVector;
  using MobodSpatialMotion = typename Traits::MobodSpatialMotion;
  using PositionKinematicsData = typename Traits::PositionKinematicsData;
  using VelocityKinematicsData = typename Traits::VelocityKinematicsData;
  using AccelerationKinematicsData =
      typename Traits::AccelerationKinematicsData;

  QuaternionFloatingMobod() = default;

  explicit QuaternionFloatingMobod(MobodInfo info) : Base(std::move(info)){};

 private:
  /* Friend the base class so that MobodImpl::Foo() can reach its
   implementation, DoFoo(), in this class. */
  friend MobodImpl<QuaternionFloatingMobod>;

  void DoProjectSpatialForce(const SpatialForce<T>& F_BMo_M,
                             VelocitiesVector* tau) const {
    // H_FM is the identity matrix in ℝ⁶, therefore the projection is a trivial
    // copy.
    (*tau) = F_BMo_M.get_coeffs();
  }

  void DoCalcPositionKinematicsData(const ConfigVector& q,
                                    PositionKinematicsData* data) const {
    // Note: In q we store the quaternion's components first, followed
    // by the position vector components. The quaternion components are stored
    // as a plain vector in the order: (qs, qv₁, qv₂, qv₃), where qs corresponds
    // to the "scalar" component of the quaternion and qv corresponds to the
    // "vector" component. Eigen::Quaternion's constructor takes the scalar
    // component first followed by the vector components.
    const Eigen::Quaternion<T> q_FM(q[0], q[1], q[2], q[3]);
    math::RotationMatrix<T> R_FM(q_FM);
    Vector3<T> p_FM(q.template tail<3>());
    data->X_FM = math::RigidTransform<T>(std::move(R_FM), std::move(p_FM));
  }

  void DoCalcVelocityKinematicsData(
      const ConfigVector&, const VelocitiesVector& v,
      const PositionKinematicsData&,
      VelocityKinematicsData* velocity_kinematics) const {
    velocity_kinematics->V_FM = MobodSpatialMotion(v);
  }

  void DoCalcAccelerationKinematicsData(const ConfigVector&,
                                        const VelocitiesVector&,
                                        const VelocitiesVector& v_dot,
                                        const PositionKinematicsData&,
                                        const VelocityKinematicsData&,
                                        AccelerationKinematicsData* ak) const {
    ak->A_FM_M = MobodSpatialMotion(v_dot);
  }

  void DoAccumulateAccelerationBias(const Vector3<T>& w_WM_M,
                                    const MobodSpatialMotion& V_FM_M,
                                    SpatialAcceleration<T>* Ab_M) const {
    Vector3<T>& alpha_bias = Ab_M->rotational();
    Vector3<T>& a_bias = Ab_M->translational();
    alpha_bias += w_WM_M.cross(w_WM_M);
    const auto w_FM_M = V_FM_M.rotational();
    const auto v_FM_M = V_FM_M.translational();
    a_bias += 2.0 * (w_WM_M - w_FM_M).cross(v_FM_M);
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
