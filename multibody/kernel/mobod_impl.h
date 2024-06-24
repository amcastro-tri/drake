#pragma once

#include <iostream>
#include <memory>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/value.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/kernel/acceleration_kinematics_data.h"
#include "drake/multibody/kernel/mobod_base.h"
#include "drake/multibody/kernel/multibody_kernel_parameters.h"
#include "drake/multibody/kernel/position_kinematics_data.h"
#include "drake/multibody/kernel/velocity_kinematics_data.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
class MultibodyKernel;

template <class DerivedMobod>
struct MobodTraits {};

template <class DerivedMobod>
class MobodImpl : public MobodBase<typename MobodTraits<DerivedMobod>::Scalar> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MobodImpl);
  using Traits = MobodTraits<DerivedMobod>;
  using T = typename Traits::Scalar;
  static constexpr int num_positions = Traits::num_configurations;
  static constexpr int num_velocities = Traits::num_velocities;
  // TODO: static_cast that ConfigVector is a fixed size Eigen::Vector.
  using ConfigVector = typename Traits::ConfigVector;
  using VelocitiesVector = typename Traits::VelocitiesVector;
  using MobodRigidTransform = typename Traits::MobodRigidTransform;
  using MobodSpatialMotion = typename Traits::MobodSpatialMotion;
  using DerivedPositionKinematicsData = typename Traits::PositionKinematicsData;
  using DerivedVelocityKinematicsData = typename Traits::VelocityKinematicsData;
  using DerivedAccelerationKinematicsData =
      typename Traits::AccelerationKinematicsData;
  using Base = MobodBase<T>;

  using Base::index;

  MobodImpl() = default;

  explicit MobodImpl(MobodInfo info) : Base(std::move(info)){};

  static constexpr int get_num_positions() { return num_positions; }
  static constexpr int get_num_velocities() { return num_velocities; }

  /* @name  mobod type-specific data and parameters accessors. In other words,
    these functions allow to retrieve/set data with a type that depends on the
    specific mobod's compile-time traits.
  */
  //@{
  const ConfigVector& get_mobod_positions(
      const Eigen::Ref<const VectorX<T>>& q) const {
    return *reinterpret_cast<const ConfigVector*>(q.data() +
                                                  this->first_position());
  }

  const VelocitiesVector& get_mobod_velocities(
      const Eigen::Ref<const VectorX<T>>& v) const {
    return *reinterpret_cast<const VelocitiesVector*>(v.data() +
                                                      this->first_velocity());
  }

  VelocitiesVector& get_mutable_mobod_velocities(EigenPtr<VectorX<T>> v) const {
    return *reinterpret_cast<VelocitiesVector*>(v->data() +
                                                this->first_velocity());
  }

  const DerivedPositionKinematicsData& get_mobod_data(
      const PositionKinematicsData<T>& data) const {
    return std::get<DerivedPositionKinematicsData>(data.mobod_data(index()));
  }

  DerivedPositionKinematicsData& get_mutable_mobod_data(
      PositionKinematicsData<T>* data) const {
    return std::get<DerivedPositionKinematicsData>(
        data->mutable_mobod_data(index()));
  }

  const DerivedVelocityKinematicsData& get_mobod_data(
      const VelocityKinematicsData<T>& data) const {
    return std::get<DerivedVelocityKinematicsData>(data.mobod_data(index()));
  }

  DerivedVelocityKinematicsData& get_mutable_mobod_data(
      VelocityKinematicsData<T>* data) const {
    return std::get<DerivedVelocityKinematicsData>(
        data->mutable_mobod_data(index()));
  }

  const DerivedAccelerationKinematicsData& get_mobod_data(
      const AccelerationKinematicsData<T>& data) const {
    return std::get<DerivedAccelerationKinematicsData>(
        data.mobod_data(index()));
  }

  DerivedAccelerationKinematicsData& get_mutable_mobod_data(
      AccelerationKinematicsData<T>* data) const {
    return std::get<DerivedAccelerationKinematicsData>(
        data->mutable_mobod_data(index()));
  }

  // @pre DerivedPositionKinematicsData::X_FM, of type MobodRigidTransform is
  // defined.
  const MobodRigidTransform& get_X_FM(
      const PositionKinematicsData<T>& data) const {
    return get_mobod_data(data).X_FM;
  }

  // Returns the spatial velocity V_FM of this mobod B measured and expressed in
  // the inboard frame F.
  const MobodSpatialMotion& get_V_FM(
      const VelocityKinematicsData<T>& data) const {
    return get_mobod_data(data).V_FM;
  }

  const MobodSpatialMotion& get_A_FM_M(
      const AccelerationKinematicsData<T>& data) const {
    return get_mobod_data(data).A_FM_M;
  }
  //@} // mobod type-specific data accessors.

  DerivedPositionKinematicsData MakePositionKinematicsData() const {
    return DerivedPositionKinematicsData{};
  }

  DerivedVelocityKinematicsData MakeVelocityKinematicsData() const {
    return DerivedVelocityKinematicsData{};
  }

  DerivedAccelerationKinematicsData MakeAccelerationKinematicsData() const {
    return DerivedAccelerationKinematicsData{};
  }

  void CalcPositionKinematicsData(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      PositionKinematicsData<T>* data) const {
    DRAKE_ASSERT(data != nullptr);
    DRAKE_DEMAND(index() > 0);  // Do not call on the world.
    DerivedPositionKinematicsData& mobod_data =
        this->get_mutable_mobod_data(data);
    const ConfigVector& q_mob = this->get_mobod_positions(q);
    derived().DoCalcPositionKinematicsData(q_mob, &mobod_data);

    const math::RigidTransform<T>& X_PF = this->get_X_PF(kernel_parameters);
    const MobodRigidTransform& X_FM = this->get_X_FM(*data);
    math::RigidTransform<T>& X_PM = this->get_mutable_X_PM(data);
    X_PM = X_PF * X_FM;
  }

  void CalcVelocityKinematicsData(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& v,
      const PositionKinematicsData<T>& position_kinematics,
      VelocityKinematicsData<T>* velocity_kinematics) const {
    DRAKE_DEMAND(index() > 0);  // Do not call on the world.

    // With the now updated mobod position kinematics, proceed to compute this
    // mobod's velocity kinematics.
    const DerivedPositionKinematicsData& mobod_position_kinematics =
        this->get_mobod_data(position_kinematics);
    DerivedVelocityKinematicsData& mobod_velocity_kinematics =
        this->get_mutable_mobod_data(velocity_kinematics);
    const ConfigVector& q_mob = this->get_mobod_positions(q);
    const VelocitiesVector& v_mob = this->get_mobod_velocities(v);
    derived().DoCalcVelocityKinematicsData(
        q_mob, v_mob, mobod_position_kinematics, &mobod_velocity_kinematics);

    // Recursive algorithm.
    // Per recursive precondition, X_PM, and V_WP_P were already computed.
    const math::RigidTransform<T>& X_PM = this->get_X_PM(position_kinematics);
    const SpatialVelocity<T>& V_WP_P = this->get_V_WP_P(*velocity_kinematics);
    const MobodSpatialMotion& V_FM = this->get_V_FM(*velocity_kinematics);
    SpatialVelocity<T>& V_WM_M = this->get_mutable_V_WM_M(velocity_kinematics);
    V_WM_M = X_PM.transpose() * V_WP_P + V_FM;  // = V_WPMo_M + V_FM_M
  }

  void CalcVelocityKinematicsData(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& v, PositionKinematicsData<T>* pk,
      VelocityKinematicsData<T>* vk) const {
    DRAKE_DEMAND(index() > 0);  // Do not call on the world.
    CalcPositionKinematicsData(kernel_parameters, q, pk);
    CalcVelocityKinematicsData(kernel_parameters, q, v, *pk, vk);
  }

  void CalcAccelerationKinematicsData(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& v,
      const Eigen::Ref<const VectorX<T>>& vdot, PositionKinematicsData<T>* pk,
      VelocityKinematicsData<T>* vk, AccelerationKinematicsData<T>* ak) const {
    DRAKE_DEMAND(index() > 0);  // Do not call on the world.

    // Update position and velocity kinematics for this mobod.
    CalcVelocityKinematicsData(kernel_parameters, q, v, pk, vk);

    CalcAccelerationKinematicsData(kernel_parameters, q, v, vdot, *pk, *vk, ak);
  }

  void CalcAccelerationKinematicsData(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& v,
      const Eigen::Ref<const VectorX<T>>& vdot,
      const PositionKinematicsData<T>& pk, const VelocityKinematicsData<T>& vk,
      AccelerationKinematicsData<T>* ak) const {
    DRAKE_DEMAND(index() > 0);  // Do not call on the world.

    const DerivedPositionKinematicsData& mobod_pk = this->get_mobod_data(pk);
    const DerivedVelocityKinematicsData& mobod_vk = this->get_mobod_data(vk);
    const math::RigidTransform<T>& X_PM = this->get_X_PM(pk);
    const SpatialVelocity<T>& V_WM_M = this->get_V_WM_M(vk);

    // Update per mobod acceleration kinematics data.
    DerivedAccelerationKinematicsData& mobod_ak =
        this->get_mutable_mobod_data(ak);
    const ConfigVector& q_mob = this->get_mobod_positions(q);
    const VelocitiesVector& v_mob = this->get_mobod_velocities(v);
    const VelocitiesVector& vdot_mob = this->get_mobod_velocities(vdot);
    derived().DoCalcAccelerationKinematicsData(q_mob, v_mob, vdot_mob, mobod_pk,
                                               mobod_vk, &mobod_ak);

    // Per recursive pre-condition, A_WP_P and V_WP_P are up to date.
    const SpatialVelocity<T>& V_WP_P = this->get_V_WP_P(vk);
    const SpatialAcceleration<T>& A_WP_P = this->get_A_WP_P(*ak);

    // Compute the component due to the motion of P. That is, P rigidly shifted
    // to M.
    const Vector3<T>& p_PM = X_PM.translation();
    const math::RotationMatrix<T> R_MP = X_PM.rotation().transpose();
    SpatialAcceleration<T>& A_WM_M = this->get_mutable_A_WM_M(ak);
    const Vector3<T>& w_WP_P = V_WP_P.rotational();
    A_WM_M = R_MP * A_WP_P.Shift(p_PM, w_WP_P);  // Rigid shift to M.

    // Add the acceleration bias Ac_M for this mobilizer.
    const MobodSpatialMotion& V_FM_M = mobod_vk.V_FM;
    const Vector3<T>& w_WM_M = V_WM_M.rotational();
    AccumulateAccelerationBias(w_WM_M, V_FM_M, &A_WM_M);

    // Add the across-mobilizer contribution, A_FM.
    const MobodSpatialMotion& A_FM_M = this->get_A_FM_M(*ak);
    A_WM_M = A_WM_M + A_FM_M;
  }

  void CalcInverseDynamics(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const PositionKinematicsData<T>& pk, const VelocityKinematicsData<T>& vk,
      const AccelerationKinematicsData<T>& ak,
      const std::vector<SpatialForce<T>>& applied_spatial_forces,
      const VectorX<T>& applied_generalized_forces,
      std::vector<SpatialForce<T>>* F_BMo_M_ptr, VectorX<T>* tau_ptr) const {
    std::vector<SpatialForce<T>>& mobod_spatial_forces = *F_BMo_M_ptr;
    VectorX<T>& mobod_generalized_forces = *tau_ptr;

    // Model parameters.
    const SpatialInertia<T>& M_BMo_M = kernel_parameters.M_BMo_M(index());

    // Kinematics.
    const SpatialVelocity<T>& V_WM_M = vk.V_WM_M(index());
    const Vector3<T>& w_WM_M = V_WM_M.rotational();
    const SpatialAcceleration<T>& A_WM_M = ak.A_WM_M(index());

    // Total force on B about Mo due to this mobod's motion.
    SpatialForce<T>& F_BMo_M = mobod_spatial_forces[index()];
    const SpatialForce<T>& Fapp_BMo_M = applied_spatial_forces[index()];
    const SpatialForce<T> b_Mo_M = CalcDynamicsBias(w_WM_M, M_BMo_M);
    F_BMo_M = M_BMo_M * A_WM_M + b_Mo_M - Fapp_BMo_M;

    // Contribution from children (outboard) mobilizers. Per recursive
    // pre-conditions, the spatial force on outboard mobilizers is already
    // computed.
    for (const MobodIndex child_index : this->outboards()) {
      // Spatial force at child mobod C about its inboard frame Mc, expressed in
      // Mc.
      const math::RigidTransform<T>& X_MMc = pk.X_PM(child_index);
      const SpatialForce<T>& F_CMc_Mc = mobod_spatial_forces[child_index];
      const SpatialForce<T> F_CMo_M = X_MMc * F_CMc_Mc;
      F_BMo_M += F_CMo_M;
    }

    const VelocitiesVector& applied_tau =
        this->get_mobod_velocities(applied_generalized_forces);
    VelocitiesVector& mobod_tau =
        this->get_mutable_mobod_velocities(&mobod_generalized_forces);

    // Project along the along the hinge matrix, i.e. mobod_tau = Hᵀ⋅F_BMo_M,
    // with H ≔ H_FM_M.
    ProjectSpatialForce(F_BMo_M, &mobod_tau);

    // Include applied generalized forces contribution.
    mobod_tau -= applied_tau;
  }

 protected:
  const DerivedMobod& derived() const {
    return *static_cast<const DerivedMobod*>(this);
  }

  // Computes the product tau = Hᵀ⋅F_BMo_M, with with H ≔ H_FM_M.
  // Specific mobilizers must implement DoProjectSpatialForce().
  void ProjectSpatialForce(const SpatialForce<T>& F_BMo_M,
                           VelocitiesVector* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    derived().DoProjectSpatialForce(F_BMo_M, tau);
  }

  // The acceleration A_WM_M of the mobilized frame M is computed recursively
  // and it entails three terms:
  //
  //   A_WM = A_WP.Shift(p_PM, w_WP) + A_FM + Ab
  //                  (1)              (2)   (3)
  //
  // see SpatialAcceleration::ComposeWithMovingFrameAcceleration() for a
  // derivation of these terms. This function computes the bias term (3),
  // expressed in frame M, and adds it to Ab_M.
  //
  // The first term is the spatial acceleration frame M would have if
  // moving rigidly attached to parent frame P. Therefore the rigid shift.
  // Notice that for accelerations this operation carries gyroscopic terms and
  // thus depends on the angular velocity w_WP of the parent frame.
  //
  // The second term is the simple contribution due to the spatial acceleration
  // of M in F (F is rigidly attached to P), the across mobilizer contribution.
  //
  // Finally, Ab is a term that carries gyroscopic contributions due to the
  // motion of M in the moving frame P.
  //
  // This particular decomposition in these three terms has computational
  // advantages. Term (1) involves generic spatial algebra, with a full
  // 6-dimensional acceleration A_WP. However, A_FM in (2) is a much smaller
  // object. For instance, it can be represented by a single scalar for revolute
  // and prismatic joints, with spatial operations that can be optimized for
  // this case. We take advantage of this fact in MobodSpatialMotion, a type
  // specialized for each derived mobod.
  //
  // Term (3) can be highly optimized as well, given that often it includes zero
  // or simple to compute terms for most mobilizer types. Expressed in frame M,
  // this term is:
  //  Ab_M = |    w_WM_M x w_FM_M |  = |                w_WM_M x w_FM_M |
  //         |2 * w_WP_M x v_FM_M |    | 2 * (w_WM_M - w_FM_M) x v_FM_M |
  // Notice this computation is greatly simplified when:
  //  1. Either v_FM_M or w_FM_M is zero.
  //  2. When v_FM_M and/or w_FM_M only have a single non-zero component.
  // As an example, for revolute joints, v_FM_M is zero and only one
  // component in v_FM_M is non-zero. Similarly for prismatic joints.
  //
  // Specific mobilizers will implement custom optimizations in
  // DoAccumulateAccelerationBias().
  void AccumulateAccelerationBias(const Vector3<T>& w_WM_M,
                                  const MobodSpatialMotion& V_FM_M,
                                  SpatialAcceleration<T>* Ab_M) const {
    derived().DoAccumulateAccelerationBias(w_WM_M, V_FM_M, Ab_M);
  }

  // The Newton-Euler equations for a rigid body, about an arbitrary point Mo,
  // and with derivatives in an inertial frame W (in this case the world frame),
  // read:
  //   F_Mo = M_Mo * A_WM + b_Mo
  // where b_Mo is the dynamics bias for these equations about Mo. This function
  // computes bias b_Mo_M, the dynamics bias about a point Mo, expressed in
  // frame m.
  SpatialForce<T> CalcDynamicsBias(const Vector3<T>& w_WM_M,
                                   const SpatialInertia<T>& M_BMo_M) const {
    const T& mass = M_BMo_M.get_mass();
    const Vector3<T>& p_MoBcm_M = M_BMo_M.get_com();
    const UnitInertia<T>& G_BMo_M = M_BMo_M.get_unit_inertia();

    // For a derivation, refer to Eq. 2.26 in [A. Jain, 2010].
    const Vector3<T> tau = mass * w_WM_M.cross(G_BMo_M * w_WM_M);
    const Vector3<T> f = mass * w_WM_M.cross(w_WM_M.cross(p_MoBcm_M));
    return SpatialForce<T>(tau, f);
  }

  /* `DerivedMobod` must provide an implementation for
     `DoCalcPositionKinematicsData()`.
     @throws std::exception if `DerivedMobod` does not provide an
     implementation for this method. */
  void DoCalcPositionKinematicsData(const ConfigVector& q,
                                    DerivedPositionKinematicsData* data) const {
    ThrowIfNotImplemented(__func__);
  }

  void DoCalcVelocityKinematicsData(
      const ConfigVector& q, const VelocitiesVector& v,
      const DerivedPositionKinematicsData& position_kinematics,
      DerivedVelocityKinematicsData* velocity_kinematics) const {
    ThrowIfNotImplemented(__func__);
  }

  void DoAccumulateAccelerationBias(const Vector3<T>& w_WM_M,
                                    const MobodSpatialMotion& V_FM_M,
                                    SpatialAcceleration<T>* Ab_M) const {
    ThrowIfNotImplemented(__func__);
  }

 private:
  friend class MultibodyKernel<T>;

  /* Helper to throw a descriptive exception when a given "virtual" (through
   CRTP) function is not implemented by a derived class. */
  void ThrowIfNotImplemented(const char* source_method) const {
    throw std::runtime_error("The DerivedMobod from " +
                             NiceTypeName::Get(*this) +
                             " must provide an implementation for " +
                             std::string(source_method) + "().");
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
