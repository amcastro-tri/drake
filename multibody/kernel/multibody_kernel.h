#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/value.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/kernel/acceleration_kinematics_data.h"
#include "drake/multibody/kernel/mobod_variant.h"
#include "drake/multibody/kernel/multibody_kernel_parameters.h"
#include "drake/multibody/kernel/position_kinematics_data.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
class MultibodyKernel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyKernel);

  // Creates an "empty" kernel with a single mobod for the "world", even though
  // this "world" body has no mobilizer (or we could think of this "world" body
  // "welded" to the "universe".)
  // Therefore num_mobods() equals to one (1) after construction.
  MultibodyKernel() {
    // MobodInfo world_info(0 /* index */, -1 /* (invalid) parent */,
    //                      0 /* q_start */, 0 /* q_start */);
    mobods_.push_back(MobodVariant<T>{WorldMobod<T>()});
    //{std::in_place_type<WorldMobod<T>>});
  }

  // Takes ownership of `mobod` and updates its index to the value of
  // num_mobods() before making this call.
  // @returns the new mobod index.
  template <class MobodType>
  int AddMobod(MobodInfo info,
               MobodParameters<typename MobodType::Scalar> mobod_parameters,
               MultibodyKernelParameters<typename MobodType::Scalar>*
                   kernel_parameters) {
    DRAKE_DEMAND(num_mobods() == info.index);
    this->num_positions_ += MobodType::num_positions;
    this->num_velocities_ += MobodType::num_velocities;
    const int mobod_index = num_mobods();
    MobodVariant<T> mobod = MobodType(std::move(info));
    mobods_.push_back(std::move(mobod));
    kernel_parameters->AddParameters(mobod_index, std::move(mobod_parameters));
    return mobod_index;
  }

  int num_mobods() const { return ssize(mobods_); }
  int num_positions() const { return num_positions_; }
  int num_velocities() const { return num_velocities_; }

  // Factory methods to make data for different computations.
  PositionKinematicsData<T> MakePositionKinematicsData() const;
  VelocityKinematicsData<T> MakeVelocityKinematicsData() const;
  AccelerationKinematicsData<T> MakeAccelerationKinematicsData() const;

  // Updates position kinematics data as a function of configurations q.
  void CalcPositionKinematicsData(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      PositionKinematicsData<T>* data) const;

  // Updates velocity kinematics data as a function of configurations q and
  // generalized velocities v. Position kinematics are also computed.
  void CalcVelocityKinematicsData(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& v,
      PositionKinematicsData<T>* position_kinemaics,
      VelocityKinematicsData<T>* velocity_kinematics) const;

  // Updates velocity kinematics data as a function of configurations q and
  // generalized velocities v. Previously computed position kinematics at q are
  // reused.
  void CalcVelocityKinematicsData(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& v,
      const PositionKinematicsData<T>& pk, VelocityKinematicsData<T>* vk) const;

  // Updates acceleration kinematics as a function of state an time derivatives
  // of the generalized velocities, vdot. Postion and velocity kinematics are
  // also computed.
  void CalcAccelerationKinematicsData(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& v,
      const Eigen::Ref<const VectorX<T>>& vdot, PositionKinematicsData<T>* pk,
      VelocityKinematicsData<T>* vk, AccelerationKinematicsData<T>* ak) const;

  // Updates acceleration kinematics as a function of state an time derivatives
  // of the generalized velocities, vdot. Position and velocity kinematics
  // already at {q, v} are reused.
  void CalcAccelerationKinematicsData(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      const Eigen::Ref<const VectorX<T>>& v,
      const Eigen::Ref<const VectorX<T>>& vdot,
      const PositionKinematicsData<T>& pk, const VelocityKinematicsData<T>& vk,
      AccelerationKinematicsData<T>* ak) const;

  // Performs an inverse dynamics computation using RNEA. The result of inverse
  // dynamics are the generalized forces that would need to be applied to
  // achieve the accelerations stored in the acceleration kinematics ak
  // (function of vdot, see CalcAccelerationKinematicsData()). Mathematically:
  //   ID(v̇, q, v, {τₐₚₚ, Fₐₚₚ}) = M⋅v̇ + C(q, v) − τₐₚₚ − Jᵀ⋅Fₐₚₚ
  //
  // @param applied_spatial_forces Applied spatial forces, of size num_mobods().
  //   For each mobod, it must store the spatial force F_BMo_M on mobilized body
  //   B, about its inboard frame Mo, expressed in the inboard frame M.
  // @param applied_generalized_forces Applied generalized forces, of size
  // num_velocities().
  // @param mobod_spatial_forces On output, the reaction forces on mobilized
  // frame M, about Mo and expressed in M.
  // @param mobod_generalized_forces The generalized force that must be applied
  // in order to achieve the input accelerations.
  //
  // @warning For free floating bodies applied_generalized_forces will store the
  // force F_BMo_M about Mo and expressed in M. Probably best to use
  // applied_spatial_forces to apply spatial forces on free bodies to avoid
  // confusion with frames.
  void CalcInverseDynamics(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const PositionKinematicsData<T>& pk, const VelocityKinematicsData<T>& vk,
      const AccelerationKinematicsData<T>& ak,
      const std::vector<SpatialForce<T>>& applied_spatial_forces,
      const VectorX<T>& applied_generalized_forces,
      std::vector<SpatialForce<T>>* mobod_spatial_forces,
      VectorX<T>* mobod_generalized_forces) const;

  // Computes the mass matrix of the model as a function of configurations q.
  // Position kinematics are reused and must be already updated to q.
  void CalcMassMatrixViaInverseDynamics(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const Eigen::Ref<const VectorX<T>>& q,
      const PositionKinematicsData<T>& pk, MatrixX<T>* M) const;

  // @group Helper Functions
  // Recall that for performance, most kernel computations are performed in the
  // body's inboard (mobilized) frame M. However, it is most often useful to
  // obtain quantities in the world frame. Therefore, this functions help to
  // transform spatial quantities to the world frame.
  // @{
  void CalcBodyPosesInWorld(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const PositionKinematicsData<T>& pk,
      std::vector<drake::math::RigidTransform<T>>* X_WB) const;

  void CalcBodySpatialVelocitiesInWorld(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const std::vector<drake::math::RigidTransform<T>>& X_WB,
      const VelocityKinematicsData<T>& vk,
      std::vector<SpatialVelocity<T>>* V_WB) const;

  void CalcBodySpatialAccelerationsInWorld(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const std::vector<drake::math::RigidTransform<T>>& X_WB,
      const VelocityKinematicsData<T>& vk,
      const AccelerationKinematicsData<T>& ak,
      std::vector<SpatialAcceleration<T>>* A_WB) const;

  void CalcBodySpatialForcesInWorld(
      const MultibodyKernelParameters<T>& kernel_parameters,
      const std::vector<drake::math::RigidTransform<T>>& X_WB,
      const std::vector<SpatialForce<T>>& F_BMo_M,
      std::vector<SpatialForce<T>>* F_BBo_W) const;
  //@}        

 private:
  // Consider grabbing these straight from the spanning forest class at
  // construction.
  int num_positions_{0};
  int num_velocities_{0};
  std::vector<MobodVariant<T>> mobods_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
