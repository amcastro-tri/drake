#include "drake/multibody/kernel/multibody_kernel.h"

#include <iostream>

#include "drake/common/default_scalars.h"
#include "drake/common/fmt_eigen.h"

using drake::fmt_eigen;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
PositionKinematicsData<T> MultibodyKernel<T>::MakePositionKinematicsData()
    const {
  std::vector<MobodPositionKinematicsDataVariant<T>> mobod_data;
  mobod_data.reserve(num_mobods());
  for (const auto& mobod : mobods_) {
    std::visit(
        [&](auto&& derived) {
          MobodPositionKinematicsDataVariant<T> mobod_kin =
              derived.MakePositionKinematicsData();
          mobod_data.push_back(std::move(mobod_kin));
        },
        mobod);
  }
  return PositionKinematicsData<T>(std::move(mobod_data));
}

template <typename T>
VelocityKinematicsData<T> MultibodyKernel<T>::MakeVelocityKinematicsData()
    const {
  std::vector<MobodVelocityKinematicsDataVariant<T>> mobod_data;
  mobod_data.reserve(num_mobods());
  for (const auto& mobod : mobods_) {
    std::visit(
        [&](auto&& derived) {
          MobodVelocityKinematicsDataVariant<T> mobod_kin =
              derived.MakeVelocityKinematicsData();
          mobod_data.push_back(std::move(mobod_kin));
        },
        mobod);
  }
  return VelocityKinematicsData<T>(std::move(mobod_data));
}

template <typename T>
AccelerationKinematicsData<T>
MultibodyKernel<T>::MakeAccelerationKinematicsData() const {
  std::vector<MobodAccelerationKinematicsDataVariant<T>> mobod_data;
  mobod_data.reserve(num_mobods());
  for (const auto& mobod : mobods_) {
    std::visit(
        [&](auto&& derived) {
          MobodAccelerationKinematicsDataVariant<T> mobod_kin =
              derived.MakeAccelerationKinematicsData();
          mobod_data.push_back(std::move(mobod_kin));
        },
        mobod);
  }
  return AccelerationKinematicsData<T>(std::move(mobod_data));
}

template <typename T>
void MultibodyKernel<T>::CalcPositionKinematicsData(
    const MultibodyKernelParameters<T>& kernel_parameters,
    const Eigen::Ref<const VectorX<T>>& q,
    PositionKinematicsData<T>* data) const {
  for (int i = 1; i < num_mobods(); ++i) {  // skip the world.
    const auto& mobod = mobods_[i];
    std::visit(
        [&](auto&& derived) {
          derived.CalcPositionKinematicsData(kernel_parameters, q, data);
        },
        mobod);
  }
}

template <typename T>
void MultibodyKernel<T>::CalcVelocityKinematicsData(
    const MultibodyKernelParameters<T>& kernel_parameters,
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& v, PositionKinematicsData<T>* pk,
    VelocityKinematicsData<T>* vk) const {
  for (int i = 1; i < num_mobods(); ++i) {
    const auto& mobod = mobods_[i];
    std::visit(
        [&](auto&& derived) {
          derived.CalcVelocityKinematicsData(kernel_parameters, q, v, pk, vk);
        },
        mobod);
  }
}

template <typename T>
void MultibodyKernel<T>::CalcVelocityKinematicsData(
    const MultibodyKernelParameters<T>& kernel_parameters,
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& v, const PositionKinematicsData<T>& pk,
    VelocityKinematicsData<T>* vk) const {
  for (int i = 1; i < num_mobods(); ++i) {
    const auto& mobod = mobods_[i];
    std::visit(
        [&](auto&& derived) {
          derived.CalcVelocityKinematicsData(kernel_parameters, q, v, pk, vk);
        },
        mobod);
  }
}

template <typename T>
void MultibodyKernel<T>::CalcAccelerationKinematicsData(
    const MultibodyKernelParameters<T>& kernel_parameters,
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& v,
    const Eigen::Ref<const VectorX<T>>& vdot, PositionKinematicsData<T>* pk,
    VelocityKinematicsData<T>* vk, AccelerationKinematicsData<T>* ak) const {
  DRAKE_THROW_UNLESS(q.size() == num_positions());
  DRAKE_THROW_UNLESS(v.size() == num_velocities());
  DRAKE_THROW_UNLESS(pk != nullptr);
  DRAKE_THROW_UNLESS(vk != nullptr);
  DRAKE_THROW_UNLESS(ak != nullptr);
  ak->mutable_A_WM_M(0).SetZero();  // The world's acceleration.
  for (int i = 1; i < num_mobods(); ++i) {
    const auto& mobod = mobods_[i];
    std::visit(
        [&](auto&& derived) {
          derived.CalcAccelerationKinematicsData(kernel_parameters, q, v, vdot,
                                                 pk, vk, ak);
        },
        mobod);
  }
}

template <typename T>
void MultibodyKernel<T>::CalcAccelerationKinematicsData(
    const MultibodyKernelParameters<T>& kernel_parameters,
    const Eigen::Ref<const VectorX<T>>& q,
    const Eigen::Ref<const VectorX<T>>& v,
    const Eigen::Ref<const VectorX<T>>& vdot,
    const PositionKinematicsData<T>& pk, const VelocityKinematicsData<T>& vk,
    AccelerationKinematicsData<T>* ak) const {
  DRAKE_THROW_UNLESS(q.size() == num_positions());
  DRAKE_THROW_UNLESS(v.size() == num_velocities());
  DRAKE_THROW_UNLESS(ak != nullptr);
  ak->mutable_A_WM_M(0).SetZero();  // The world's acceleration.
  for (int i = 1; i < num_mobods(); ++i) {
    const auto& mobod = mobods_[i];
    std::visit(
        [&](auto&& derived) {
          derived.CalcAccelerationKinematicsData(kernel_parameters, q, v, vdot,
                                                 pk, vk, ak);
        },
        mobod);
  }
}

template <typename T>
void MultibodyKernel<T>::CalcInverseDynamics(
    const MultibodyKernelParameters<T>& kernel_parameters,
    const PositionKinematicsData<T>& pk, const VelocityKinematicsData<T>& vk,
    const AccelerationKinematicsData<T>& ak,
    const std::vector<SpatialForce<T>>& applied_spatial_forces,
    const VectorX<T>& applied_generalized_forces,
    std::vector<SpatialForce<T>>* mobod_spatial_forces,
    VectorX<T>* mobod_generalized_forces) const {
  DRAKE_THROW_UNLESS(mobod_spatial_forces != nullptr);
  DRAKE_THROW_UNLESS(ssize(*mobod_spatial_forces) == num_mobods());
  DRAKE_THROW_UNLESS(mobod_generalized_forces != nullptr);
  DRAKE_THROW_UNLESS(mobod_generalized_forces->size() == num_velocities());

  // Tip-to-base recursion.
  for (int i = num_mobods() - 1; i >= 1; --i) {
    const auto& mobod = mobods_[i];
    std::visit(
        [&](auto&& derived) {
          derived.CalcInverseDynamics(
              kernel_parameters, pk, vk, ak, applied_spatial_forces,
              applied_generalized_forces, mobod_spatial_forces,
              mobod_generalized_forces);
        },
        mobod);
  }
}

template <typename T>
void MultibodyKernel<T>::CalcMassMatrixViaInverseDynamics(
    const MultibodyKernelParameters<T>& parameters,
    const Eigen::Ref<const VectorX<T>>& q, const PositionKinematicsData<T>& pk,
    MatrixX<T>* M_ptr) const {
  DRAKE_THROW_UNLESS(M_ptr != nullptr);
  const int nv = num_velocities();

  // Workspace.
  // TODO(amcastro-tri): Make this workspace a struct argument.
  VelocityKinematicsData<T> vk = MakeVelocityKinematicsData();
  AccelerationKinematicsData<T> ak = MakeAccelerationKinematicsData();
  VectorX<T> Mcol(nv);
  const VectorX<T> zero_v = VectorX<T>::Zero(nv);
  const std::vector<SpatialForce<T>> zero_spatial_forces = [this]() {
    std::vector<SpatialForce<T>> spatial_forces(num_mobods());
    for (auto& F : spatial_forces) F.SetZero();
    return spatial_forces;
  }();
  std::vector<SpatialForce<T>> mobod_spatial_forces(num_mobods());
  VectorX<T> ei = VectorX<T>::Zero(nv);

  CalcVelocityKinematicsData(parameters, q, zero_v, pk, &vk);

  // Compute one column of M at a time using inverse dynamics.
  MatrixX<T>& M = *M_ptr;
  M = MatrixX<T>::Zero(nv, nv);
  for (int i = 0; i < nv; ++i) {
    ei(i) = 1.0;

    CalcAccelerationKinematicsData(parameters, q, zero_v, ei, pk, vk, &ak);

    // TODO(amcastro-tri): optimize ID for the case v = 0.
    CalcInverseDynamics(parameters, pk, vk, ak, zero_spatial_forces, zero_v,
                        &mobod_spatial_forces, &Mcol);

    M.col(i) = Mcol;

    ei(i) = 0.0;
  }
}

template <typename T>
void MultibodyKernel<T>::CalcBodyPosesInWorld(
    const MultibodyKernelParameters<T>& kernel_parameters,
    const PositionKinematicsData<T>& pk,
    std::vector<RigidTransform<T>>* X_WB_ptr) const {
  DRAKE_DEMAND(X_WB_ptr != nullptr);
  auto& X_WB = *X_WB_ptr;
  DRAKE_DEMAND(ssize(X_WB) == num_mobods());
  X_WB[0] = RigidTransform<T>::Identity();
  for (int i = 1; i < num_mobods(); ++i) {
    const auto& mobod = mobods_[i];
    const int inboard_index = std::visit(
        [&](auto&& derived) {
          return derived.inboard_index();
        },
        mobod);

    // By recursive pre-condition, the parent pose was already computed.
    const RigidTransform<T>& X_WP = X_WB[inboard_index];
    const RigidTransform<T>& X_PM = pk.X_PM(i);
    const RigidTransform<T>& X_MB = kernel_parameters.X_MB(i);
    X_WB[i] = X_WP * X_PM * X_MB;
  }
}

template <typename T>
void MultibodyKernel<T>::CalcBodySpatialVelocitiesInWorld(
    const MultibodyKernelParameters<T>& kernel_parameters,
    const std::vector<RigidTransform<T>>& X_WB,
    const VelocityKinematicsData<T>& vk,
    std::vector<SpatialVelocity<T>>* V_WB_ptr) const {
  DRAKE_DEMAND(V_WB_ptr != nullptr);
  auto& V_WB = *V_WB_ptr;
  DRAKE_DEMAND(ssize(V_WB) == num_mobods());
  V_WB[0].SetZero();  // the world's spatial velocity.
  for (int i = 1; i < num_mobods(); ++i) {
    // No need to compute recursively, we only need to shift to B and re-express
    // in W. The recursive computation was already performed in V_WM_M.
    const SpatialVelocity<T>& V_WM_M = vk.V_WM_M(i);
    const RigidTransform<T>& X_MB = kernel_parameters.X_MB(i);
    const RigidTransform<T> X_BM = X_MB.inverse();
    const SpatialVelocity<T> V_WB_B = X_BM * V_WM_M;

    const RotationMatrix<T>& R_WB = X_WB[i].rotation();
    V_WB[i] = R_WB * V_WB_B;
  }
}

template <typename T>
void MultibodyKernel<T>::CalcBodySpatialAccelerationsInWorld(
    const MultibodyKernelParameters<T>& kernel_parameters,
    const std::vector<drake::math::RigidTransform<T>>& X_WB,
    const VelocityKinematicsData<T>& vk,
    const AccelerationKinematicsData<T>& ak,
    std::vector<SpatialAcceleration<T>>* A_WB_ptr) const {
  DRAKE_DEMAND(A_WB_ptr != nullptr);
  auto& A_WB = *A_WB_ptr;
  DRAKE_DEMAND(ssize(A_WB) == num_mobods());
  A_WB[0].SetZero();  // the world's spatial acceleration.
  for (int i = 1; i < num_mobods(); ++i) {
    // No need to compute recursively, we only need to shift M to B and
    // re-express in W. The recursive computation was already performed in
    // A_WM_M.
    const RigidTransform<T>& X_MB = kernel_parameters.X_MB(i);
    const Vector3<T>& p_MB_M = X_MB.translation();
    const RotationMatrix<T> R_BM = X_MB.rotation().transpose();
    const RotationMatrix<T>& R_WB = X_WB[i].rotation();
    const SpatialVelocity<T>& V_WM_M = vk.V_WM_M(i);
    const Vector3<T>& w_WM_M = V_WM_M.rotational();
    const SpatialAcceleration<T>& A_WM_M = ak.A_WM_M(i);

    const SpatialAcceleration<T> A_WB_B = R_BM * A_WM_M.Shift(p_MB_M, w_WM_M);
    A_WB[i] = R_WB * A_WB_B;
  }
}

template <typename T>
void MultibodyKernel<T>::CalcBodySpatialForcesInWorld(
    const MultibodyKernelParameters<T>& kernel_parameters,
    const std::vector<drake::math::RigidTransform<T>>& X_WB,
    const std::vector<SpatialForce<T>>& F_BMo_M,
    std::vector<SpatialForce<T>>* F_BBo_W_ptr) const {
  DRAKE_DEMAND(F_BBo_W_ptr != nullptr);
  auto& F_BBo_W = *F_BBo_W_ptr;
  DRAKE_DEMAND(ssize(F_BBo_W) == num_mobods());
  for (int i = 0; i < num_mobods(); ++i) {
    const RigidTransform<T>& X_MB = kernel_parameters.X_MB(i);
    const Vector3<T>& p_MB_M = X_MB.translation();
    const RotationMatrix<T> R_BM = X_MB.rotation().transpose();
    const RotationMatrix<T>& R_WB = X_WB[i].rotation();

    const SpatialForce<T> F_BBo_B = R_BM * F_BMo_M[i].Shift(p_MB_M);
    F_BBo_W[i] = R_WB * F_BBo_B;
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::MultibodyKernel)
