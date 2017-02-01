#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"

namespace drake {
namespace multibody {

template <typename T>
class MobilizerPositionKinematics {
 public:
  MobilizerPositionKinematics(
      Isometry3<T>* X_FM, T* H_FM) :
      X_FM_(X_FM), H_FM_(H_FM) {}

  const Isometry3<T>& get_X_FM() const {
    DRAKE_ASSERT(X_FM_ != nullptr);
    return *X_FM_;
  }

  Isometry3<T>& get_mutable_X_FM() {
    DRAKE_ASSERT(X_FM_ != nullptr);
    return *X_FM_;
  }

  template <int nv>
  SpatialVelocityJacobian<T, nv>& get_mutable_H_FM() {
    DRAKE_ASSERT(H_FM_ != nullptr);
    return *reinterpret_cast<SpatialVelocityJacobian<T, nv>*>(H_FM_);
  }

 private:
  Isometry3<T>* X_FM_;
  T* H_FM_;  // Pointer into the first column of H_FM(q)
  //SpatialVector<T>* HdotTimesV_FM_;
};

template <typename T>
class MobilizerContext {
 public:
  MobilizerContext(int num_positions, int num_velocities,
                   T* q, T* v,
                   const MobilizerPositionKinematics<T>& position_kinematics) :
      num_positions_(num_positions), num_velocities_(num_velocities),
      q_(q), v_(v),
      position_kinematics_(position_kinematics) {
    DRAKE_ASSERT(q != nullptr);
  }

  template <int nq>
  const Vector<T, nq>& get_positions() const {
    DRAKE_ASSERT(num_positions_ == nq);
    return *reinterpret_cast<const Vector<T, nq>*>(q_);
  }

  template <int nq>
  Vector<T, nq>& get_mutable_positions() {
    DRAKE_ASSERT(num_positions_ == nq);
    return *reinterpret_cast<Vector<T, nq>*>(q_);
  }

  template <int nv>
  Vector<T, nv>& get_mutable_velocities() {
    DRAKE_ASSERT(num_velocities_ == nv);
    return *reinterpret_cast<Vector<T, nv>*>(v_);
  }

  MobilizerPositionKinematics<T>* get_mutable_position_kinematics() const {
    return &position_kinematics_;
  }

 private:
  // These entries are used for sanity checks on debug builds.
  int num_positions_, num_velocities_;

  T* q_;  // Pointer into the global array of positions.
  T* v_;  // Pointer into the global array of velocities.

  // Cached entries.
  // MobilizerPositionKinematics provides:
  //   - Fast access from Mobilizer's.
  //   - Encapsulation. Only local variables to a mobilizer are accessible to
  //     them.
  mutable MobilizerPositionKinematics<T> position_kinematics_;
};

}  // namespace multibody
}  // namespace drake
