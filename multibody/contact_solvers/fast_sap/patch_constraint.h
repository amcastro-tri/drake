#pragma once

#include <numeric>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/fast_sap/patch_constraint_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

// A bunch of std::vector with the actual memory storage for all params in a
// SapModel
template <typename T>
class PatchConstraint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PatchConstraint);

  explicit PatchConstraint(int index) : patch_index_(index) {}

  void AddPair(const T& fn0, const T& stiffness, const Matrix3<T>& R_WC,
               const Vector3<T>& p_AoC_W, const Vector3<T>& p_BoC_W,
               PatchConstraintParamsPool<T>* pool) const {
    int& pair = pool->num_pairs()[patch_index_];
    pool->fn0(pair) = fn0;
    pool->stiffness(pair) = stiffness;
    pool->R_WC(pair) = R_WC;
    pool->p_AoC_W(pair) = p_AoC_W;
    pool->p_BoC_W(pair) = p_BoC_W;
    ++pair;
  }

 private:
  int patch_index_{0};
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
