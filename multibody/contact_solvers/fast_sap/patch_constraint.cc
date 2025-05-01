#include "drake/multibody/contact_solvers/fast_sap/patch_constraint_data.h"

#include <numeric>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

template <typename T>
int PatchConstraintParamsPool<T>::Add(std::array<int, 2> cliques,
                                      std::array<int, 2> clique_nv,
                                      const std::vector<Vector3<T>>& p_WC,
                                      const std::vector<Matrix3<T>>& R_WC,
                                      std::vector<T> stiffness) {
    const int num_pairs =  ssize(p_WC);                                       
    num_pairs_ += num_pairs;

    p_WC_.PushBack(p_WC);
    R_WC.PushBack(R_WC);
    stiffness_.insert(stiffness_.end(), stiffness.begin(), stiffness.end());

    return num_patches_++;
}

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
