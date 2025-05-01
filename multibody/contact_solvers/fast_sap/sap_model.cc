#include "drake/multibody/contact_solvers/fast_sap/sap_model.h"

#include <memory>
#include <set>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

template <typename T>
using MatrixXView = typename EigenPool<MatrixX<T>>::ElementView;
template <typename T>
using ConstMatrixXView = typename EigenPool<MatrixX<T>>::ConstElementView;

template <typename T>
Eigen::VectorBlock<const VectorX<T>> PooledSapModel<T>::clique_segment(
    int clique, const VectorX<T>& x) const {
  DRAKE_ASSERT(x.size() == num_velocities());
  return x.segment(clique_start_[clique], clique_sizes_[clique]);
}

template <typename T>
Eigen::VectorBlock<VectorX<T>> PooledSapModel<T>::clique_segment(
    int clique, VectorX<T>* x) const {
  DRAKE_ASSERT(x != nullptr);
  DRAKE_ASSERT(x->size() == num_velocities());
  return x->segment(clique_start_[clique], clique_sizes_[clique]);
}

/* A⋅v = r + Jᵀ⋅γ,
 which corresponds to the convex cost:
    ℓ(v) = 1/2‖v‖²−r⋅v + ℓ(vc).
*/

/* Computes:
  - Av
  - momentum_cost */
template <typename T>
void PooledSapModel<T>::CalcMomentumTerms(const SapData<T>& data,
                                    typename SapData<T>::Cache* cache) const {
  const VectorX<T>& v = data.v();
  VectorX<T>& Av = cache->Av;

  // Scratch data.
  VectorX<T>& tmp = data.scratch().v1;

  Av.setZero();
  for (int c = 0; c < A_.size(); ++c) {
    ConstMatrixXView<T> A_clique = A_[c];
    const auto v_clique = clique_segment(c, v);
    clique_segment(c, &Av) = A_clique * v_clique;
  }

  // Cost.
  tmp = 0.5 * Av - r_;
  cache->momentum_cost = v.dot(tmp);

  // Gradient.
  cache->cost_gradient = Av - r_;

  // TODO. Hessian.
}

template <typename T>
void PooledSapModel<T>::CalcData(const VectorX<T>& v, SapData<T>* data) const {
  data->v() = v;
  typename SapData<T>::Cache& cache = data->cache();
  CalcMomentumTerms(*data, &cache);
  cache.cost = cache.momentum_cost + cache.constraints_cost;
}

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::fast_sap::PooledSapModel<double>;
