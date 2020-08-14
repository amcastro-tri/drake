#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace solvers {

template <typename T>
class DenseSystemDynamicsData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DenseSystemDynamicsData)

  DenseSystemDynamicsData(const MatrixX<T>* M, const VectorX<T>* v0,
                          const VectorX<T>* tau, const MatrixX<T>* Jc)
      : M_(M), Jc_(Jc), tau_(tau), v0_(v0) {
    // TODO: add demands for non-nullptr and for consistent sizes.
    nv_ = M_->rows();
    nc_ = Jc->rows() / 3;  // TODO: verify multiple of 3.
  }

  int num_contacts() const { return nc_; }
  int num_velocities() const { return nv_; }

  const MatrixX<T>& M() const { return *M_; };
  const MatrixX<T>& get_Jc() const { return *Jc_; };
  const VectorX<T>& get_v0() const { return *v0_; };
  const VectorX<T>& get_tau() const { return *tau_; }

 private:
  // sizes.
  int nc_;
  int nv_;

  // TODO: use a general matrix representation that enables the use of operator
  // forms.
  /// System's mass matrix.
  const MatrixX<T>* M_;
  const MatrixX<T>* Jc_;

  // Right hand side.
  const VectorX<T>* tau_;
  const VectorX<T>* v0_;
};

}  // namespace solvers
}  // namespace multibody
}  // namespace drake