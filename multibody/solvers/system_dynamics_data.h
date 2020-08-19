#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/solvers/linear_operator.h"

#include <string>

namespace drake {
namespace multibody {
namespace solvers {  

template <typename T>
class SystemDynamicsData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemDynamicsData)

  SystemDynamicsData(const LinearOperator<T>* Minv,
                     const LinearOperator<T>* Jc, 
                     const LinearOperator<T>* JcT, 
                     const VectorX<T>* v0,
                     const VectorX<T>* tau)
      : Minv_(Minv), Jc_(Jc), JcT_(JcT), v0_(v0), tau_(tau) {
    DRAKE_DEMAND(Minv != nullptr);
    DRAKE_DEMAND(Jc != nullptr);
    DRAKE_DEMAND(JcT != nullptr);
    DRAKE_DEMAND(v0 != nullptr);
    DRAKE_DEMAND(tau != nullptr);
    DRAKE_DEMAND(Minv->rows() == Minv->cols());
    DRAKE_DEMAND(Jc->rows() % 3 == 0);
    DRAKE_DEMAND(Jc->cols() == Minv->rows());
    DRAKE_DEMAND(JcT->rows() == Jc->cols());
    DRAKE_DEMAND(JcT->cols() == Jc->rows());
    nv_ = Minv->rows();
    nc_ = Jc->rows() / 3;
  }

  int num_contacts() const { return nc_; }
  int num_velocities() const { return nv_; }

  const LinearOperator<T>& get_Minv() const { return *Minv_; };
  const LinearOperator<T>& get_Jc() const { return *Jc_; };
  const LinearOperator<T>& get_JcT() const { return *JcT_; };
  const VectorX<T>& get_v0() const { return *v0_; };
  const VectorX<T>& get_tau() const { return *tau_; }

 private:
  // sizes.
  int nc_;
  int nv_;

  const LinearOperator<T>* Minv_{nullptr};
  const LinearOperator<T>* Jc_{nullptr};
  const LinearOperator<T>* JcT_{nullptr};

  // Right hand side.
  const VectorX<T>* v0_{nullptr};
  const VectorX<T>* tau_{nullptr};
};

}  // namespace solvers
}  // namespace multibody
}  // namespace drake