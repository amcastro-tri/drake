#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Imposes the constraint g = qᵢ − r⋅qⱼ that couples the i-th and j-th DOFs. r
// is the gear ratio, which can also be negative.
// The compliant constraint forces obeys the relationship: γ=−k⋅g−c⋅ġ with
// linear dissipation model constant c = τ⋅k.
template <typename T>
class SapDistanceConstraint final : public SapConstraint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapDistanceConstraint);

  // TODO: Simplify, get rid of struct Parameters.
  struct Parameters {
    T distance{0.0};  // It must be non-negative.
    T stiffness{0.0};
    T dissipation_time_scale{0.0};
    // Rigid approximation constant: Rₙ = β²/(4π²)⋅w when the contact frequency
    // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. w
    // corresponds to a diagonal approximation of the Delassuss operator for
    // each contact. See [Castro et al., 2021. §IX.A] for details.
    double beta{1.0};
  };

  // @param d0 Current distance.
  // @param J Current distance constraint Jacobian.
  // @throws if the number of rows in J is different from three.
  SapDistanceConstraint(const Parameters& p, int clique, const MatrixX<T>& J,
                        const T& d0);

  // @throws if the number of rows in J0 and J1 is different from three.
  SapDistanceConstraint(const Parameters& p, int clique0, int clique1,
                        const MatrixX<T>& J0, const MatrixX<T>& J1,
                        const T& d0);

  void Project(const Eigen::Ref<const VectorX<T>>& y,
               const Eigen::Ref<const VectorX<T>>& R,
               EigenPtr<VectorX<T>> gamma,
               MatrixX<T>* dPdy = nullptr) const final;

  VectorX<T> CalcBiasTerm(const T& time_step, const T& wi) const final;
  VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                        const T& wi) const final;

 private:
  Parameters parameters_;
  T d0_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
