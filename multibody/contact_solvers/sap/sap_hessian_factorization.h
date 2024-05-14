#pragma once

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/conex_supernodal_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// The type of the factorization.
enum class SapHessianFactorizationType {
  // Supernodal KKT solver as implemented in conex [Permenter, 2020].
  //
  // [Permenter, 2020] Permenter, Frank. "A geodesic interior-point method for
  // linear optimization over symmetric cones." SIAM Journal on
  // Optimization 33.2 (2023): 1006-1034.
  kConex,
  // Block sparse supernodal solver implemented by BlockSparseCholeskySolver.
  kBlockSparseCholesky,
  // Dense algebra. Typically used for testing.
  kDense,
  // No solver has been specified yet.
  kNotSpecified,
};


// This class essentially stores a factorization for the SAP solver and
// implements methods to compute and work with this factorization. Since
// factorizations are expensive to compute, only computations with T = double
// are allowed.
class SapHessianFactorization {
 public:
  /* We allow move semantics and forbid copy semantics. */
  SapHessianFactorization(const SapHessianFactorization&) = delete;
  SapHessianFactorization& operator=(const SapHessianFactorization&) = delete;
  SapHessianFactorization(SapHessianFactorization&&) = default;
  SapHessianFactorization& operator=(SapHessianFactorization&&) = default;

  SapHessianFactorization() = default;

  // Constructor for a SAP factorization where the Hessian as the form H = A +
  // Jᵀ⋅G⋅J, see [Castro et al. 2022]. Matrix G is set with the call  to
  // Update().
  // This method holds references to A and J and therefore they must outlive an
  // instance of this class.
  // @pre A and J must not be nullptr.
  SapHessianFactorization(SapHessianFactorizationType type,
                          const std::vector<MatrixX<double>>* A,
                          const BlockSparseMatrix<double>* J);

  const std::vector<MatrixX<double>>& A() const { return *A_; };
  const BlockSparseMatrix<double>& J() const { return *J_; };

  SapHessianFactorizationType solver_type() const { return type_; }

  void Update(const std::vector<MatrixX<double>>& G);

  void SolveInPlace(EigenPtr<MatrixX<double>> rhs) const;

  // Makes a deep-copy of `this` factorization.
  // @warning The new clone holds references to the the original A and J to
  // which `this` factorization references to.
  std::unique_ptr<SapHessianFactorization> Clone() const;

  MatrixX<double> MakeFullMatrix() const;

 private:
  MatrixX<double> CalcDenseHessian(
      const std::vector<MatrixX<double>>& G) const;

  void CalcDenseFactorization(const std::vector<MatrixX<double>>& G,
                              Eigen::LDLT<MatrixX<double>>* factization) const;

  std::variant<Eigen::LDLT<MatrixX<double>>, std::unique_ptr<SuperNodalSolver>>
      factorization_;
  SapHessianFactorizationType type_{SapHessianFactorizationType::kNotSpecified};
  const std::vector<MatrixX<double>>* A_{nullptr};
  const BlockSparseMatrix<double>* J_{nullptr};
};

}
}
}
}
