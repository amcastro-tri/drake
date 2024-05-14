#include "drake/multibody/contact_solvers/sap/sap_hessian_factorization.h"

#include <algorithm>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "drake/multibody/contact_solvers/block_sparse_supernodal_solver.h"
#include "drake/multibody/contact_solvers/conex_supernodal_solver.h"
//#include "drake/multibody/contact_solvers/sap/sap_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

SapHessianFactorization::SapHessianFactorization(
    SapHessianFactorizationType type, const std::vector<MatrixX<double>>* A_ptr,
    const BlockSparseMatrix<double>* J_ptr)
    : type_(type), A_(A_ptr), J_(J_ptr) {
  switch (type) {
    case SapHessianFactorizationType::kConex:
      factorization_ = std::make_unique<ConexSuperNodalSolver>(
          J().block_rows(), J().get_blocks(), A());
      break;
    case SapHessianFactorizationType::kBlockSparseCholesky:
      factorization_ = std::make_unique<BlockSparseSuperNodalSolver>(
          J().block_rows(), J().get_blocks(), A());
      break;
    case SapHessianFactorizationType::kDense:
      // For now an empty factorization until we call Update().
      factorization_ = Eigen::LDLT<MatrixX<double>>();
      break;
    case SapHessianFactorizationType::kNotSpecified:
      break;
  }
}

std::unique_ptr<SapHessianFactorization> SapHessianFactorization::Clone()
    const {
  // TODO(amcastro-tri): A more efficient implementation would take advantage of
  // a Clone() method in the stored variant. This might lead to additional and
  // costly symbolic factorization.
  throw std::runtime_error("Attempting to clone the expensive Hessian.");
  return std::make_unique<SapHessianFactorization>(type_, A_, J_);
}

void SapHessianFactorization::Update(const std::vector<MatrixX<double>>& G) {
  if (solver_type() == SapHessianFactorizationType::kDense) {
    auto& dense_factorization =
        std::get<Eigen::LDLT<MatrixX<double>>>(factorization_);
    CalcDenseFactorization(G, &dense_factorization);
  } else {
    auto& supernodal_solver =
        std::get<std::unique_ptr<SuperNodalSolver>>(factorization_);
    supernodal_solver->SetWeightMatrix(G);
    if (!supernodal_solver->Factor()) {
      throw std::logic_error("SapSolver: Supernodal factorization failed.");
    }
  }
}

void SapHessianFactorization::SolveInPlace(
    EigenPtr<MatrixX<double>> rhs) const {
  const int num_rhs = rhs->cols();
  if (solver_type() == SapHessianFactorizationType::kDense) {
    auto& dense_hessian =
        std::get<Eigen::LDLT<MatrixX<double>>>(factorization_);
    for (int i = 0; i < num_rhs; ++i) {
      auto rhs_i = rhs->col(i);
      rhs->col(i) = dense_hessian.solve(rhs_i);
    }
  } else {
    auto& supernodal_solver =
        std::get<std::unique_ptr<SuperNodalSolver>>(factorization_);
    for (int i = 0; i < num_rhs; ++i) {
      auto rhs_i = rhs->col(i);
      rhs->col(i) = supernodal_solver->Solve(rhs_i);
    }    
  }
}

MatrixX<double> SapHessianFactorization::CalcDenseHessian(
    const std::vector<MatrixX<double>>& G) const {
  // Explicitly build dense Hessian.
  // These matrices could be saved in the cache. However this method is only
  // intended as an alternative for debugging and optimizing it might not be
  // worth it.
  const int nv = [this]() {
    int sz = 0;
    for (const auto& Ai : A()) {
      sz += Ai.rows();
    }
    return sz;
  }();

  // Make dense dynamics matrix.
  MatrixX<double> Adense = MatrixX<double>::Zero(nv, nv);
  int offset = 0;
  for (const auto& Ac : A()) {
    const int nv_clique = Ac.rows();
    Adense.block(offset, offset, nv_clique, nv_clique) = Ac;
    offset += nv_clique;
  }

  // Make dense Jacobian matrix.
  const MatrixX<double> Jdense = J().MakeDenseMatrix();

  // Make dense Hessian matrix G.
  const int nk = Jdense.rows();
  MatrixX<double> Gdense = MatrixX<double>::Zero(nk, nk);
  offset = 0;
  for (const auto& Gi : G) {
    const int ni = Gi.rows();
    Gdense.block(offset, offset, ni, ni) = Gi;
    offset += ni;
  }

  const MatrixX<double> H = Adense + Jdense.transpose() * Gdense * Jdense;

  return H;
}

void SapHessianFactorization::CalcDenseFactorization(
    const std::vector<MatrixX<double>>& G,
    Eigen::LDLT<MatrixX<double>>* dense_factorization) const {
  const MatrixX<double> H = CalcDenseHessian(G);

  // Factorize Hessian.
  // TODO(amcastro-tri): when T = AutoDiffXd propagate gradients analytically
  // using the chain rule so that here we can use T = double for performance.
  // N.B. The support for dense algebra is mostly for testing purposes, even
  // though the computation of the dense H (and in particular of the Jᵀ⋅G⋅J
  // term) is very costly. Therefore below we decided to trade off speed for
  // stability when choosing to use an LDLT decomposition instead of a slightly
  // faster, though less stable, LLT decomposition.
  dense_factorization->compute(H);
  if (dense_factorization->info() != Eigen::Success) {
    // TODO(amcastro-tri): Unit test this condition.
    throw std::runtime_error("Dense LDLT factorization of the Hessian failed.");
  }
      
}

MatrixX<double> SapHessianFactorization::MakeFullMatrix() const {
  if (solver_type() == SapHessianFactorizationType::kDense) {
    auto& dense_factorization =
        std::get<Eigen::LDLT<MatrixX<double>>>(factorization_);
    return dense_factorization.reconstructedMatrix();
  } else {
    auto& supernodal_solver =
        std::get<std::unique_ptr<SuperNodalSolver>>(factorization_);
    return supernodal_solver->MakeFullMatrix();
  }  
}

}
}
}
}
