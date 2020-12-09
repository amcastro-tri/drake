#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/eigen_types.h"

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;
#define PRINT_DENSE_VEC(a) \
  std::cout << #a ":\n" << VectorXd(a).transpose() << std::endl;

#define PRINT_DENSE_AUTODIFF(a)                                         \
  std::cout << #a ":\n"                                                 \
            << "value: " << a.value() << std::endl                      \
            << "derivatives: " << VectorXd(a.derivatives()).transpose() \
            << std::endl;

using Eigen::SparseMatrix;
using SparseMatrixd = SparseMatrix<double>;
using SparseVectord = Eigen::SparseVector<double>;
using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Triplet = Eigen::Triplet<double>;

// Naive "implementation" of sparse automatic differentiation.
using AutoDiffSparseVectord = Eigen::AutoDiffScalar<SparseVectord>;

namespace drake {
namespace test {

using NonZeroEntry = std::pair<int, double>;

template <typename Iterator>
Eigen::SparseVector<double> MakeSparseVectorFromPairs(int size, Iterator begin,
                                                      Iterator end) {
  typedef typename std::iterator_traits<Iterator>::value_type IteratorValueType;
  static_assert(std::is_same<IteratorValueType, NonZeroEntry>::value);
  Eigen::SparseVector<double> x(size);
  for (auto it = begin; it != end; ++it) {
    const int i = it->first;
    const double v = it->second;
    x.coeffRef(i) = v;
  }
  return x;
}

SparseVectord MakeSparseVector(int size,
                               const std::vector<NonZeroEntry>& entries) {
  return MakeSparseVectorFromPairs(size, entries.begin(), entries.end());
}

// At each i-th point on a 1D grid, this method computes:
//   Rᵢ = −uᵢ₋₁ + 2 uᵢ − uᵢ₊₁
// N.B. Actually the negative of the Laplacian operator.
// N.B. Zero gradient BCs are enforced.
template <typename T>
VectorX<T> CalcDiscreteLaplacian(const VectorX<T>& u) {
  const int n = u.size();
  VectorX<T> R(n);
  R(0) = u(0) - u(1);
  for (int i = 1; i < n - 1; ++i) {
    R(i) = -u(i - 1) + 2.0 * u(i) - u(i + 1);
  }
  R(n - 1) = -u(n - 2) + u(n - 1);
  return R;
}

VectorX<AutoDiffSparseVectord> InitializeAutoDiff(const VectorXd& x) {
  const int n = x.size();
  VectorX<AutoDiffSparseVectord> x_ad(n);
  for (int i = 0; i < n; ++i) {
    x_ad(i).value() = x(i);
    x_ad(i).derivatives().resize(n);
    x_ad(i).derivatives().coeffRef(i) = 1.0;
  }
  return x_ad;
}

SparseMatrixd AutoDiffToGradientMatrix(
    const VectorX<AutoDiffSparseVectord>& x) {
  const int rows = x.size();
  int cols = 0;  // some entries in x might have empty derivatives.
  for (int i = 0; i < rows; ++i) {
    cols = std::max(cols, static_cast<int>(x(i).derivatives().size()));
  }

  std::vector<Triplet> triplets;
  for (int i = 0; i < rows; ++i) {
    const SparseVectord& ith_row = x(i).derivatives();
    for (int k = 0; k < ith_row.nonZeros(); ++k) {
      const int j = *(ith_row.innerIndexPtr() + k);
      const double v = *(ith_row.valuePtr() + k);
      triplets.push_back({i, j, v});
    }
  }

  SparseMatrixd G(rows, cols);
  G.setFromTriplets(triplets.begin(), triplets.end());

  return G;
}

GTEST_TEST(AutoDiffSparseVectord, LaplacianOperator) {
  const int size = 10;
  // Values for u are arbitrary since the operator is linear.
  const VectorXd u = VectorXd::LinSpaced(size, 1.0, 10.0);

  const VectorX<AutoDiffSparseVectord> u_autodiff = InitializeAutoDiff(u);

  const VectorX<AutoDiffSparseVectord> R = CalcDiscreteLaplacian(u_autodiff);
  SparseMatrixd Lop = AutoDiffToGradientMatrix(R);
  PRINT_VAR(Lop.nonZeros());
  PRINT_VARn(Lop);
}

GTEST_TEST(AutoDiffSparseVectord, InitializeAutoDiff) {
  const int size = 10;
  VectorXd x = VectorXd::LinSpaced(size, 1.0, 10.0);
  PRINT_VAR(x.transpose());

  VectorX<AutoDiffSparseVectord> x_autodiff = InitializeAutoDiff(x);
  PRINT_VAR(x_autodiff.transpose());

  SparseMatrixd x_gradient = AutoDiffToGradientMatrix(x_autodiff);
  PRINT_VARn(x_gradient);
  // PRINT_VARn(MatrixXd(x_gradient));
}

GTEST_TEST(AutoDiffSparseVectord, Construction) {
  AutoDiffSparseVectord x(2.5);
  PRINT_VAR(x);
  PRINT_DENSE_VEC(x.derivatives());
}

GTEST_TEST(AutoDiffSparseVectord, Construction2) {
  const int size = 5;
  const AutoDiffSparseVectord x(1.5,
                                MakeSparseVector(size, {{1, 1.5}, {3, 2.2}}));
  PRINT_VAR(x);
  PRINT_DENSE_VEC(x.derivatives());
}

GTEST_TEST(AutoDiffSparseVectord, Addition) {
  const int size = 5;
  const AutoDiffSparseVectord x(
      -1.0, MakeSparseVector(size, {{1, 1.5}, {3, 2.2}, {4, -2.0}}));
  const AutoDiffSparseVectord y(1.5,
                                MakeSparseVector(size, {{0, 0.5}, {4, -1.0}}));

  const AutoDiffSparseVectord z = x + y;

  PRINT_DENSE_AUTODIFF(x);
  PRINT_DENSE_AUTODIFF(y);
  PRINT_DENSE_AUTODIFF(z);

  PRINT_VARn(z.derivatives());
}

GTEST_TEST(AutoDiffSparseVectord, Multiplication) {
  const int size = 5;
  const AutoDiffSparseVectord x(
      -1.0, MakeSparseVector(size, {{1, 1.5}, {3, 2.2}, {4, -2.0}}));
  const AutoDiffSparseVectord y(1.5,
                                MakeSparseVector(size, {{0, 0.5}, {4, -1.0}}));

  const AutoDiffSparseVectord z = x * y;

  PRINT_DENSE_AUTODIFF(x);
  PRINT_DENSE_AUTODIFF(y);
  PRINT_DENSE_AUTODIFF(z);

  PRINT_VARn(z.derivatives());
}

GTEST_TEST(AutoDiffSparseVectord, SinCos) {
  using std::cos;
  using std::sin;
  const int size = 20;
  const AutoDiffSparseVectord x(1.5,
                                MakeSparseVector(size, {{0, 0.5}, {13, -1.0}}));

  PRINT_DENSE_AUTODIFF(x);

  const AutoDiffSparseVectord s = sin(x);
  PRINT_DENSE_AUTODIFF(s);
  PRINT_VARn(s.derivatives());

  const AutoDiffSparseVectord c = cos(x);
  PRINT_DENSE_AUTODIFF(c);
  PRINT_VARn(c.derivatives());
}

}  // namespace test
}  // namespace drake
