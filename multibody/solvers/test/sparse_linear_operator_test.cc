#include "drake/multibody/solvers/sparse_linear_operator.h"

#include <memory>

#include <gtest/gtest.h>

#include <Eigen/SparseCore>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace solvers {
namespace {

using SparseMatrixd = Eigen::SparseMatrix<double>;
using SparseVectord = Eigen::SparseVector<double>;
using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Triplet = Eigen::Triplet<double>;

// This method makes a sparse matrix that emulates the contact Jacobian that we
// would have in an application in which a subset of the vertices of a mesh is
// in contact.
//
// nc is the number of contact ponts, which in this case coincides with the
// number of particles.
// nv is the number of velocities.
// particle is a vector of size nc. For each ic-th entry, it contains the index
// of the only particle that contributes to the ic-th velocity.
// The resulting matrix is (3nc)x(3np) in size.
SparseMatrixd MakeMeshInContactJacobian(
    int num_vertices,
    const std::vector<int>& vertices_in_contact) {
  const int num_contacts = static_cast<int>(vertices_in_contact.size());
  const int nnz = 3 * num_contacts;
  std::vector<Triplet> triplets;
  triplets.reserve(nnz);
  for (int ic = 0; ic < num_contacts; ++ic) {
    const int v = vertices_in_contact[ic];
    DRAKE_DEMAND(v < num_vertices);
    triplets.emplace_back(3 * ic    , 3 * v    , 1.0);
    triplets.emplace_back(3 * ic + 1, 3 * v + 1, 1.0);
    triplets.emplace_back(3 * ic + 2, 3 * v + 2, 1.0);
  }
  SparseMatrixd J(3 * num_contacts, 3 * num_vertices);
  J.setFromTriplets(triplets.begin(), triplets.end());
  return J;
}

class ContactJacobianTest : public ::testing::Test {
 public:
  void SetUp() override { 
      J_ = MakeMeshInContactJacobian(kNumVertices_, contact_set_);
      Lop_ = std::make_unique<SparseLinearOperator<double>>("Jc", &J_);
      kRows_ = 3 * static_cast<int>(contact_set_.size());
      kCols_ = 3 * kNumVertices_;
  }

 protected:
  const int kNumVertices_ = 300;    
  const std::vector<int> contact_set_{12, 3, 75, 100, 99, 233, 7};
  int kRows_;
  int kCols_;
  SparseMatrixd J_;
  std::unique_ptr<SparseLinearOperator<double>> Lop_;
};

TEST_F(ContactJacobianTest, Construction) {
  EXPECT_EQ(Lop_->name(), "Jc");
  EXPECT_EQ(Lop_->rows(), kRows_);
  EXPECT_EQ(Lop_->cols(), kCols_);
}

TEST_F(ContactJacobianTest, MultiplyDense) {
  VectorXd y(kRows_);
  const VectorXd x = VectorXd::LinSpaced(kCols_, 0.0, 1.0);
  Lop_->Multiply(x, &y);

  VectorXd y_expected = J_ * x;
  // y's values should equal those in y_expected bit by bit.  
  EXPECT_EQ(y, y_expected);
}

TEST_F(ContactJacobianTest, MultiplyByTransposeDense) {
  VectorXd y(kCols_);
  const VectorXd x = VectorXd::LinSpaced(kRows_, 0.0, 1.0);
  Lop_->MultiplyByTranspose(x, &y);

  VectorXd y_expected = J_.transpose() * x;
  // y's values should equal those in y_expected bit by bit.  
  EXPECT_EQ(y, y_expected);
}

TEST_F(ContactJacobianTest, MultiplySparse) {
  SparseVectord y(kRows_);
  const SparseVectord x = VectorXd::LinSpaced(kCols_, 0.0, 1.0).sparseView();
  Lop_->Multiply(x, &y);

  VectorXd y_expected = J_ * x;
  // y's values should equal those in y_expected bit by bit.  
  EXPECT_EQ(VectorXd(y), y_expected);
}

TEST_F(ContactJacobianTest, MultiplyByTransposeSparse) {
  SparseVectord y(kCols_);
  const SparseVectord x = VectorXd::LinSpaced(kRows_, 0.0, 1.0).sparseView();
  Lop_->MultiplyByTranspose(x, &y);

  VectorXd y_expected = J_.transpose() * x;
  // y's values should equal those in y_expected bit by bit.  
  EXPECT_EQ(VectorXd(y), y_expected);
}

}  // namespace
}  // namespace solvers
}  // namespace multibody
}  // namespace drake