#include "drake/systems/framework/subvector.h"

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/supervector.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace systems {
namespace {

const int kSubVectorLength = 2;

class SubvectorTest : public ::testing::Test {
 protected:
  void SetUp() override { vector_ = BasicVector<double>::Make({1, 2, 3, 4}); }

  std::unique_ptr<VectorBase<double>> vector_;
};

TEST_F(SubvectorTest, NullptrVector) {
  EXPECT_THROW(Subvector<double> subvec(nullptr), std::logic_error);
}

TEST_F(SubvectorTest, EmptySubvector) {
  Subvector<double> subvec(vector_.get());
  EXPECT_EQ(0, subvec.size());
  EXPECT_THROW(subvec.GetAtIndex(0), std::runtime_error);
}

TEST_F(SubvectorTest, OutOfBoundsSubvector) {
  EXPECT_THROW(Subvector<double>(vector_.get(), 1, 4), std::out_of_range);
}

TEST_F(SubvectorTest, Access) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  EXPECT_EQ(2, subvec.GetAtIndex(0));
  EXPECT_EQ(3, subvec.GetAtIndex(1));
  EXPECT_THROW(subvec.GetAtIndex(2), std::runtime_error);
}

TEST_F(SubvectorTest, Copy) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  Eigen::Vector2d expected;
  expected << 2, 3;
  EXPECT_EQ(expected, subvec.CopyToVector());
}

// Tests that writes to the subvector pass through to the sliced vector.
TEST_F(SubvectorTest, Mutation) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  VectorX<double> next_value(kSubVectorLength);
  next_value << 5, 6;
  subvec.SetFromVector(next_value);
  EXPECT_EQ(5, subvec.GetAtIndex(0));
  EXPECT_EQ(6, subvec.GetAtIndex(1));

  subvec.SetAtIndex(1, 42);
  EXPECT_EQ(1, vector_->GetAtIndex(0));
  EXPECT_EQ(5, vector_->GetAtIndex(1));
  EXPECT_EQ(42, vector_->GetAtIndex(2));
  EXPECT_EQ(4, vector_->GetAtIndex(3));
}

// Tests that a subvector can be SetFrom another VectorBase.
TEST_F(SubvectorTest, SetFrom) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  auto next_value = BasicVector<double>::Make({7, 8});
  subvec.SetFrom(*next_value);
  EXPECT_EQ(7, subvec.GetAtIndex(0));
  EXPECT_EQ(8, subvec.GetAtIndex(1));
}

// Tests that the Subvector can be addressed as an array.
TEST_F(SubvectorTest, ArrayOperator) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  subvec[0] = 42;
  EXPECT_EQ(42, vector_->GetAtIndex(1));
  EXPECT_EQ(3, subvec[1]);
}

// Tests that a VectorBase can be added to a Subvector.
TEST_F(SubvectorTest, PlusEq) {
  BasicVector<double> addend(2);
  addend.SetAtIndex(0, 7);
  addend.SetAtIndex(1, 8);

  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  subvec += addend;

  EXPECT_EQ(1, vector_->GetAtIndex(0));
  EXPECT_EQ(9, vector_->GetAtIndex(1));
  EXPECT_EQ(11, vector_->GetAtIndex(2));
  EXPECT_EQ(4, vector_->GetAtIndex(3));
}

// Tests that a Subvector can be added to an Eigen vector.
TEST_F(SubvectorTest, ScaleAndAddToVector) {
  VectorX<double> target(2);
  target << 100, 1000;

  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  subvec.ScaleAndAddToVector(1, target);

  Eigen::Vector2d expected;
  expected << 102, 1003;
  EXPECT_EQ(expected, target);
}

// TODO(david-german-tri): Once GMock is available in the Drake build, add a
// test case demonstrating that the += operator on Subvector calls
// ScaleAndAddToVector on the addend.

TEST_F(SubvectorTest, PlusEqInvalidSize) {
  BasicVector<double> addend(1);
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  EXPECT_THROW(subvec += addend, std::out_of_range);
}

TEST_F(SubvectorTest, AddToVectorInvalidSize) {
  VectorX<double> target(3);
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  EXPECT_THROW(subvec.ScaleAndAddToVector(1, target), std::out_of_range);
}

// Tests SetZero functionality in VectorBase.
TEST_F(SubvectorTest, SetZero) {
  Subvector<double> subvec(vector_.get(), 0, kSubVectorLength);
  subvec.SetZero();
  for (int i = 0; i < subvec.size(); i++) EXPECT_EQ(subvec.GetAtIndex(i), 0);
}

// Tests the infinity norm.
TEST_F(SubvectorTest, InfNorm) {
  Subvector<double> subvec(vector_.get(), 0, kSubVectorLength);
  EXPECT_EQ(subvec.NormInf(), 2);
}

// Tests the infinity norm for an autodiff type.
TEST_F(SubvectorTest, InfNormAutodiff) {
  AutoDiffXd element0;
  element0.value() = -11.5;
  element0.derivatives() = Eigen::Vector2d(1.5, -2.5);
  AutoDiffXd element1;
  element1.value() = 22.5;
  element1.derivatives() = Eigen::Vector2d(-3.5, 4.5);
  auto basic_vector = BasicVector<AutoDiffXd>::Make({element0, element1});
  Subvector<AutoDiffXd> subvec(basic_vector.get(), 0, 2);

  // The element1 has the max absolute value of the AutoDiffScalar's scalar.
  // It is positive, so the sign of its derivatives remains unchanged.
  AutoDiffXd expected_norminf;
  expected_norminf.value() = 22.5;
  expected_norminf.derivatives() = Eigen::Vector2d(-3.5, 4.5);
  EXPECT_EQ(subvec.NormInf().value(), expected_norminf.value());
  EXPECT_EQ(subvec.NormInf().derivatives(), expected_norminf.derivatives());

  // The element0 has the max absolute value of the AutoDiffScalar's scalar.
  // It is negative, so the sign of its derivatives gets flipped.
  basic_vector->GetAtIndex(0).value() = -33.5;
  expected_norminf.value() = 33.5;
  expected_norminf.derivatives() = Eigen::Vector2d(-1.5, 2.5);
  EXPECT_EQ(subvec.NormInf().value(), expected_norminf.value());
  EXPECT_EQ(subvec.NormInf().derivatives(), expected_norminf.derivatives());
}

// Tests all += * operations for VectorBase.
TEST_F(SubvectorTest, PlusEqScaled) {
  Subvector<double> orig_vec(vector_.get(), 0, kSubVectorLength);
  BasicVector<double> vec1(2), vec2(2), vec3(2), vec4(2), vec5(2);
  Eigen::Vector2d ans1, ans2, ans3, ans4, ans5;
  vec1.get_mutable_value() << 1, 2;
  vec2.get_mutable_value() << 3, 5;
  vec3.get_mutable_value() << 7, 11;
  vec4.get_mutable_value() << 13, 17;
  vec5.get_mutable_value() << 19, 23;
  VectorBase<double>& v1 = vec1;
  VectorBase<double>& v2 = vec2;
  VectorBase<double>& v3 = vec3;
  VectorBase<double>& v4 = vec4;
  VectorBase<double>& v5 = vec5;

  orig_vec.SetZero();
  orig_vec.PlusEqScaled(2, v1);
  EXPECT_EQ(orig_vec.GetAtIndex(0), 2);
  EXPECT_EQ(orig_vec.GetAtIndex(1), 4);

  orig_vec.SetZero();
  orig_vec.PlusEqScaled({{2, v1}, {3, v2}});
  EXPECT_EQ(orig_vec.GetAtIndex(0), 11);
  EXPECT_EQ(orig_vec.GetAtIndex(1), 19);

  orig_vec.SetZero();
  orig_vec.PlusEqScaled({{2, v1}, {3, v2}, {5, v3}});
  EXPECT_EQ(orig_vec.GetAtIndex(0), 46);
  EXPECT_EQ(orig_vec.GetAtIndex(1), 74);

  orig_vec.SetZero();
  orig_vec.PlusEqScaled({{2, v1}, {3, v2}, {5, v3}, {7, v4}});
  EXPECT_EQ(orig_vec.GetAtIndex(0), 137);
  EXPECT_EQ(orig_vec.GetAtIndex(1), 193);

  orig_vec.SetZero();
  orig_vec.PlusEqScaled({{2, v1}, {3, v2}, {5, v3}, {7, v4}, {11, v5}});
  EXPECT_EQ(orig_vec.GetAtIndex(0), 346);
  EXPECT_EQ(orig_vec.GetAtIndex(1), 446);
}

// Verifies we can request a contiguous in memory Eigen vector from SubVector
// when the original vector being subdivided is a contiguous BasicVector.
GTEST_TEST(SubvectorIsContiguous, WithinBasicVector) {
  auto vector = BasicVector<double>::Make({1, 2, 3, 4, 5, 6});
  Subvector<double> subvec(vector.get(), 1 /* first element */, 2 /* size */);
  EXPECT_TRUE(subvec.is_contiguous());
  EXPECT_EQ(Eigen::Vector2d(2.0, 3.0), subvec.get_contiguous_block());
  subvec.get_mutable_contiguous_block().coeffRef(1) = -1.0;
  EXPECT_EQ(Eigen::Vector2d(2.0, -1.0), subvec.get_contiguous_block());
}

// Verifies we can request a contiguous in memory Eigen vector from SubVector
// when the original vector being subdivided is a contiguous Subvector.
GTEST_TEST(SubvectorIsContiguous, WithinContiguousSubvector) {
  auto vector = BasicVector<double>::Make({1, 2, 3, 4, 5, 6});
  Subvector<double> subvec(vector.get(), 1 /* first element */, 3 /* size */);
  Subvector<double> subsubvec(&subvec, 1 /* first element */, 2 /* size */);
  EXPECT_TRUE(subvec.is_contiguous());
  EXPECT_TRUE(subsubvec.is_contiguous());
  EXPECT_EQ(Eigen::Vector3d(2.0, 3.0, 4.0), subvec.get_contiguous_block());
  EXPECT_EQ(Eigen::Vector2d(3.0, 4.0), subsubvec.get_contiguous_block());
  subsubvec.get_mutable_contiguous_block().coeffRef(1) = -1.0;
  EXPECT_EQ(Eigen::Vector2d(3.0, -1.0), subsubvec.get_contiguous_block());
}

// Verify that Subvector still can map a contiguous block of memory within a
// non-contiguous Supervector if the block actually is contiguous in memory.
GTEST_TEST(SubvectorIsContiguous, WithinContiguousPartOfASupervector) {
  auto vector1 = BasicVector<double>::Make({1, 2, 3, 4});
  auto vector2 = BasicVector<double>::Make({5, 6, 7, 8, 9, 10});
  // Concatenate into supervector = {vector1, vector2}.
  auto supervector = std::make_unique<Supervector<double>>(
      std::vector<VectorBase<double>*>{vector1.get(), vector2.get()});

  // Build a Supervector and verify it indeed is the concatenation of vector1
  // and vector2.
  VectorX<double> expected_supervector_value(supervector->size());
  expected_supervector_value.segment(0, 4) = vector1->get_value();
  expected_supervector_value.segment(4, 6) = vector2->get_value();
  EXPECT_EQ(expected_supervector_value.rows(), supervector->size());
  EXPECT_EQ(expected_supervector_value.cols(), 1);
  for (int i = 0; i < supervector->size(); ++i) {
    EXPECT_EQ(expected_supervector_value[i], supervector->GetAtIndex(i));
  }

  // Create a Subvector into a contiguous chunk of the Supervector.
  Subvector<double> subvec(
      supervector.get(), 5 /* first element */, 3 /* size */);

  // Even though the original Supervector is not contiguous in memory, the
  // Subvector is.
  EXPECT_TRUE(subvec.is_contiguous());

  // Verify it points to the appropriate chunk of memory.
  EXPECT_EQ(Eigen::Vector3d(6.0, 7.0, 8.0), subvec.get_contiguous_block());

  // Create an extra layer of indirection to verify correctness.
  Subvector<double> subsubvec(&subvec, 1 /* first element */, 2 /* size */);
  EXPECT_TRUE(subsubvec.is_contiguous());
  EXPECT_EQ(Eigen::Vector2d(7.0, 8.0), subsubvec.get_contiguous_block());

  // Verify we can change an entry through subsubvec and we can see the result
  // through subvec, supervector and the original vector2.
  subsubvec.get_mutable_contiguous_block().coeffRef(1) = -1.0;
  EXPECT_EQ(Eigen::Vector2d(7.0, -1.0), subsubvec.get_contiguous_block());
  EXPECT_EQ(Eigen::Vector3d(6.0, 7.0, -1.0), subvec.get_contiguous_block());
  EXPECT_EQ(-1, supervector->GetAtIndex(7));
  EXPECT_EQ(Eigen::Vector4d({1, 2, 3, 4}), vector1->get_contiguous_block());
}

}  // namespace
}  // namespace systems
}  // namespace drake
