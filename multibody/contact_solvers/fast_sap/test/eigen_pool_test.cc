#include "drake/multibody/contact_solvers/fast_sap/eigen_pool.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using std::pair;
using std::vector;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

const Eigen::Matrix3d S33 =
    (Eigen::Matrix3d() << 4, 1, 2,
                          1, 5, 3,
                          2, 3, 6).finished();  

GTEST_TEST(EigenPoolTest, ColumnVectorConstructor) {
  vector<int> sizes = {3, 2, 4};
  EigenPool<Eigen::VectorXd> pool(sizes);

  ASSERT_EQ(pool.size(), 3);
  EXPECT_EQ(pool[0].rows(), 3);
  EXPECT_EQ(pool[1].rows(), 2);
  EXPECT_EQ(pool[2].rows(), 4);
}

GTEST_TEST(EigenPoolTest, MatrixConstructor) {
  vector<pair<int, int>> shape_list = {{2, 2}, {3, 1}, {1, 4}};
  EigenPool<MatrixXd> pool(shape_list);

  ASSERT_EQ(pool.size(), 3);
  EXPECT_EQ(pool[0].rows(), 2);
  EXPECT_EQ(pool[0].cols(), 2);
  EXPECT_EQ(pool[1].rows(), 3);
  EXPECT_EQ(pool[1].cols(), 1);
  EXPECT_EQ(pool[2].rows(), 1);
  EXPECT_EQ(pool[2].cols(), 4);
}

GTEST_TEST(EigenPoolTest, Matrix3Constructor) {
  const int kDim = 3;

  vector<pair<int, int>> inconsistent_shapes = {
      {kDim, 2}, {kDim, kDim}, {kDim, kDim}};
  DRAKE_EXPECT_THROWS_MESSAGE(EigenPool<Matrix3d>(inconsistent_shapes),
                              "Shape does not match compile time sizes.");

  vector<pair<int, int>> shapes = {{kDim, kDim}, {kDim, kDim}};
  EigenPool<Matrix3d> pool(shapes);

  ASSERT_EQ(pool.size(), 2);
  EXPECT_EQ(pool[0].rows(), kDim);
  EXPECT_EQ(pool[0].cols(), kDim);
  EXPECT_EQ(pool[1].rows(), kDim);
  EXPECT_EQ(pool[1].cols(), kDim);
}

GTEST_TEST(EigenPoolTest, PushBack) {
  EigenPool<Eigen::Matrix3d> pool;  

  std::vector<Matrix3d> data = {S33, 2.0 * S33, 3.0 * S33};
  pool.PushBack(data);

  EXPECT_EQ(pool.size(), 3);
  for (int i = 0; i < pool.size(); ++i) {
    EXPECT_EQ(Matrix3d(pool[i]), data[i]);
  }
}

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
