#include "drake/multibody/contact_solvers/supernodal_solver.h"

#include "conex/clique_ordering.h"
#include "gtest/gtest.h"

#undef NDEBUG

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::get;
using std::vector;

using RowData = vector<std::pair<MatrixXd, vector<int>>>;

int NumberOfEquations(const MatrixBlocks& r) { return r.at(0).first.rows(); }

int NumberOfEquations(const std::vector<MatrixBlocks>& row_data) {
  int y = 0;
  for (auto& r : row_data) {
    y += r.at(0).first.rows();
  }
  return y;
}

void FillRowData(int row_start, const RowData& row_data, MatrixXd* Y) {
  for (size_t i = 0; i < row_data.size(); i++) {
    auto& M = row_data.at(i).first;
    // This assumes that the columns of row_data are contiguous
    Y->block(row_start, row_data.at(i).second.at(0), M.rows(), M.cols()) = M;
  }
}

MatrixXd BuildFullJacobian(int num_vars, const vector<RowData>& row_data) {
  int row_start = 0;
  MatrixXd J(NumberOfEquations(row_data), num_vars);
  J.setZero();
  for (size_t i = 0; i < row_data.size(); i++) {
    FillRowData(row_start, row_data.at(i), &J);
    row_start += NumberOfEquations(row_data.at(i));
  }
  return J;
}

MatrixXd BuildFullWeightMatrix(const vector<RowData>& row_data,
                               const vector<MatrixXd>& weight_matrix) {
  int row_start = 0;
  MatrixXd J(NumberOfEquations(row_data), NumberOfEquations(row_data));
  J.setZero();
  for (size_t i = 0; i < row_data.size(); i++) {
    int m = NumberOfEquations(row_data.at(i));
    J.block(row_start, row_start, m, m) = weight_matrix.at(i);
    row_start += m;
  }
  return J;
}

MatrixXd BuildFullDiagonalBlocks(int num_vars, const RowData& diagonal_blocks) {
  MatrixXd D(num_vars, num_vars);
  D.setZero();
  for (auto& c : diagonal_blocks) {
    int s = c.second.at(0);
    int size = c.second.size();
    D.block(s, s, size, size) = c.first;
  }
  return D;
}

void DoUpdateWeightTest(SuperNodalSolver& solver,
                        const vector<MatrixXd>& weight_matrix,
                        const vector<RowData>& row_data, const MatrixXd& J,
                        const MatrixXd& D) {
  solver.SetWeightMatrix(weight_matrix);
  auto W = BuildFullWeightMatrix(row_data, weight_matrix);
  MatrixXd M = solver.FullMatrix();
  MatrixXd T = D + J.transpose() * W * J;
  EXPECT_NEAR((T - M).norm(), 0, 1e-9);

  MatrixXd x_ref = Eigen::MatrixXd::Random(T.rows(), 1);
  solver.Factor();
  MatrixXd x_calc = solver.Solve(M * x_ref);
  EXPECT_NEAR((x_calc - x_ref).norm(), 0, 1e-9);
}

void DoTests(int num_vars, const RowData& diagonal_blocks,
             const vector<MatrixXd>& weight_matrix,
             const vector<RowData>& row_data) {
  auto J = BuildFullJacobian(num_vars, row_data);
  auto D = BuildFullDiagonalBlocks(num_vars, diagonal_blocks);

  SuperNodalSolver solver(diagonal_blocks, row_data);

  DoUpdateWeightTest(solver, weight_matrix, row_data, J, D);
  auto weight_matrix_2 = weight_matrix;
  for (auto& w : weight_matrix_2) {
    w.setIdentity();
  }
  DoUpdateWeightTest(solver, weight_matrix_2, row_data, J, D);
}

GTEST_TEST(SupernodalSolver, BasicTests) {
  int num_vars = 4;

  vector<vector<int>> cols{{0}, {1}, {2}};

  vector<RowData> row_data(3);
  std::vector<MatrixXd> weight_matrix(3);
  RowData diagonal_blocks;
  for (size_t i = 0; i < cols.size(); i++) {
    int n = cols.at(i).size();
    diagonal_blocks.emplace_back(MatrixXd::Random(n, n), cols.at(i));
  }

  for (size_t i = 0; i < cols.size(); i++) {
    row_data.at(i).emplace_back(MatrixXd::Random(3, cols.at(i).size()),
                                cols.at(i));
    row_data.at(i).emplace_back(MatrixXd::Random(3, 1), vector{num_vars - 1});
    MatrixXd T = Eigen::MatrixXd::Random(3, 3);
    weight_matrix.at(i) = T * T.transpose();
  }
  DoTests(num_vars, diagonal_blocks, weight_matrix, row_data);

  row_data.clear();
  row_data.resize(3);
  for (size_t i = 0; i < cols.size(); i++) {
    row_data.at(i).emplace_back(MatrixXd::Random(3, cols.at(i).size()),
                                cols.at(i));
    weight_matrix.at(i) = Eigen::MatrixXd::Identity(3, 3);
  }
  DoTests(3, diagonal_blocks, weight_matrix, row_data);
}

GTEST_TEST(SupernodalSolver, InterfaceTest) {
  int num_row_blocks_of_J = 3;
  Eigen::MatrixXd J(9, 6);

  // clang-format off
  J << 1, 2, 0, 0, 2, 4,
       0, 1, 0, 0, 1, 3,
       1, 3, 0, 0, 2, 4,
       0, 0, 0, 0, 1, 2,
       0, 0, 0, 0, 2, 1,
       0, 0, 0, 0, 2, 3,
       0, 0, 1, 1, 0, 0,
       0, 0, 2, 1, 0, 0,
       0, 0, 3, 3, 0, 0;
  // clang-format on

  std::vector<BlockMatrixTriplet> Jtriplets(4);
  get<0>(Jtriplets.at(0)) = 0;
  get<1>(Jtriplets.at(0)) = 0;
  get<2>(Jtriplets.at(0)) = J.block(0, 0, 3, 2);

  get<0>(Jtriplets.at(1)) = 0;
  get<1>(Jtriplets.at(1)) = 2;
  get<2>(Jtriplets.at(1)) = J.block(0, 4, 3, 2);

  get<0>(Jtriplets.at(2)) = 1;
  get<1>(Jtriplets.at(2)) = 2;
  get<2>(Jtriplets.at(2)) = J.block(3, 4, 3, 2);

  get<0>(Jtriplets.at(3)) = 2;
  get<1>(Jtriplets.at(3)) = 1;
  get<2>(Jtriplets.at(3)) = J.block(6, 2, 3, 2);

  Eigen::MatrixXd W(9, 9);
  // clang-format off
  W << 1, 2, 2, 0, 0, 0, 0, 0, 0,
       2, 5, 3, 0, 0, 0, 0, 0, 0,
       2, 3, 4, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 4, 1, 1, 0, 0, 0,
       0, 0, 0, 1, 4, 2, 0, 0, 0,
       0, 0, 0, 1, 2, 5, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 4, 1, 1,
       0, 0, 0, 0, 0, 0, 1, 4, 2,
       0, 0, 0, 0, 0, 0, 1, 2, 5;
  // clang-format on

  std::vector<Eigen::MatrixXd> blocks_of_W(3);
  blocks_of_W.at(0) = W.block(0, 0, 3, 3);
  blocks_of_W.at(1) = W.block(3, 3, 3, 3);
  blocks_of_W.at(2) = W.block(6, 6, 3, 3);

  Eigen::MatrixXd M(6, 6);

  // clang-format off
  M << 1, 1, 0, 0, 0, 0,
       1, 5, 0, 0, 0, 0,
       0, 0, 4, 1, 0, 0,
       0, 0, 1, 4, 0, 0,
       0, 0, 0, 0, 4, 2,
       0, 0, 0, 0, 2, 5;
  // clang-format on

  std::vector<Eigen::MatrixXd> blocks_of_M(3);
  blocks_of_M.at(0) = M.block(0, 0, 2, 2);
  blocks_of_M.at(1) = M.block(2, 2, 2, 2);
  blocks_of_M.at(2) = M.block(4, 4, 2, 2);

  SuperNodalSolver solver(num_row_blocks_of_J, Jtriplets, blocks_of_M);
  solver.SetWeightMatrix(blocks_of_W);

  MatrixXd full_matrix_ref = M + J.transpose() * W * J;
  EXPECT_NEAR((solver.FullMatrix() - full_matrix_ref).norm(), 0, 1e-10);
}

// Test the condition when J blocks might have different number of rows. Of
// course blocks in the same block-row must have the same number of rows, though
// they don't if in different block-rows.
// In this test:
//   - J00 and J02 are 6x2 matrices (both must have the same number of rows).
//   - J12 is a 3x2 matrix.
//   - J21 is a 3x2 matrix.
// NOTE: In this test J00 and J02 happen to have the same number of columns, but
// it is not a requirement and in general will not be true.
GTEST_TEST(SupernodalSolver, SeveralPointsPerPatch) {
  int num_row_blocks_of_J = 3;
  Eigen::MatrixXd J(12, 6);

  // Here we repeat the first three rows to emulate a duplicated contact point,
  // something a contact solver should recover from gracefully.
#if 0   
  // clang-format off 
  J << 1, 2, 0, 0, 2, 4,
       0, 1, 0, 0, 1, 3,
       1, 3, 0, 0, 2, 4,
       1, 2, 0, 0, 2, 4,
       0, 1, 0, 0, 1, 3,
       1, 3, 0, 0, 2, 4,
       1, 1, 0, 0, 2, 2,
       1, 1, 0, 0, 0, 2,
       1, 1, 0, 0, 1, 2,
       0, 0, 1, 1, 0, 0,
       0, 0, 2, 1, 0, 0,
       0, 0, 3, 3, 0, 0;
  // clang-format on
#endif

  // My version
  // clang-format off
  J << 1, 2, 0, 0, 2, 4, 0, 1, 0, 0, 1, 3, 1, 3, 0, 0, 2, 4, 1, 2, 0, 0, 2, 4,
      0, 1, 0, 0, 1, 3, 1, 3, 0, 0, 2, 4, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 2, 1, 0,
      0, 0, 0, 2, 3, 0, 0, 1, 1, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 3, 3, 0, 0;
  // clang-format on

  std::vector<BlockMatrixTriplet> Jtriplets(5);
  get<0>(Jtriplets.at(0)) = 0;
  get<1>(Jtriplets.at(0)) = 0;
  get<2>(Jtriplets.at(0)) = J.block(0, 0, 6, 2);

  get<0>(Jtriplets.at(1)) = 0;
  get<1>(Jtriplets.at(1)) = 2;
  get<2>(Jtriplets.at(1)) = J.block(0, 4, 6, 2);

  get<0>(Jtriplets.at(2)) = 1;
  get<1>(Jtriplets.at(2)) = 0;
  get<2>(Jtriplets.at(2)) = J.block(6, 0, 3, 2);

  get<0>(Jtriplets.at(3)) = 1;
  get<1>(Jtriplets.at(3)) = 2;
  get<2>(Jtriplets.at(3)) = J.block(6, 4, 3, 2);

  get<0>(Jtriplets.at(4)) = 2;
  get<1>(Jtriplets.at(4)) = 1;
  get<2>(Jtriplets.at(4)) = J.block(9, 2, 3, 2);

  Eigen::MatrixXd W(12, 12);
  // clang-format off
  W << 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       2, 5, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0,       
       0, 0, 0, 9, 2, 2, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 2, 5, 3, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 2, 3, 8, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 4, 1, 1, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 1, 4, 2, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 1, 2, 5, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 1, 1,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 4, 2,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 7;
  // clang-format on

  std::vector<Eigen::MatrixXd> blocks_of_W(4);
  blocks_of_W.at(0) = W.block(0, 0, 3, 3);
  blocks_of_W.at(1) = W.block(3, 3, 3, 3);
  blocks_of_W.at(2) = W.block(6, 6, 3, 3);
  blocks_of_W.at(3) = W.block(9, 9, 3, 3);

  Eigen::MatrixXd M(6, 6);

  // clang-format off
  M << 1, 1, 0, 0, 0, 0,
       1, 5, 0, 0, 0, 0,
       0, 0, 4, 1, 0, 0,
       0, 0, 1, 4, 0, 0,
       0, 0, 0, 0, 4, 2,
       0, 0, 0, 0, 2, 5;
  // clang-format on

  std::vector<Eigen::MatrixXd> blocks_of_M(3);
  blocks_of_M.at(0) = M.block(0, 0, 2, 2);
  blocks_of_M.at(1) = M.block(2, 2, 2, 2);
  blocks_of_M.at(2) = M.block(4, 4, 2, 2);

  SuperNodalSolver solver(num_row_blocks_of_J, Jtriplets, blocks_of_M);
  solver.SetWeightMatrix(blocks_of_W);

  MatrixXd full_matrix_ref = M + J.transpose() * W * J;
  EXPECT_NEAR((solver.FullMatrix() - full_matrix_ref).norm(), 0, 1e-10);
}

GTEST_TEST(SupernodalSolver, ColumnsNotSorted) {
  int num_row_blocks_of_J = 3;
  Eigen::MatrixXd J(12, 6);

  // Here we repeat the first three rows to emulate a duplicated contact point,
  // something a contact solver should recover from gracefully.
  // clang-format off
  J << 1, 2, 0, 0, 2, 4,
       0, 1, 0, 0, 1, 3,
       1, 3, 0, 0, 2, 4,
       1, 2, 0, 0, 2, 4,
       0, 1, 0, 0, 1, 3,
       1, 3, 0, 0, 2, 4,

       1, 1, 0, 0, 1, 1,
       1, 1, 0, 0, 1, 2,
       1, 1, 0, 0, 1, 1,

       0, 0, 1, 1, 0, 0,
       0, 0, 2, 1, 0, 0,
       0, 0, 3, 3, 0, 0;
  // clang-format on

  std::vector<BlockMatrixTriplet> Jtriplets(5);
  get<0>(Jtriplets.at(0)) = 0;
  get<1>(Jtriplets.at(0)) = 0;
  get<2>(Jtriplets.at(0)) = J.block(0, 0, 6, 2);

  get<0>(Jtriplets.at(1)) = 0;
  get<1>(Jtriplets.at(1)) = 2;
  get<2>(Jtriplets.at(1)) = J.block(0, 4, 6, 2);

  get<0>(Jtriplets.at(3)) = 1;
  get<1>(Jtriplets.at(3)) = 0;
  get<2>(Jtriplets.at(3)) = J.block(6, 0, 3, 2);

  get<0>(Jtriplets.at(2)) = 1;
  get<1>(Jtriplets.at(2)) = 2;
  get<2>(Jtriplets.at(2)) = J.block(6, 4, 3, 2);

  get<0>(Jtriplets.at(4)) = 2;
  get<1>(Jtriplets.at(4)) = 1;
  get<2>(Jtriplets.at(4)) = J.block(9, 2, 3, 2);

  Eigen::MatrixXd W(12, 12);
  // clang-format off
  W << 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       2, 5, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0,       
       0, 0, 0, 9, 2, 2, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 2, 5, 3, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 2, 3, 8, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 4, 1, 1, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 1, 4, 2, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 1, 2, 5, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 1, 1,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 4, 2,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 7;
  // clang-format on

  std::vector<Eigen::MatrixXd> blocks_of_W(4);
  blocks_of_W.at(0) = W.block(0, 0, 3, 3);
  blocks_of_W.at(1) = W.block(3, 3, 3, 3);
  blocks_of_W.at(2) = W.block(6, 6, 3, 3);
  blocks_of_W.at(3) = W.block(9, 9, 3, 3);

  Eigen::MatrixXd M(6, 6);

  // clang-format off
  M << 1, 1, 0, 0, 0, 0,
       1, 5, 0, 0, 0, 0,
       0, 0, 4, 1, 0, 0,
       0, 0, 1, 4, 0, 0,
       0, 0, 0, 0, 4, 2,
       0, 0, 0, 0, 2, 5;
  // clang-format on

  std::vector<Eigen::MatrixXd> blocks_of_M(3);
  blocks_of_M.at(0) = M.block(0, 0, 2, 2);
  blocks_of_M.at(1) = M.block(2, 2, 2, 2);
  blocks_of_M.at(2) = M.block(4, 4, 2, 2);

  SuperNodalSolver solver(num_row_blocks_of_J, Jtriplets, blocks_of_M);
  solver.SetWeightMatrix(blocks_of_W);

  MatrixXd full_matrix_ref = M + J.transpose() * W * J;
  EXPECT_NEAR((solver.FullMatrix() - full_matrix_ref).norm(), 0, 1e-10);

  std::cout << full_matrix_ref << std::endl;
  std::cout << std::endl;
  std::cout << solver.FullMatrix() << std::endl;
}

// In this example there are three contact patches of one contact point each.
// There are three trees. The first two have two dofs and the third one has
// three dofs. The purpose of this test is to verify correctness when trees
// might have different number of dofs.
// The Jacobian's blocks are:
//   - J00 is a 3x2 matrix.
//   - J02 is a 3x3 matrix.
//   - J12 is a 3x3 matrix.
//   - J21 is a 3x2 matrix.
// The mass matrix's blocks are:
//   - M00 a 2x2 matrix.
//   - M11 a 2x2 matrix.
//   - M22 a 3x3 matrix.
GTEST_TEST(SupernodalSolver, DifferentTreeSizes) {
  // number of patches. In this example, it happens to equal the number of
  // contact points since each patch has only a single point.
  int num_row_blocks_of_J = 3;
  Eigen::MatrixXd J(9, 7);

  // clang-format off
  J << 1, 2, 0, 0, 2, 4, 4,
       0, 1, 0, 0, 1, 3, 3,
       1, 3, 0, 0, 2, 4, 4,
       0, 0, 0, 0, 1, 2, 2,
       0, 0, 0, 0, 2, 1, 1,
       0, 0, 0, 0, 2, 3, 3,
       0, 0, 1, 1, 0, 0, 0,
       0, 0, 2, 1, 0, 0, 0,
       0, 0, 3, 3, 0, 0, 0;
  // clang-format on

  std::vector<BlockMatrixTriplet> Jtriplets(4);
  get<0>(Jtriplets.at(0)) = 0;
  get<1>(Jtriplets.at(0)) = 0;
  get<2>(Jtriplets.at(0)) = J.block(0, 0, 3, 2);

  get<0>(Jtriplets.at(1)) = 0;
  get<1>(Jtriplets.at(1)) = 2;
  get<2>(Jtriplets.at(1)) = J.block(0, 4, 3, 3);

  get<0>(Jtriplets.at(2)) = 1;
  get<1>(Jtriplets.at(2)) = 2;
  get<2>(Jtriplets.at(2)) = J.block(3, 4, 3, 3);

  get<0>(Jtriplets.at(3)) = 2;
  get<1>(Jtriplets.at(3)) = 1;
  get<2>(Jtriplets.at(3)) = J.block(6, 2, 3, 2);

  Eigen::MatrixXd W(9, 9);
  // clang-format off
  W << 1, 2, 2, 0, 0, 0, 0, 0, 0,
       2, 5, 3, 0, 0, 0, 0, 0, 0,
       2, 3, 4, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 4, 1, 1, 0, 0, 0,
       0, 0, 0, 1, 4, 2, 0, 0, 0,
       0, 0, 0, 1, 2, 5, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 4, 1, 1,
       0, 0, 0, 0, 0, 0, 1, 4, 2,
       0, 0, 0, 0, 0, 0, 1, 2, 5;
  // clang-format on

  std::vector<Eigen::MatrixXd> blocks_of_W(3);
  blocks_of_W.at(0) = W.block(0, 0, 3, 3);
  blocks_of_W.at(1) = W.block(3, 3, 3, 3);
  blocks_of_W.at(2) = W.block(6, 6, 3, 3);

  Eigen::MatrixXd M(7, 7);

  // clang-format off
  M << 1, 1, 0, 0, 0, 0, 0,
       1, 5, 0, 0, 0, 0, 0,
       0, 0, 4, 1, 0, 0, 0,
       0, 0, 1, 4, 0, 0, 0,
       0, 0, 0, 0, 4, 2, 0,
       0, 0, 0, 0, 2, 5, 0,
       0, 0, 0, 0, 0, 0, 1;
  // clang-format on

  std::vector<Eigen::MatrixXd> blocks_of_M(3);
  blocks_of_M.at(0) = M.block(0, 0, 2, 2);
  blocks_of_M.at(1) = M.block(2, 2, 2, 2);
  blocks_of_M.at(2) = M.block(4, 4, 3, 3);

  SuperNodalSolver solver(num_row_blocks_of_J, Jtriplets, blocks_of_M);
  solver.SetWeightMatrix(blocks_of_W);

  MatrixXd full_matrix_ref = M + J.transpose() * W * J;
  EXPECT_NEAR((solver.FullMatrix() - full_matrix_ref).norm(), 0, 1e-10);
  std::cout << full_matrix_ref << std::endl;
  std::cout << std::endl;
  std::cout << solver.FullMatrix() << std::endl;
}

// Unit test for the sparisty pattern occuring on a problem with four stacks of
// two objects each. Patches and trees are provided in some arbitrary
// permutation.
GTEST_TEST(SupernodalSolver, FourStacks) {
  // For this problem each patch has a single contact point.
  // Therefore there'll be num_patches blocks of W.
  const int num_patches = 8;
  const int num_trees = 8;
  int num_row_blocks_of_J = num_patches;
  // A typical block of J.
  MatrixXd J3x6(3, 6);
  // clang-format off
  J3x6 << 1, 2, 3, 4, 5, 6,
          1, 2, 3, 4, 5, 6,
          1, 2, 3, 4, 5, 6;
  // clang-format on
  const MatrixXd Z3x6 = MatrixXd::Zero(3, 6);
  // Three patches:
  //  - Patch 1: 1 point.
  //  - Patch 2: 2 points.
  //  - Patch 3: 1 point.
  // Total of 12 rows.
  // Three trees of six dofs each = 18.
  // These are the blocks (and they are all of size 3x6):
  // (p,t) = (0,6). 3x6.
  // (p,t) = (0,7). 3x6.
  // (p,t) = (1,4). 3x6.
  // (p,t) = (1,5). 3x6.
  // (p,t) = (2,6). 3x6.
  // (p,t) = (3,0). 3x6.
  // (p,t) = (4,4). 3x6.
  // (p,t) = (5,2). 3x6.
  // (p,t) = (6,0). 3x6.
  // (p,t) = (6,1). 3x6.
  // (p,t) = (7,2). 3x6.
  // (p,t) = (7,3). 3x6.
  Eigen::MatrixXd J(24, 48);
  // clang-format off
  J <<        
       Z3x6, Z3x6, Z3x6, Z3x6, Z3x6, Z3x6, J3x6, J3x6,
       Z3x6, Z3x6, Z3x6, Z3x6, J3x6, J3x6, Z3x6, Z3x6, 
       Z3x6, Z3x6, Z3x6, Z3x6, Z3x6, Z3x6, J3x6, Z3x6, 
       J3x6, Z3x6, Z3x6, Z3x6, Z3x6, Z3x6, Z3x6, Z3x6,
       Z3x6, Z3x6, Z3x6, Z3x6, J3x6, Z3x6, Z3x6, Z3x6,
       Z3x6, Z3x6, J3x6, Z3x6, Z3x6, Z3x6, Z3x6, Z3x6,
       0.6*J3x6, J3x6, Z3x6, Z3x6, Z3x6, Z3x6, Z3x6, Z3x6,
       Z3x6, Z3x6, J3x6, J3x6, Z3x6, Z3x6, Z3x6, Z3x6;
  // clang-format on
  std::vector<BlockMatrixTriplet> Jtriplets;
  // Patch 0:
  Jtriplets.push_back({0, 6, J3x6});
  Jtriplets.push_back({0, 7, J3x6});
  // Patch 1:
  Jtriplets.push_back({1, 4, J3x6});
  Jtriplets.push_back({1, 5, J3x6});
  // Patch 2:
  Jtriplets.push_back({2, 6, J3x6});
  // Patch 3:
  Jtriplets.push_back({3, 0, J3x6});
  // Patch 4:
  Jtriplets.push_back({4, 4, J3x6});
  // Patch 5:
  Jtriplets.push_back({5, 2, J3x6});
  // Patch 6:
  Jtriplets.push_back({6, 0, 0.6 * J3x6});
  Jtriplets.push_back({6, 1, J3x6});
  // Patch 7:
  Jtriplets.push_back({7, 2, J3x6});
  Jtriplets.push_back({7, 3, J3x6});
  MatrixXd Wb(3, 3);
  // clang-format off
  Wb << 1, 2, 2,
        2, 5, 3,
        2, 3, 4;
  // clang-format on        
  Eigen::MatrixXd W(3*num_patches, 3*num_patches);
  std::vector<Eigen::MatrixXd> blocks_of_W(num_patches);
  for (int i = 0; i < num_patches; ++i) {
    W.block(3*i,3*i,3,3)= Wb;
    blocks_of_W.at(i) = Wb;
  }
  const MatrixXd Mt = VectorXd::LinSpaced(6, 1.0, 6.0).asDiagonal();
  Eigen::MatrixXd M = MatrixXd::Zero(6*num_trees, 6*num_trees);
  std::vector<Eigen::MatrixXd> blocks_of_M(num_trees);
  for (int i = 0; i < num_trees; ++i) {
    M.block(6*i, 6*i, 6, 6) = Mt;
    blocks_of_M.at(i) = Mt; 
  }
  SuperNodalSolver solver(num_row_blocks_of_J, Jtriplets, blocks_of_M);
  solver.SetWeightMatrix(blocks_of_W);
  MatrixXd full_matrix_ref = M + J.transpose() * W * J;
  EXPECT_NEAR((solver.FullMatrix() - full_matrix_ref).norm(), 0, 1e-10);
  Eigen::VectorXd x_ref = Eigen::VectorXd::Random(M.rows());
  solver.Factor();
  EXPECT_NEAR((solver.Solve(full_matrix_ref * x_ref) - x_ref).norm(), 0, 1e-8); 


  VectorXd b = full_matrix_ref * x_ref;
  solver.SolveInPlace(&b);
  EXPECT_NEAR((b - x_ref).norm(), 0, 1e-8);

}

GTEST_TEST(SupernodalSolver, ColumnSizesDifferent) {
  int num_row_blocks_of_J = 4;
  Eigen::MatrixXd J(13, 6);

  // Here we repeat the first three rows to emulate a duplicated contact point,
  // something a contact solver should recover from gracefully.
  // clang-format off
  J << 1, 2, 0,  2, 4, 0, 
       0, 1, 0,  1, 3, 0,
       1, 3, 0,  2, 4, 0,
       1, 2, 0,  2, 4, 0,
       0, 1, 0,  1, 3, 0, 
       1, 3, 0,  2, 4, 0,

       1, 1, 0,  2, 2, 0,
       1, 1, 0,  0, 2, 0,
       1, 1, 0,  1, 2, 0,

       0, 0, 1,  0, 0, 0,
       0, 0, 2,  0, 0, 0,
       0, 0, 3,  0, 0, 0,

       0, 0, 0,  0, 0, 1;
  // clang-format on

  std::vector<BlockMatrixTriplet> Jtriplets(7);
  get<0>(Jtriplets.at(0)) = 0;
  get<1>(Jtriplets.at(0)) = 0;
  get<2>(Jtriplets.at(0)) = J.block(0, 0, 6, 2);

  get<0>(Jtriplets.at(1)) = 0;
  get<1>(Jtriplets.at(1)) = 2;
  get<2>(Jtriplets.at(1)) = J.block(0, 3, 6, 2);

  get<0>(Jtriplets.at(2)) = 1;
  get<1>(Jtriplets.at(2)) = 0;
  get<2>(Jtriplets.at(2)) = J.block(6, 0, 3, 2);

  get<0>(Jtriplets.at(3)) = 1;
  get<1>(Jtriplets.at(3)) = 2;
  get<2>(Jtriplets.at(3)) = J.block(6, 3, 3, 2);

  get<0>(Jtriplets.at(4)) = 2;
  get<1>(Jtriplets.at(4)) = 1;
  get<2>(Jtriplets.at(4)) = J.block(9, 2, 3, 1);

  get<0>(Jtriplets.at(5)) = 3;
  get<1>(Jtriplets.at(5)) = 3;
  get<2>(Jtriplets.at(5)) = J.block(12, 5, 1, 1);

  get<0>(Jtriplets.at(6)) = 3;
  get<1>(Jtriplets.at(6)) = 2;
  get<2>(Jtriplets.at(6)) = J.block(12, 0, 1, 2) * 0;

  Eigen::MatrixXd W(13, 13);
  // clang-format off
  W << 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
       2, 5, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,      
       0, 0, 0, 9, 2, 2, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 2, 5, 3, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 2, 3, 8, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 4, 1, 1, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 1, 4, 2, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 1, 2, 5, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 1, 1, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 4, 2, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 7, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7;
  // clang-format on

  std::vector<Eigen::MatrixXd> blocks_of_W(5);
  blocks_of_W.at(0) = W.block(0, 0, 3, 3);
  blocks_of_W.at(1) = W.block(3, 3, 3, 3);
  blocks_of_W.at(2) = W.block(6, 6, 3, 3);
  blocks_of_W.at(3) = W.block(9, 9, 3, 3);
  blocks_of_W.at(4) = W.block(12, 12, 1, 1);

  Eigen::MatrixXd M(6, 6);

  // clang-format off
  M << 1, 1, 0, 0, 0, 0,
       1, 5, 0, 0, 0, 0,
       0, 0, 4, 0, 0, 0,
       0, 0, 0, 4, 2, 0,
       0, 0, 0, 2, 5, 0,
       0, 0, 0, 0, 0, 4;
  // clang-format on

  std::vector<Eigen::MatrixXd> blocks_of_M(4);
  blocks_of_M.at(0) = M.block(0, 0, 2, 2);
  blocks_of_M.at(1) = M.block(2, 2, 1, 1);
  blocks_of_M.at(2) = M.block(3, 3, 2, 2);
  blocks_of_M.at(3) = M.block(5, 5, 1, 1);

  SuperNodalSolver solver(num_row_blocks_of_J, Jtriplets, blocks_of_M);
  solver.SetWeightMatrix(blocks_of_W);

  MatrixXd full_matrix_ref = M + J.transpose() * W * J;
  EXPECT_NEAR((solver.FullMatrix() - full_matrix_ref).norm(), 0, 1e-10);
}

}  // namespace conex
