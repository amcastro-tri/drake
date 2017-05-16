#include "drake/common/symbolic_variable.h"

#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/test/is_memcpy_movable.h"
#include "drake/common/test/symbolic_test_util.h"

namespace drake {

using test::IsMemcpyMovable;

namespace symbolic {
namespace {

using std::move;
using std::ostringstream;
using std::unordered_map;
using std::unordered_set;
using std::vector;
using test::VarEqual;
using test::VarLess;
using test::VarNotEqual;
using test::VarNotLess;

// Provides common variables that are used by the following tests.
class VariableTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable w_{"w"};
  Eigen::Matrix<Variable, 2, 2> M_;

  void SetUp() override {
    // clang-format off
      M_ << x_, y_,
            z_, w_;
    // clang-format on
  }
};

TEST_F(VariableTest, GetId) {
  const Variable dummy{};
  const Variable x_prime{"x"};
  EXPECT_TRUE(dummy.is_dummy());
  EXPECT_FALSE(x_.is_dummy());
  EXPECT_FALSE(x_prime.is_dummy());
  EXPECT_NE(x_.get_id(), x_prime.get_id());
}

TEST_F(VariableTest, GetName) {
  const Variable x_prime{"x"};
  EXPECT_EQ(x_.get_name(), x_prime.get_name());
}

TEST_F(VariableTest, MoveCopyPreserveId) {
  Variable x{"x"};
  const size_t x_id{x.get_id()};
  const size_t x_hash{x.get_hash()};
  const Variable x_copied{x};
  const Variable x_moved{move(x)};
  EXPECT_EQ(x_id, x_copied.get_id());
  EXPECT_EQ(x_hash, x_copied.get_hash());
  EXPECT_EQ(x_id, x_moved.get_id());
  EXPECT_EQ(x_hash, x_moved.get_hash());
}

TEST_F(VariableTest, Less) {
  EXPECT_PRED2(VarNotLess, x_, x_);
  EXPECT_PRED2(VarLess, x_, y_);
  EXPECT_PRED2(VarLess, x_, z_);
  EXPECT_PRED2(VarLess, x_, w_);

  EXPECT_PRED2(VarNotLess, y_, x_);
  EXPECT_PRED2(VarNotLess, y_, y_);
  EXPECT_PRED2(VarLess, y_, z_);
  EXPECT_PRED2(VarLess, y_, w_);

  EXPECT_PRED2(VarNotLess, z_, x_);
  EXPECT_PRED2(VarNotLess, z_, y_);
  EXPECT_PRED2(VarNotLess, z_, z_);
  EXPECT_PRED2(VarLess, z_, w_);

  EXPECT_PRED2(VarNotLess, w_, x_);
  EXPECT_PRED2(VarNotLess, w_, y_);
  EXPECT_PRED2(VarNotLess, w_, z_);
  EXPECT_PRED2(VarNotLess, w_, w_);
}

TEST_F(VariableTest, EqualTo) {
  EXPECT_PRED2(VarEqual, x_, x_);
  EXPECT_PRED2(VarNotEqual, x_, y_);
  EXPECT_PRED2(VarNotEqual, x_, z_);
  EXPECT_PRED2(VarNotEqual, x_, w_);

  EXPECT_PRED2(VarNotEqual, y_, x_);
  EXPECT_PRED2(VarEqual, y_, y_);
  EXPECT_PRED2(VarNotEqual, y_, z_);
  EXPECT_PRED2(VarNotEqual, y_, w_);

  EXPECT_PRED2(VarNotEqual, z_, x_);
  EXPECT_PRED2(VarNotEqual, z_, y_);
  EXPECT_PRED2(VarEqual, z_, z_);
  EXPECT_PRED2(VarNotEqual, z_, w_);

  EXPECT_PRED2(VarNotEqual, w_, x_);
  EXPECT_PRED2(VarNotEqual, w_, y_);
  EXPECT_PRED2(VarNotEqual, w_, z_);
  EXPECT_PRED2(VarEqual, w_, w_);
}

TEST_F(VariableTest, ToString) {
  EXPECT_EQ(x_.to_string(), "x");
  EXPECT_EQ(y_.to_string(), "y");
  EXPECT_EQ(z_.to_string(), "z");
  EXPECT_EQ(w_.to_string(), "w");
}

// This test checks whether Variable is compatible with std::unordered_set.
TEST_F(VariableTest, CompatibleWithUnorderedSet) {
  unordered_set<Variable, hash_value<Variable>> uset;
  uset.emplace(x_);
  uset.emplace(y_);
}

// This test checks whether Variable is compatible with std::unordered_map.
TEST_F(VariableTest, CompatibleWithUnorderedMap) {
  unordered_map<Variable, Variable, hash_value<Variable>> umap;
  umap.emplace(x_, y_);
}

// This test checks whether Variable is compatible with std::vector.
TEST_F(VariableTest, CompatibleWithVector) {
  vector<Variable> vec;
  vec.push_back(x_);
}

TEST_F(VariableTest, EigenVariableMatrix) {
  EXPECT_PRED2(VarEqual, M_(0, 0), x_);
  EXPECT_PRED2(VarEqual, M_(0, 1), y_);
  EXPECT_PRED2(VarEqual, M_(1, 0), z_);
  EXPECT_PRED2(VarEqual, M_(1, 1), w_);
}

TEST_F(VariableTest, EigenVariableMatrixOutput) {
  ostringstream oss1;
  oss1 << M_;

  ostringstream oss2;
  oss2 << "x y"
       << "\n"
       << "z w";

  EXPECT_EQ(oss1.str(), oss2.str());
}

TEST_F(VariableTest, MemcpyKeepsVariableIntact) {
  // We have it to test that a variable with a long name (>16 chars), which is
  // not using SSO (Short String Optimization) internally, is memcpy-movable.
  const Variable long_var("12345678901234567890");
  for (const Variable& var : {x_, y_, z_, w_, long_var}) {
    EXPECT_TRUE(IsMemcpyMovable(var));
  }
}

TEST_F(VariableTest, CheckType) {
  // By default, a symbolic variable has CONTINUOUS type if not specified at
  // construction time.
  const Variable v1("v1");
  EXPECT_EQ(v1.get_type(), Variable::Type::CONTINUOUS);

  // When a type is specified, it should be correctly assigned.
  const Variable v2("v2", Variable::Type::CONTINUOUS);
  const Variable v3("v3", Variable::Type::INTEGER);
  const Variable v4("v4", Variable::Type::BINARY);
  const Variable v5("v5", Variable::Type::BOOLEAN);
  EXPECT_EQ(v2.get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(v3.get_type(), Variable::Type::INTEGER);
  EXPECT_EQ(v4.get_type(), Variable::Type::BINARY);
  EXPECT_EQ(v5.get_type(), Variable::Type::BOOLEAN);

  // Dummy variable gets CONTINUOUS type.
  EXPECT_TRUE(Variable{}.get_type() == Variable::Type::CONTINUOUS);

  // Variables are identified by their IDs. Names and types are not considered
  // in the identification process.
  const Variable v_continuous("v", Variable::Type::CONTINUOUS);
  const Variable v_int("v", Variable::Type::INTEGER);
  EXPECT_FALSE(v_continuous.equal_to(v_int));
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
