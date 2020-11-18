#include <array>       // std::array
#include <functional>  // std::function
#include <limits>      // std::numeric_limits
#include <map>         // std::map
#include <memory>      // std::shared_ptr
#include <stdexcept>   // std::runtime_error
#include <utility>     // std::pair
#include <vector>      // std::vector

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/polynomial.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/test/mathematical_program_test_util.h"
#include "drake/solvers/test/optimization_examples.h"
#include "drake/solvers/test/second_order_cone_program_examples.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/choose_best_solver.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::solvers::internal::VecIn;
using drake::solvers::internal::VecOut;

using std::numeric_limits;

namespace drake {
namespace solvers {
namespace test {
void RunNonlinearProgram(const MathematicalProgram& prog,
                         const std::optional<Eigen::VectorXd>& x_init,
                         std::function<void(void)> test_func,
                         MathematicalProgramResult* result) {
  IpoptSolver ipopt_solver;
  NloptSolver nlopt_solver;
  SnoptSolver snopt_solver;

  //std::pair<const char*, SolverInterface*> solvers[] = {
    //  std::make_pair("SNOPT", &snopt_solver),
     // std::make_pair("NLopt", &nlopt_solver),
      //std::make_pair("Ipopt", &ipopt_solver)};
  std::pair<const char*, SolverInterface*> solvers[] = {
   std::make_pair("Ipopt", &ipopt_solver),
   std::make_pair("NLopt", &nlopt_solver)};

  for (const auto& solver : solvers) {
    SCOPED_TRACE(fmt::format("Using solver: {}", solver.first));
    PRINT_VAR(solver.first);
    if (!solver.second->available()) {
      continue;
    }
    DRAKE_EXPECT_NO_THROW(solver.second->Solve(prog, x_init, {}, result));
    EXPECT_TRUE(result->is_success());
    if (result->is_success()) {
      DRAKE_EXPECT_NO_THROW(test_func());
    }
  }
}

GTEST_TEST(testNonlinearProgram, MinimalDistanceFromSphereProblem) {
  bool with_linear_cost = true;
  MinimalDistanceFromSphereProblem<3> dut(
      Eigen::Vector3d(0, 1, 1), Eigen::Vector3d(0, 0, 0), 1, with_linear_cost);
  MathematicalProgram& prog = *dut.get_mutable_prog();

  // N.B. Some solves do not like guess x = 0.
  // NLOPT for instance fails, i.e. result.is_success() is false.
  // Ipopt however crashes and there is no even an exception.
  const Eigen::VectorXd x_init = Eigen::Vector3d(0.0, 0.0, 0.0);

  PRINT_VAR(dut.continuous_variables());
  PRINT_VAR(prog.decision_variables());
  PRINT_VAR(&dut.continuous_variables());
  PRINT_VAR(&prog.decision_variables());

  // Lets print the "best solver" choice before it gets used.
  EXPECT_NO_THROW(ChooseBestSolver(prog));
  const SolverId solver_id = ChooseBestSolver(prog);
  PRINT_VAR(solver_id.name());

  // Level 5 is the default for Ipopt.
  prog.SetSolverOption(IpoptSolver::id(), "print_level", 5);
  prog.SetSolverOption(IpoptSolver::id(), "print_user_options", "yes");
  PRINT_VAR(prog.solver_options());

  // Automatically selected solver.
  MathematicalProgramResult result;
  // Thus far the automatically selected solver is IPOTP and this solver crashes
  // with no simple backtrace.
#if 0  
  result = Solve(prog, x_init);
  const auto& x_sol = result.GetSolution(prog.decision_variables());  
  PRINT_VAR(x_sol.transpose());
  PRINT_VAR(result.is_success());
  PRINT_VAR(result.get_optimal_cost());
  PRINT_VAR(result.get_solver_id().name());
  //PRINT_VAR(result.get_solver_details().status);
  //PRINT_VAR(result.get_solver_details().ConvertStatusToString());
#endif

  // This runs specific solvers.
  RunNonlinearProgram(
      prog, x_init,
      [&]() {
        PRINT_VAR(result.is_success());
        PRINT_VAR(result.get_solution_result());
        PRINT_VAR(result.GetSolution(prog.decision_variables()).transpose());
        PRINT_VAR(result.GetSolution().transpose());
        PRINT_VAR(result.get_x_val().transpose());
        std::vector<std::string> infeasible_names =
            result.GetInfeasibleConstraintNames(prog, 1.0e-14);
        PRINT_VAR(infeasible_names.size());
        for (const auto& name : infeasible_names) {
            PRINT_VAR(name);
        }
        PRINT_VAR(result.get_optimal_cost());
        PRINT_VAR(result.get_solver_id().name());

        // N.B. We can always save our bindings and use them later. Here we dont
        // have them and that's why I request them.
        const std::vector<Binding<LorentzConeConstraint>>&
            all_cone_constraints = prog.lorentz_cone_constraints();
        PRINT_VAR(all_cone_constraints.size());
        for (const auto& c : all_cone_constraints) {
          PRINT_VAR(c);
          const VectorXd xc_sol = result.GetSolution(c.variables());
          VectorXd c_value(1);
          c.evaluator()->Eval(xc_sol, &c_value);
          PRINT_VAR(c_value);

          // Apparently Nlopt does not report the dual.  
          if (result.get_solver_id() != NloptSolver::id()) {
            VectorXd x_dual = result.GetDualSolution(c);
            PRINT_VAR(x_dual);
          } else {
            std::cout << "Not reporting dual for solver "
                      << result.get_solver_id() << std::endl;
          }
        }

        if (result.get_solver_id() == IpoptSolver::id()) {
          const IpoptSolverDetails& details =
              result.get_solver_details<IpoptSolver>();
          PRINT_VAR(details.ConvertStatusToString());
          PRINT_VAR(details.z_L.transpose());
          PRINT_VAR(details.z_U.transpose());
          PRINT_VAR(details.g.transpose());
          PRINT_VAR(details.lambda.transpose());
        }

        if (result.get_solver_id() == NloptSolver::id()) {
          const NloptSolverDetails& details =
              result.get_solver_details<NloptSolver>();
          // N.B. see
          // https://nlopt.readthedocs.io/en/latest/NLopt_Reference#return-values.
          std::cout << "Nlopt details only include status code: "
                    << details.status << std::endl;
        }

#if 0
        EXPECT_TRUE(CompareMatrices(
            prog.EvalBinding(prog.quadratic_costs().front(),
                             result.GetSolution(prog.decision_variables())),
            0.5 * x_value.transpose() * Q_symmetric * x_value +
                b.transpose() * x_value,
            1E-14, MatrixCompareType::absolute));
#endif
      },
      &result);
}

#if 0
GTEST_TEST(testNonlinearProgram, QuadraticCost) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();

  Vector4d Qdiag(1.0, 2.0, 3.0, 4.0);
  Matrix4d Q = Qdiag.asDiagonal();
  Q(1, 2) = 0.1;
  Q(2, 3) = -0.02;

  Vector4d b(1.0, -0.5, 1.3, 2.5);
  prog.AddQuadraticCost(Q, b, x);

  Matrix4d Q_transpose = Q;
  Q_transpose.transposeInPlace();
  Matrix4d Q_symmetric = 0.5 * (Q + Q_transpose);
  Vector4d expected = -Q_symmetric.ldlt().solve(b);
  const Eigen::VectorXd x_init = Eigen::Vector4d::Zero();
  MathematicalProgramResult result;
  RunNonlinearProgram(
      prog, x_init,
      [&]() {
        const auto& x_value = result.GetSolution(x);
        EXPECT_TRUE(CompareMatrices(x_value, expected, 1e-6,
                                    MatrixCompareType::absolute));
        EXPECT_TRUE(CompareMatrices(
            prog.EvalBinding(prog.quadratic_costs().front(),
                             result.GetSolution(prog.decision_variables())),
            0.5 * x_value.transpose() * Q_symmetric * x_value +
                b.transpose() * x_value,
            1E-14, MatrixCompareType::absolute));
      },
      &result);
}
#endif

}  // namespace test
}  // namespace solvers
}  // namespace drake