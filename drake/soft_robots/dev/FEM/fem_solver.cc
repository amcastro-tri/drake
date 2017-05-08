#include "drake/soft_robots/dev/FEM/fem_solver.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {

template class FEMSolver<double>;

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake
