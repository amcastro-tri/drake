#include "drake/soft_robots/dev/FEM/triangle_element.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {

template class TriangleElement<double, 2>;
template class TriangleElement<double, 3>;
template class TriangleElement<AutoDiffXd, 2>;
template class TriangleElement<AutoDiffXd, 3>;

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake
