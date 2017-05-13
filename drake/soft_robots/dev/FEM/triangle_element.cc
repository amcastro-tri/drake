#include "drake/soft_robots/dev/FEM/triangle_element.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {

template class TriangleElement3D<double>;
template class TriangleElement3D<AutoDiffXd>;

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake
