#include "drake/soft_robots/dev/FEM/triangle_shape_function.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {

template class TriangleShapeFunction<double>;
template class TriangleShapeFunction<AutoDiffXd>;

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake
