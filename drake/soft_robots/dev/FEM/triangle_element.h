#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/soft_robots/dev/FEM/isoparametric_element.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {

namespace internal {
template <typename T, int dim>
struct triangle_area;

// Specialization to 2D
template <typename T>
struct triangle_area<T, 2> {
  // If points in xa are in counterclockwise order, are will be positive.
  // If in clockwise order, are will be negative.
  static T compute(const Eigen::Ref<const MatrixX<T>>& xa) {
    return 0.5 * (
        xa(0,0) * ( xa(1,1) - xa(1,2) ) +
            xa(0,1) * ( xa(1,2) - xa(1,0) ) +
            xa(0,2) * ( xa(1,0) - xa(1,1) ) );
  }
};

// Specialization to 3D
template <typename T>
struct triangle_area<T, 3> {
  // If points in xa are ordered following the right-hand rule.
  // The returned area is always positive.
  static T compute(const Eigen::Ref<const MatrixX<T>>& xa) {
    const Vector3<T> u = xa.col(1) - xa.col(0);
    const Vector3<T> v = xa.col(2) - xa.col(0);
    return 0.5 * u.cross(v).norm();
  }
};
}  // namespace internal

/// @tparam T Must be a Scalar compatible with Eigen.
/// @tparam npd The number of physical directions. Either 2 or 3 only.
template <typename T, int npd>
class TriangleElement : public IsoparametricElement<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TriangleElement);

  TriangleElement() {}

  int get_num_nodes() const override { return num_element_nodes_; }

  int get_num_physical_dimensions() const final { return npd;}

  int get_num_reference_dimensions() const final { return 2;}

 protected:
  void DoCalcShapeFunctions(
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<MatrixX<T>> Na) const final {
    // a, as usual, is the element node index.
    // N(a = 0) = t = 1 - r - s
    // N(a = 1) = r
    // N(a = 2) = s
    // The code below fails at runtime. Why?
    //Na.template topRows<1>(0) = MatrixX<T>::Ones(1, x_ref.cols());
    //Na.template topRows<1>(0) -= (x_ref.row(0) + x_ref.row(1));
    //Na.template bottomRows<2>(1) = x_ref;

    for (int ipoint = 0; ipoint < x_ref.cols(); ++ipoint) {
      Na(0, ipoint) = 1.0 - x_ref(0, ipoint) - x_ref(1, ipoint);
      Na(1, ipoint) = x_ref(0, ipoint);
      Na(2, ipoint) = x_ref(1, ipoint);
    }
  }

  /// In general a function of x_ref, but constant for the linear triangle
  /// element.
  void DoCalcJacobianNorm(
      const Eigen::Ref<const MatrixX<T>>& xa,
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<VectorX<T>> Jnorm) const final {
    Jnorm.setConstant(internal::triangle_area<T, npd>::compute(xa));
  }
 private:
  static constexpr int num_element_nodes_{3};
};

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake
