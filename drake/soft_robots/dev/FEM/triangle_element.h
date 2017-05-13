#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/soft_robots/dev/FEM/isoparametric_element.h"
#include "drake/soft_robots/dev/FEM/triangle_shape_function.h"

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
template <typename T>
class TriangleElement3D : public IsoparametricElement<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TriangleElement3D);

  TriangleElement3D() {}

  int get_num_nodes() const override { return nen_; }

  int get_num_spatial_dimensions() const final { return nsd_;}

  int get_num_reference_dimensions() const final { return nrd_;}

  /// Computes the area vector oriented according to the order of the nodes `xa`
  /// following to the right-hand rule. The area vector points in the direction
  /// of the normal and its magnitude is the area of the triangle.
  /// @param[in] xa Triangle nodes ordered according to the right hand side rule
  /// around the normal vector.
  Vector3<T> CalcAreaVector(const Eigen::Ref<const MatrixX<T>>& xa) const {
    const Vector3<T> u = xa.col(1) - xa.col(0);
    const Vector3<T> v = xa.col(2) - xa.col(0);
    return 0.5 * u.cross(v);
  }

  /// Computes the area of this triangle element. The result is always positive.
  T CalcArea(const Eigen::Ref<const MatrixX<T>>& xa) const {
    return CalcAreaVector(xa).norm();
  }

 protected:
  void DoCalcShapeFunctions(
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<MatrixX<T>> Na) const final {
    shape_function_.EvaluateAt(x_ref, Na);
  }

  /// In general a function of x_ref, but constant for the linear triangle
  /// element.
  void DoCalcJacobianNorm(
      const Eigen::Ref<const MatrixX<T>>& xa,
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<VectorX<T>> Jnorm) const final {
    Jnorm.setConstant(internal::triangle_area<T, nsd_>::compute(xa));
  }
 private:
  // Number of Element nodes.
  static constexpr int nen_{3};
  // Number of Spatial Dimensions.
  static constexpr int nsd_{3};
  // Number of Reference Dimensions.
  static constexpr int nrd_{2};
  TriangleShapeFunction<T> shape_function_;
};

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake
