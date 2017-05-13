#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {

template <typename T>
class ShapeFunction {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ShapeFunction);
  ShapeFunction() {}
  virtual ~ShapeFunction() {}

  virtual int get_reference_coordinates_dimension() const = 0;

  virtual int get_num_nodes() const = 0;

  /// @param[in] x_ref A set of points in reference element coordinates.
  /// x_ref must have get_num_reference_dimensions() rows and an arbitrary number
  /// of columns. Typically the number of colums equals the number of quadrature
  /// points used for numerical integration within element routines.
  /// @param[out] Na Shape functions at each element node evaluated at points
  /// x_ref expressed in reference coordinates.
  /// Na must have get_num_nodes() rows and as many columns as x_ref.
  void EvaluateAt(
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<MatrixX<T>> Na) const {
    DRAKE_ASSERT(x_ref.rows() == get_reference_coordinates_dimension());
    DRAKE_ASSERT(Na.rows() == get_num_nodes());
    DRAKE_ASSERT(Na.cols() == x_ref.cols());
    DoEvaluateAt(x_ref, Na);
  }

 protected:
  virtual void DoEvaluateAt(
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<MatrixX<T>> Na) const = 0;
};

/// @tparam T Must be a Scalar compatible with Eigen.
/// @tparam npd The number of physical directions. Either 2 or 3 only.
template <typename T>
class TriangleShapeFunction : public ShapeFunction<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TriangleShapeFunction);

  TriangleShapeFunction() {}

  int get_reference_coordinates_dimension() const final { return 2;}

  int get_num_nodes() const final { return num_element_nodes_;};

 protected:
  void DoEvaluateAt(
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

 private:
  static constexpr int num_element_nodes_{3};
};

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake
