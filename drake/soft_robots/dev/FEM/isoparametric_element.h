#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {

template <typename T>
class IsoparametricElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IsoparametricElement);

  virtual ~IsoparametricElement() {}

  /// Returns the number of element nodes.
  virtual int get_num_nodes() const = 0;

  /// Returns the number of spatial directions.
  virtual int get_num_physical_dimensions() const = 0;

  /// Returns the number of reference coordiantes.
  virtual int get_num_reference_dimensions() const = 0;

  /// @param[in] xa Node coordinates. Each column in xa contains the position for
  /// the a-th element node.
  /// xa must be of size get_num_physical_dimensions() x get_num_nodes().
  /// @param[in] x_ref A set of points in reference element coordinates.
  /// x_ref must have get_num_reference_dimensions() rows and an arbitrary number
  /// of columns. Typically the number of colums equals the number of quadrature
  /// points used for numerical integration within element routines.
  /// @param[out] x_phys Points x_ref mapped to physical coordinates. x_phys
  /// must have get_num_physical_dimensions() rows and as many columns as x_ref.
  void MapCoordinatesFromReferenceToPhysical(
      const Eigen::Ref<const MatrixX<T>>& xa,
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<MatrixX<T>> x_phys) const {
#ifdef DRAKE_ASSERT_IS_ARMED
    if (xa.rows() != get_num_physical_dimensions())
      throw std::runtime_error(
          "xa does not have the correct number of physical dimensions");
    if (xa.cols() != get_num_nodes())
      throw std::runtime_error(
          "xa does not have the correct number of element nodes");
    if (x_ref.rows() != get_num_reference_dimensions())
      throw std::runtime_error(
          "x_ref does not have the correct number of reference dimensions");
    if (x_phys.rows() != get_num_physical_dimensions())
      throw std::runtime_error(
          "x_phys does not have the correct number of physical dimensions");
    if (x_phys.cols() != x_ref.cols())
      throw std::runtime_error(
          "x_phys and x_ref must have the same number of columns");
#endif
    DoMapCoordinatesFromReferenceToPhysical(xa, x_ref, x_phys);
  }

  /// @param[in] x_ref A set of points in reference element coordinates.
  /// x_ref must have get_num_reference_dimensions() rows and an arbitrary number
  /// of columns. Typically the number of colums equals the number of quadrature
  /// points used for numerical integration within element routines.
  /// @param[out] Na Shape functions at each element node evaluated at points
  /// x_ref expressed in reference coordinates.
  /// Na must have get_num_nodes() rows and as many columns as x_ref.
  void CalcShapeFunctions(
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<MatrixX<T>> Na) const {
#ifdef DRAKE_ASSERT_IS_ARMED
    if (x_ref.rows() != get_num_reference_dimensions())
      throw std::runtime_error(
          "x_ref does not have the correct number of reference dimensions");
    if (Na.rows() != get_num_nodes())
      throw std::runtime_error(
          "Na does not have the correct number of nodes");
    if (Na.cols() != x_ref.cols())
      throw std::runtime_error(
          "Na and x_ref must have the same number of columns");
#endif
    DoCalcShapeFunctions(x_ref, Na);
  }

  void CalcJacobianNorm(
      const Eigen::Ref<const MatrixX<T>>& xa,
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<VectorX<T>> Jnorm) const {
    DoCalcJacobianNorm(xa, x_ref, Jnorm);
  }

 protected:
  void DoMapCoordinatesFromReferenceToPhysical(
      const Eigen::Ref<const MatrixX<T>>& xa,
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<MatrixX<T>> x_phys) const {
    MatrixX<T> Na(get_num_nodes(), x_ref.cols());
    DoCalcShapeFunctions(x_ref, Na);
    x_phys = xa * Na;
  }

  virtual void DoCalcShapeFunctions(
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<MatrixX<T>> Na) const = 0;

  virtual void DoCalcJacobianNorm(
      const Eigen::Ref<const MatrixX<T>>& xa,
      const Eigen::Ref<const MatrixX<T>>& x_ref,
      Eigen::Ref<VectorX<T>> Jnorm) const = 0;
};

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake
