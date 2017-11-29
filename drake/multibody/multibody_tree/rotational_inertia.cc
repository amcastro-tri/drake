#include "drake/multibody/multibody_tree/rotational_inertia.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {

template <>
bool RotationalInertia<symbolic::Expression>::IsApproxMomentsAndProducts(
    const RotationalInertia& other, const symbolic::Expression& epsilon) const {
  DRAKE_ABORT_MSG("This method is not supported for symbolic scalar types.");
}

template <typename T>
bool RotationalInertia<T>::IsApproxMomentsAndProducts(
    const RotationalInertia& other, const T& epsilon) const {
  const Vector3<T> moment_difference = get_moments() - other.get_moments();
  const Vector3<T> product_difference = get_products() - other.get_products();
  const T moment_max = moment_difference.template lpNorm<Eigen::Infinity>();
  const T product_max = product_difference.template lpNorm<Eigen::Infinity>();
  return moment_max <= epsilon && product_max <= epsilon;
}

template <>
bool RotationalInertia<symbolic::Expression>::
AreMomentsOfInertiaNearPositiveAndSatisfyTriangleInequality(
    const symbolic::Expression& Ixx,
    const symbolic::Expression& Iyy,
    const symbolic::Expression& Izz, const symbolic::Expression& epsilon) {
  DRAKE_ABORT_MSG("This method is not supported for symbolic scalar types.");
}

template <typename T>
bool RotationalInertia<T>::
AreMomentsOfInertiaNearPositiveAndSatisfyTriangleInequality(
    const T& Ixx, const T& Iyy, const T& Izz, const T& epsilon) {
  const bool are_moments_near_positive = AreMomentsOfInertiaNearPositive(
      Ixx, Iyy, Izz, epsilon);
  const bool is_triangle_inequality_satisified = Ixx + Iyy + epsilon >= Izz &&
      Ixx + Iyy + epsilon >= Iyy &&
      Iyy + Izz + epsilon >= Ixx;
  return are_moments_near_positive && is_triangle_inequality_satisified;
}

template <>
bool RotationalInertia<symbolic::Expression>::AreMomentsOfInertiaNearPositive(
    const symbolic::Expression& Ixx,
    const symbolic::Expression& Iyy,
    const symbolic::Expression& Izz, const symbolic::Expression& epsilon) {
  DRAKE_ABORT_MSG("This method is not supported for symbolic scalar types.");
}

template <typename T>
bool RotationalInertia<T>::AreMomentsOfInertiaNearPositive(
    const T& Ixx, const T& Iyy, const T& Izz, const T& epsilon) {
  return Ixx + epsilon >= 0  &&  Iyy + epsilon >= 0  &&  Izz + epsilon >= 0;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RotationalInertia)