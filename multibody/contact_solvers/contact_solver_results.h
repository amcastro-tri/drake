#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// This struct stores the results from a computation performed with
// a discrete contact solver. For instance, we can store the results from
// TamsiSolver in this class.
// We denote `nv` the size of the vector of generalized velocities and `nc` the
// number of contact points.
// TODO(amcastro-tri): consider renaming this to ContactSolverResults, since we
// will store not only contact constraint forces but also other constraint
// forces in general.
template <class T>
struct ContactSolverResults {
  // Resizes `this` object to store the contact results for a problem with
  // `num_velocities` and `num_contacts`.
  void Resize(int num_positions, int num_velocities, int num_contacts) {
    q_next.resize(num_positions);
    v_next.resize(num_velocities);
    fn.resize(num_contacts);
    ft.resize(2 * num_contacts);
    vn.resize(num_contacts);
    vt.resize(2 * num_contacts);
    tau_contact.resize(num_velocities);
  }

  // Vector of configurations at the next time step.
  VectorX<T> q_next;

  // Vector of generalized velocities at the next time step.
  VectorX<T> v_next;

  // TODO(amcastro-tri): store state for deformables.

  // Vector storing the normal force (positive) at each contact point, of size
  // `nc`.
  VectorX<T> fn;

  // Vector of tangential velocities in the contact frame, of size `2⋅nc`.
  VectorX<T> ft;

  // Vector of normal separation speeds, of size `nc`.
  VectorX<T> vn;

  // Vector of tangential relative velocities, of size `2⋅nc`.
  VectorX<T> vt;

  // Vector of generalized forces due to contact, of size `nv`.
  VectorX<T> tau_contact;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::contact_solvers::internal::ContactSolverResults)
