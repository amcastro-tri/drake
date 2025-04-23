#pragma once

#include <array>
#include <vector>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

// TODO(amcastro-tri): Consider specialized struct sizes for the most common
// constraints. E.g.:
//   - OneCliqueConstraintSizes
//   - PatchConstraintSizes
//
// Being patch constraint the most common, probably that's the only one worth
// specializing.

// Sores the structure for a pool of constraints involving at most two cliques.
struct TwoCliquesConstraintSizes {
  int num_constraints;

  // The number of constraint equations. E.g., for patch constraints, 3 times
  // the number of patch constraints, i.e. 3 times num_constraints.
  std::vector<int> num_equations;

  // Number of cliques, either one or two, for the k-th patch, of size
  // num_patches.
  std::vector<int> num_cliques;

  // Cliques involved in the k-th each constraint. At most two.
  std::vector<std::array<int, 2>> cliques;

  // Number of velocities for each clique involved. At most two.
  std::vector<std::array<int, 2>> cliques_nv;

  // Total number of velocities involved in the k-th constraint, the sum of
  // cliques_nv.
  std::vector<int> velocities;
};

struct ModelSizes {
  int num_cliques;
  int num_velocities;  // Total number of velocities.
  // Total number of constraints, including all constraint types.
  int num_constraints;

  // Number of velocities for the c-th clique, of size num_cliques.
  std::vector<int> cliques_nv;

  TwoCliquesConstraintSizes pd_constraints;
  TwoCliquesConstraintSizes limit_constraints;

  // Constraint sizes for the pool of patch constraints.
  // There are patch_constraints.num_constraints patches, with
  // (patch_constraints.num_equations[k] / 3) contact pairs for the k-th patch.
  TwoCliquesConstraintSizes patch_constraints;
};

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
