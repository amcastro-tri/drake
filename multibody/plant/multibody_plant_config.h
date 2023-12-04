#pragma once

#include <string>

#include "drake/common/name_value.h"

namespace drake {
namespace multibody {

/// The set of configurable properties on a MultibodyPlant.
///
/// The field names and defaults here match MultibodyPlant's defaults exactly,
/// with the exception of time_step.
struct MultibodyPlantConfig {
  /// Passes this object to an Archive.
  /// Refer to @ref yaml_serialization "YAML Serialization" for background.
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(time_step));
    a->Visit(DRAKE_NVP(penetration_allowance));
    a->Visit(DRAKE_NVP(stiction_tolerance));
    a->Visit(DRAKE_NVP(contact_model));
    a->Visit(DRAKE_NVP(discrete_contact_model));
    a->Visit(DRAKE_NVP(discrete_contact_solver));
    a->Visit(DRAKE_NVP(sap_near_rigid_threshold));
    a->Visit(DRAKE_NVP(contact_surface_representation));
    a->Visit(DRAKE_NVP(adjacent_bodies_collision_filters));
    a->Visit(DRAKE_NVP(sdf_max_distance));
    a->Visit(DRAKE_NVP(sap_sigma));
    a->Visit(DRAKE_NVP(margin));
  }

  /// Configures the MultibodyPlant::MultibodyPlant() constructor time_step.
  ///
  /// There is no default value for this within MultibodyPlant itself, so here
  /// we choose a nominal value (a discrete system, with a 1ms periodic update)
  /// as a reasonably conservative estimate that works in many cases.
  double time_step{0.001};

  /// Configures the MultibodyPlant::set_penetration_allowance().
  double penetration_allowance{0.001};

  /// Configures the MultibodyPlant::set_stiction_tolerance().
  double stiction_tolerance{0.001};

  /// Configures the MultibodyPlant::set_contact_model().
  /// Refer to drake::multibody::ContactModel for details.
  /// Valid strings are:
  /// - "point"
  /// - "hydroelastic"
  /// - "hydroelastic_with_fallback"
  std::string contact_model{"hydroelastic_with_fallback"};

  // TODO(amcastro-tri): Deprecate. Use discrete_contact_model instead.
  /// Configures the MultibodyPlant::set_discrete_contact_solver().
  /// Refer to drake::multibody::DiscreteContactSolver for details.
  /// Valid strings are:
  /// - "tamsi", uses the TAMSI model.
  /// - "sap"  , uses the SAP model.
  /// @warning This option is ignored if discrete_contact_model not empty.
  std::string discrete_contact_solver{"tamsi"};

  /// Configures the MultibodyPlant::set_discrete_contact_model().
  /// Refer to drake::multibody::DiscreteContactModel for details.
  /// Valid strings are:
  /// - "tamsi",  uses TAMSI solver.
  /// - "sap",    uses SAP solver.
  /// - "convex", uses SAP solver.
  /// - "lagged", uses SAP solver.
  /// - "" (ignored, discrete_contact_solver is respected)
  /// N.B. If non-empty, this options determines the discrete solver that gets
  /// used.
  std::string discrete_contact_model{""};

  // TODO(amcastro-tri): Change default to zero, or simply eliminate.
  /// Non-negative dimensionless number typically in the range [0.0, 1.0],
  /// though larger values are allowed even if uncommon. This parameter controls
  /// the "near rigid" regime of the SAP solver, β in section V.B of [Castro et
  /// al., 2021]. It essentially controls a threshold value for the maximum
  /// amount of stiffness SAP can handle robustly. Beyond this value, stiffness
  /// saturates as explained in [Castro et al., 2021].
  /// A value of 1.0 is a conservative choice to avoid numerical
  /// ill-conditioning. However, this might introduce artificial softening of
  /// the contact constraints. If this is your case try:
  ///   1. Set this parameter to zero.
  ///   2. For difficult problems (hundreds of contacts for instance), you might
  ///      need to use a low value if the solver fails to converge.
  ///      For instance, set values in the range (1e-3, 1e-2).
  double sap_near_rigid_threshold{1.0};

  /// If lower or equal than zero, point contact uses a penetration query, that
  /// only detects collision when objects already are overlapping.
  /// If positive, the point contact model will use a signed distance query and
  /// objects farther than this distance won't be considered.
  double sdf_max_distance{0.0};

  // Controls the amount of SAP friction regularization in the Hunt&Crossley
  // model.
  double sap_sigma{0.0};

  // Models compliant contact with a geometric margin, in meters.
  double margin{0.0};

  /// Configures the MultibodyPlant::set_contact_surface_representation().
  /// Refer to drake::geometry::HydroelasticContactRepresentation for details.
  /// Valid strings are:
  /// - "triangle"
  /// - "polygon"
  ///
  /// The default value used here is consistent with the default time_step
  /// chosen above; keep this consistent with
  /// MultibodyPlant::GetDefaultContactSurfaceRepresentation().
  std::string contact_surface_representation{"polygon"};

  /// Configures the MultibodyPlant::set_adjacent_bodies_collision_filters().
  bool adjacent_bodies_collision_filters{true};
};

}  // namespace multibody
}  // namespace drake
