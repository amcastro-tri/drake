#pragma once

#include <string>

#include "drake/geometry/query_results/contact_surface.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {

/// Adds a new MultibodyPlant and SceneGraph to the given `builder`.  The
/// plant's settings such as `time_step` are set using the given `config`.
AddMultibodyPlantSceneGraphResult<double> AddMultibodyPlant(
    const MultibodyPlantConfig& config,
    systems::DiagramBuilder<double>* builder);

}  // namespace multibody
}  // namespace drake
