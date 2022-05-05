#include "drake/multibody/plant/multibody_plant_config_functions.h"

#include <array>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {

using AddResult = AddMultibodyPlantSceneGraphResult<double>;

AddResult AddMultibodyPlant(
    const MultibodyPlantConfig& config,
    systems::DiagramBuilder<double>* builder) {
  return AddMultibodyPlantSceneGraph(builder, config);
}

}  // namespace multibody
}  // namespace drake
