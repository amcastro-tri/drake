#pragma once

#include <limits>
#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_system.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {

/// This method makes a MultibodyPlant model of the Acrobot - a canonical
/// underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
///
/// @param[in] default_parameters
///   Default parameters of the model set at construction. These parameters
///   include masses, link lengths, rotational inertias, etc. Refer to the
///   documentation of AcrobotParameters for further details.
/// @param[in] finalize
///   If `true`, MultibodyPlant::Finalize() gets called on the new plant.
/// @param geometry_system
///   If a GeometrySystem is provided with this argument, this factory method
///   will register the new multibody plant to be a source for that geometry
///   system and it will also register geometry for visualization.
///   If this argument is omitted, no geometry will be registered.
std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeKukaIiwaPlant(bool finalize,
                  geometry::GeometrySystem<double>* geometry_system = nullptr);

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
