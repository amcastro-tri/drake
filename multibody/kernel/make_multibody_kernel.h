#pragma once

#include <string>
#include <utility>

#include "drake/multibody/kernel/multibody_kernel.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/topology/graph.h"

namespace drake {
namespace multibody {
namespace internal {

LinkJointGraph MakeLinkJointGraph(const MultibodyPlant<double>& plant);

std::pair<MultibodyKernel<double>, MultibodyKernelParameters<double>>
MakeMultibodyKernel(const MultibodyPlant<double>& plant,
                    const LinkJointGraph& graph);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
