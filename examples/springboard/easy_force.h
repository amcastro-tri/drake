#pragma once

#include <memory>

#include "drake/math/rigid_transform.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {

using Eigen::Vector3d;
using multibody::ExternallyAppliedSpatialForce;

class EasyForce : public systems::LeafSystem<double> {
public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EasyForce)
    explicit EasyForce(
        const multibody::BodyIndex body_index,
        const Vector3d p_BoF_B,
        const multibody::SpatialForce<double> force);

        const systems::OutputPort<double>& get_externally_applied_spatial_force_output_port() const;

private:

    void SendForce(
        const systems::Context<double>& context,
        std::vector<ExternallyAppliedSpatialForce<double>>* output) const;

    int spatial_force_output_port_{-1};

    const multibody::BodyIndex body_index_;
    const Vector3d p_BoF_B_;
    const multibody::SpatialForce<double> force_;
};

}  // namespace examples
}  // namespace drake
