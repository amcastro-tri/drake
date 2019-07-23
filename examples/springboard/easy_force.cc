#include "drake/examples/springboard/easy_force.h"

namespace drake {
namespace examples {

EasyForce::EasyForce(
    const multibody::BodyIndex body_index,
    const Vector3d p_BoF_B,
    const multibody::SpatialForce<double> force
) : body_index_(body_index),
    p_BoF_B_(p_BoF_B),
    force_(force) {

    spatial_force_output_port_ = this->DeclareAbstractOutputPort(
        "easy_force_message",
        &EasyForce::SendForce).get_index();
}

void EasyForce::SendForce(
    const systems::Context<double>& context,
    std::vector<ExternallyAppliedSpatialForce<double>>* output) const {

    std::cout << context.num_total_states();
    std::cout << "\ninside EasyForce!!\n";

    output->resize(1);
    (*output)[0].body_index = body_index_;
    (*output)[0].p_BoBq_B = p_BoF_B_;
    (*output)[0].F_Bq_W = force_;
}

const systems::OutputPort<double> &EasyForce::get_externally_applied_spatial_force_output_port() const {
    return systems::System<double>::get_output_port(spatial_force_output_port_);
}

}  // namespace examples
}  // namespace drake
