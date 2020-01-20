#include "drake/examples/integration_with_events/bouncing_ball_plant.h"

#include <algorithm>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace examples {
namespace scene_graph {
namespace bouncing_ball {

using Eigen::Vector4d;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::PenetrationAsPointPair;
using geometry::PerceptionProperties;
using geometry::ProximityProperties;
using geometry::render::RenderLabel;
using geometry::SceneGraph;
using geometry::SourceId;
using geometry::Sphere;
using math::RigidTransform;
using math::RigidTransformd;
using std::make_unique;
using systems::Context;
using systems::BasicVector;

template <typename T>
BouncingBallPlant<T>::BouncingBallPlant(SourceId source_id,
                                        SceneGraph<T>* scene_graph,
                                        const Vector2<double>& p_WB)
    : p_WB_(p_WB) {
  DRAKE_DEMAND(scene_graph != nullptr);
  DRAKE_DEMAND(source_id.is_valid());

  geometry_query_port_ = this->DeclareAbstractInputPort(
      systems::kUseDefaultName, Value<geometry::QueryObject<T>>{})
          .get_index();
  state_port_ =
      this->DeclareVectorOutputPort(BasicVector<T>(num_states()),
                                    &BouncingBallPlant::CopyStateToOutput,
                                    {this->all_state_ticket()})
          .get_index();

  this->DeclareContinuousState(num_positions() /* num_q */,
                               num_velocities() /* num_v */, 0 /* num_z */);

  ball_frame_id_ = scene_graph->RegisterFrame(
      source_id, GeometryFrame("ball_frame"));
  ball_id_ = scene_graph->RegisterGeometry(
      source_id, ball_frame_id_,
      make_unique<GeometryInstance>(RigidTransformd::Identity(), /*X_FG*/
                                    make_unique<Sphere>(diameter_ / 2.0),
                                    "ball"));
  // Use the default material.
  scene_graph->AssignRole(source_id, ball_id_, IllustrationProperties());
  scene_graph->AssignRole(source_id, ball_id_, ProximityProperties());
  PerceptionProperties perception_properties;
  perception_properties.AddProperty("phong", "diffuse",
                                    Vector4d{0.8, 0.8, 0.8, 1.0});
  perception_properties.AddProperty("label", "id",
                                    RenderLabel(ball_id_.get_value()));
  scene_graph->AssignRole(source_id, ball_id_, perception_properties);

  // Allocate the output port now that the frame has been registered.
  geometry_pose_port_ = this->DeclareAbstractOutputPort(
          &BouncingBallPlant::CalcFramePoseOutput,
          {this->configuration_ticket()})
      .get_index();
}

template <typename T>
BouncingBallPlant<T>::~BouncingBallPlant() {}

template <typename T>
const systems::InputPort<T>&
BouncingBallPlant<T>::get_geometry_query_input_port() const {
  return systems::System<T>::get_input_port(geometry_query_port_);
}

template <typename T>
const systems::OutputPort<T>&
BouncingBallPlant<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(state_port_);
}

template <typename T>
const systems::OutputPort<T>&
BouncingBallPlant<T>::get_geometry_pose_output_port() const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
void BouncingBallPlant<T>::CopyStateToOutput(
    const systems::Context<T>& context,
    systems::BasicVector<T>* output) const {
  // Get current state from context.
  const systems::VectorBase<T>& continuous_state_vector =
      context.get_continuous_state_vector();
  // Write system output.
  output->set_value(continuous_state_vector.CopyToVector());
}

template <typename T>
void BouncingBallPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  RigidTransform<T> pose = RigidTransform<T>::Identity();
  const auto xc =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const double z = xc[0];  
  pose.set_translation({p_WB_.x(), p_WB_.y(), z});
  *poses = {{ball_frame_id_, pose}};
}

// Compute the actual physics.
template <typename T>
void BouncingBallPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  using std::max;

  const auto xc =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  //const double z = xc[0];
  const double v = xc[1];

  const auto& query_object = get_geometry_query_input_port().
      template Eval<geometry::QueryObject<T>>(context);

  std::vector<PenetrationAsPointPair<T>> penetrations =
      query_object.ComputePointPairPenetration();
  T fC = 0;  // the contact force
  if (penetrations.size() > 0) {
    for (const auto& penetration : penetrations) {
      if (penetration.id_A == ball_id_ || penetration.id_B == ball_id_) {
        // Penetration depth, > 0 during penetration.
        const T& x = penetration.depth;
        // Penetration rate, > 0 implies increasing penetration.
        const T& xdot = -v;

        fC = k_ * x * (1.0 + d_ * xdot);
      }
    }
  }
  const T fN = max(0.0, fC);

  const double vdot = (-m_ * g_ + fN) / m_;
  VectorX<T> xcdot(num_states());
  xcdot << v, vdot;

  derivatives->SetFromVector(xcdot);
}

template class BouncingBallPlant<double>;

}  // namespace bouncing_ball
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake
