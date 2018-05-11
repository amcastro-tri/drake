#include "drake/examples/multibody/acrobot/acrobot_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rotary_encoders.h"

using std::sin;
using std::cos;

namespace drake {
namespace examples {
namespace multibody {
namespace acrobot {

using drake::geometry::SceneGraph;
using drake::multibody::benchmarks::acrobot::AcrobotParameters;
using drake::multibody::benchmarks::acrobot::MakeAcrobotPlant;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RevoluteJoint;

template <typename T>
AcrobotPlant<T>::AcrobotPlant() {

  systems::DiagramBuilder<T> builder;

  scene_graph_ = builder.template AddSystem<SceneGraph>();

  const AcrobotParameters acrobot_parameters;
  auto plant_on_double = MakeAcrobotPlant(
      acrobot_parameters, true /* Finalize the plant */, scene_graph_);

  // Use scalar conversion constructor to add a plant templated on T.
  const MultibodyPlant<T>& acrobot =
      *builder.template AddSystem<MultibodyPlant>(plant_on_double);

  // TODO: pass acrobot as input to AcrobotInputToMBPInput so that it knows how
  // to map coordinates. Now we are assuming they are in order.
  const auto& input_converter =
      *builder.template AddSystem<detail::AcrobotInputToMBPInput>();
  builder.ExportInput(input_converter.get_input_port(0));
  builder.Connect(input_converter.get_output_port(0),
                  acrobot.get_actuation_input_port());

  builder.ExportOutput(acrobot.get_continuous_state_output_port());

  // We are done specifying the Diagram.
  builder.BuildInto(this);

  // Easy access to the model's elements.
  shoulder_ = &acrobot.template GetJointByName<RevoluteJoint>(
      acrobot_parameters.shoulder_joint_name());
  elbow_ = &acrobot.template GetJointByName<RevoluteJoint>(
      acrobot_parameters.elbow_joint_name());
}

template <typename T>
template <typename U>
AcrobotPlant<T>::AcrobotPlant(const AcrobotPlant<U>&) : AcrobotPlant<T>() {}

#if 0
template <typename T>
T AcrobotPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  return plant_->CalcKineticEnergy(GetSubsystemContext(*plant_, context));
}

template <typename T>
T AcrobotPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
  return plant_->CalcPotentialEnergy(GetSubsystemContext(*plant_, context));
}
#endif

#if 0
std::unique_ptr<systems::AffineSystem<double>> BalancingLQRController(
    const AcrobotPlant<double>& acrobot) {
  auto context = acrobot.CreateDefaultContext();

  // Set nominal torque to zero.
  context->FixInputPort(0, Vector1d::Constant(0.0));

  // Set nominal state to the upright fixed point.
  AcrobotState<double>* x = dynamic_cast<AcrobotState<double>*>(
      &context->get_mutable_continuous_state_vector());
  DRAKE_ASSERT(x != nullptr);
  x->set_theta1(M_PI);
  x->set_theta2(0.0);
  x->set_theta1dot(0.0);
  x->set_theta2dot(0.0);

  // Setup LQR Cost matrices (penalize position error 10x more than velocity
  // to roughly address difference in units, using sqrt(g/l) as the time
  // constant.
  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  Q(0, 0) = 10;
  Q(1, 1) = 10;
  Vector1d R = Vector1d::Constant(1);

  return systems::controllers::LinearQuadraticRegulator(acrobot, *context, Q,
                                                        R);
}
#endif

}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::multibody::acrobot::AcrobotPlant)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::multibody::acrobot::AcrobotWEncoder)
