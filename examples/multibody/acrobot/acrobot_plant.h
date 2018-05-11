#pragma once

#include <memory>

#include "drake/examples/acrobot/gen/acrobot_input.h"
#include "drake/examples/acrobot/gen/acrobot_params.h"
#include "drake/examples/acrobot/gen/acrobot_state.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace examples {
namespace multibody {
namespace acrobot {

namespace detail {
template <typename T>
class AcrobotInputToMBPInput : public systems::LeafSystem<T>  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AcrobotInputToMBPInput)
  using AcrobotStateIndices = drake::examples::acrobot::AcrobotStateIndices;
  template<typename Scalar>
  using AcrobotInput = drake::examples::acrobot::AcrobotInput<Scalar>;

  AcrobotInputToMBPInput() :
      systems::LeafSystem<T>(systems::SystemTypeTag<
          ::drake::examples::multibody::acrobot::detail::AcrobotInputToMBPInput>{}) {
    this->DeclareVectorInputPort(AcrobotInput<T>());
    this->DeclareVectorOutputPort(
        systems::BasicVector<T>(AcrobotStateIndices::kNumCoordinates),
        [this](const systems::Context<T>& context,
               systems::BasicVector<T>* vector) {
          vector->SetFrom(*this->template EvalVectorInput<AcrobotInput>(context, 0));
    });
  }
  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit AcrobotInputToMBPInput(const AcrobotInputToMBPInput<U>&) :
      AcrobotInputToMBPInput() {}
};

// Helpers to scalar convert a Diagram.
template <typename T>
struct type_tag;

std::unique_ptr<systems::System<double>> ScalarConvertDiagram(
    std::unique_ptr<systems::Diagram<double>> from, type_tag<double>) {
  return from;
}

std::unique_ptr<systems::System<AutoDiffXd>> ScalarConvertDiagram(
    std::unique_ptr<systems::Diagram<double>> from, type_tag<AutoDiffXd>) {
  return from->ToAutoDiffXd();
}

}

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template <typename T>
class AcrobotPlant : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AcrobotPlant)

  template<typename Scalar>
  using AcrobotState = drake::examples::acrobot::AcrobotState<Scalar>;

  template<typename Scalar>
  using AcrobotInput = drake::examples::acrobot::AcrobotInput<Scalar>;

  template<typename Scalar>
  using RevoluteJoint = drake::multibody::RevoluteJoint<Scalar>;

  template<typename Scalar>
  using MultibodyPlant =
  drake::multibody::multibody_plant::MultibodyPlant<Scalar>;

  /// Constructs the plant.  The parameters of the system are stored as
  /// Parameters in the Context (see acrobot_params.named_vector).
  //AcrobotPlant(geometry::SceneGraph<T>* scene_graph = nullptr);
  AcrobotPlant();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit AcrobotPlant(const AcrobotPlant<U>&);

  /// Evaluates the input port and returns the scalar value
  /// of the commanded torque.
  const T& get_tau(const systems::Context<T>& context) const {
    return this->template EvalVectorInput<AcrobotInput>(context, 0)->tau();
  }

  const RevoluteJoint<T>& shoulder() const { return *shoulder_; }
  const RevoluteJoint<T>& elbow() const { return *elbow_; }

  const MultibodyPlant<T>& plant() const { return *plant_; }

  const systems::OutputPort<T>& get_geometry_poses_output_port() const {
    return plant().get_geometry_poses_output_port();
  }

#if 0
  static const AcrobotState<T>& CopyState(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const AcrobotState<T>&>(cstate.get_vector());
  }

  static const AcrobotState<T>& get_state(const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  static AcrobotState<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<AcrobotState<T>&>(cstate->get_mutable_vector());
  }

  static AcrobotState<T>& get_mutable_state(systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  const AcrobotParams<T>& get_parameters(
      const systems::Context<T>& context) const {
    return this->template GetNumericParameter<AcrobotParams>(context, 0);
  }
#endif
 protected:
  //T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
  //T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

 private:
  geometry::SceneGraph<T>* scene_graph_{nullptr};
  const MultibodyPlant<T>* plant_{nullptr};
  const RevoluteJoint<T>* shoulder_{nullptr};
  const RevoluteJoint<T>* elbow_{nullptr};
};

#if 0
/// Constructs the LQR controller for stabilizing the upright fixed point using
/// default LQR cost matrices which have been tested for this system.
std::unique_ptr<systems::AffineSystem<double>> BalancingLQRController(
    const AcrobotPlant<double>& acrobot);
#endif

}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake
