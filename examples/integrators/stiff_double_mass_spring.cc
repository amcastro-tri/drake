/// @file
///
/// This demo sets up a passive Quadrotor plant in a world described by the
/// warehouse model. The robot simply passively falls to the floor within the
/// walls of the warehouse, falling from the initial_height command line
/// argument.

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/custom_force_element.h"
#include "drake/systems/analysis/implicit_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

namespace drake {
namespace examples {
namespace hertz_contact {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrix;
using math::RotationMatrixd;
using multibody::Body;
using multibody::BodyIndex;
using multibody::CoulombFriction;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::RigidBody;
using multibody::SpatialForce;
using multibody::SpatialInertia;
using multibody::SpatialVelocity;
using multibody::UnitInertia;
using systems::Context;

const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
const Vector4<double> green(0.0, 1.0, 0.0, 1.0);
const Vector4<double> blue(0.0, 0.0, 1.0, 1.0);
const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);

DEFINE_double(duration, 3e-3, "Total duration of simulation.");
DEFINE_double(h_min, 0.0, "Minimum step we allo the integrator to take.");
DEFINE_double(viz_period, 1.0 / 60.0, "Viz period.");
DEFINE_bool(throw_on_reaching_h_min, false,
            "Whether to throw or not when h_min is reached.");
DEFINE_bool(use_full_newton, true, "Use full Newton, otherwise quasi-Newton.");
DEFINE_double(h_loose_band, 1.0e-4, "Loose band upper limit.");
DEFINE_double(a_loose_band, 0.5, "Accuracy within the loose band.");

// Adapted from:
// examples/stiff_double_mass_spring/stiff_double_mass_spring_system.h
template <class T>
class StiffDoubleMassSpringSystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StiffDoubleMassSpringSystem)
  StiffDoubleMassSpringSystem() {
    this->DeclareContinuousState(2 /* num_q */, 2 /* num_v */, 0 /* num_z */);
    this->DeclareVectorOutputPort("state", systems::BasicVector<T>(4),
                                  &StiffDoubleMassSpringSystem::CopyStateOut,
                                  {this->all_state_ticket()});
  }

  const systems::OutputPort<T>& get_state_output_port() const {
    DRAKE_DEMAND(systems::LeafSystem<T>::num_output_ports() == 1);
    return systems::LeafSystem<T>::get_output_port(0);
  }

  /// Evaluates the spring and damping forces, returning the forces on each
  /// body.
  Vector2<T> EvalSpringDamperForces(const Context<T>& context) const {
    const Eigen::Vector2d k = get_spring_constants();
    const Eigen::Vector2d b = get_damping_constants();

    const Vector2<T> x = get_position(context);
    const Vector2<T> v = get_velocity(context);

    const T stretch0 = x(0) - 0;
    const T stretch1 = x(1) - x(0) - 1;

    // Get the force from Spring 1 and Spring 2.
    const T f1 = -k(0) * stretch0 - v(0) * b(0);
    const T f2 = -k(1) * stretch1 - (v(1) - v(0)) * b(1);

    // Return the force on each body. Spring 1 acts only Body 1. Spring 2
    // acts on Body 1 and Body 2.
    return Vector2<T>(f1 - f2, f2);
  }

  /// Gets the two spring constants.
  Eigen::Vector2d get_spring_constants() const {
    return Eigen::Vector2d(750, 750 * 10000); // Should result in fs = 1000 * f0.
  }

  /// Gets the two damping constants.
  Eigen::Vector2d get_damping_constants() const {
    return Eigen::Vector2d(0, 0);
  }

  /// Gets the positions of the two point mass bodies.
  Vector2<T> get_position(const Context<T>& c) const {
    return c.get_continuous_state().get_generalized_position().CopyToVector();
  }

  /// Gets the velocity of the two point mass bodies.
  Vector2<T> get_velocity(const Context<T>& c) const {
    return c.get_continuous_state().get_generalized_velocity().CopyToVector();
  }

  /// Gets the mass for the bodies in the system.
  Eigen::Vector2d get_mass() const { return Eigen::Vector2d(1.0, 1.0); }

  void DoCalcTimeDerivatives(
      const Context<T>& context,
      systems::ContinuousState<T>* deriv) const override {
    // Get velocity.
    const systems::VectorBase<T>& xd =
        context.get_continuous_state().get_generalized_velocity();

    // Get the masses and spring and damping coefficients.
    const Vector2<T> mass = get_mass();

    // Compute the forces.
    const Vector2<T> f = EvalSpringDamperForces(context);

    // Compute the acceleration.
    const Vector2<T> a = f.array() / mass.array();

    // Set the derivatives.
    deriv->get_mutable_generalized_position().SetFrom(xd);
    deriv->get_mutable_generalized_velocity().SetFromVector(a);
  }

  /// Gets the end time for integration.
  T get_end_time() const { return 1e1; }

  /// Sets the initial conditions for the system.
  /// The first mass will be located at x1 = 0.5 and second will be located at
  /// x2 = 1.5. No initial velocity is present.
  void SetDefaultState(const Context<T>&,
                       systems::State<T>* state) const override {
    Vector2<T> x, xd;
    const Vector2<T> ks = get_spring_constants();
    x(0) = 0.5;
    x(1) = 1.5 + 0.5 * ks[0] / ks[1];
    xd.setZero();

    state->get_mutable_continuous_state()
        .get_mutable_generalized_position()
        .SetFromVector(x);
    state->get_mutable_continuous_state()
        .get_mutable_generalized_velocity()
        .SetFromVector(xd);
  }

  /// Gets the solution for the system with initial state defined at @p context,
  /// returning the solution at time @p t, in @p state. Aside from the
  /// assumption that there is zero initial stretching between the two point
  /// masses, initial conditions are arbitrary. The solution is predicated
  /// on zero damping (aborts if this is not true).
  void GetSolution(const Context<T>& context, const T& t,
                   systems::ContinuousState<T>* state) const {
    const Eigen::Vector2d b = get_damping_constants();
    DRAKE_DEMAND(b[0] == b[1] && b[0] == 0.0);

    using std::cos;
    using std::sin;

    // Get the offset between the two bodies
    const T offset = state->get_generalized_position().GetAtIndex(1) -
                     state->get_generalized_position().GetAtIndex(0);

    // Omega will use the first body (the one connected to the "world" with the
    // non-stiff spring).
    const double omega =
        std::sqrt(get_spring_constants()(0) / (get_mass()(0) + get_mass()(1)));

    // Setup c1 and c2 for ODE constants.
    const double c1 =
        context.get_continuous_state().get_generalized_position().GetAtIndex(0);
    const double c2 =
        context.get_continuous_state().get_generalized_velocity().GetAtIndex(
            0) /
        omega;

    // Set the position and velocity of the first body using the ODE solution.
    const double x1_final = c1 * cos(omega * t) + c2 * sin(omega * t);
    const double v1_final =
        c1 * -sin(omega * t) * omega + c2 * +cos(omega * t) * omega;
    state->get_mutable_generalized_position().SetAtIndex(0, x1_final);
    state->get_mutable_generalized_velocity().SetAtIndex(0, v1_final);

    // The position of the second body should be offset exactly from the first.
    state->get_mutable_generalized_position().SetAtIndex(1, x1_final + offset);

    // Velocity of the second body should be equal to that of the first body.
    state->get_mutable_generalized_velocity().SetAtIndex(1, v1_final);
  }
  private:
   void CopyStateOut(const systems::Context<T>& context,
                     systems::BasicVector<T>* output) const {
     output->SetFrom(context.get_continuous_state_vector());
   }
};

class StiffDoubleMassSpringSystemGeometry final
    : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StiffDoubleMassSpringSystemGeometry);
  ~StiffDoubleMassSpringSystemGeometry() = default;

  /// Creates, adds, and connects a PendulumGeometry system into the given
  /// `builder`.  Both the `pendulum_state.get_system()` and `scene_graph`
  /// systems must have been added to the given `builder` already.
  ///
  /// The `scene_graph` pointer is not retained by the %PendulumGeometry system.
  /// The return value pointer is an alias of the new %PendulumGeometry system
  /// that is owned by the `builder`.
  static const StiffDoubleMassSpringSystemGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& pendulum_state_port,
      geometry::SceneGraph<double>* scene_graph) {
    DRAKE_THROW_UNLESS(builder != nullptr);
    DRAKE_THROW_UNLESS(scene_graph != nullptr);

    auto pendulum_geometry =
        builder->AddSystem(std::unique_ptr<StiffDoubleMassSpringSystemGeometry>(
            new StiffDoubleMassSpringSystemGeometry(scene_graph)));
    builder->Connect(pendulum_state_port, pendulum_geometry->get_input_port(0));
    builder->Connect(
        pendulum_geometry->get_output_port(0),
        scene_graph->get_source_pose_port(pendulum_geometry->source_id_));

    return pendulum_geometry;
  }

 private:
  explicit StiffDoubleMassSpringSystemGeometry(
      geometry::SceneGraph<double>* scene_graph) {
    DRAKE_THROW_UNLESS(scene_graph != nullptr);
    source_id_ = scene_graph->RegisterSource();
    frame_id1_ =
        scene_graph->RegisterFrame(source_id_, geometry::GeometryFrame("m1"));
    frame_id2_ =
        scene_graph->RegisterFrame(source_id_, geometry::GeometryFrame("m2"));

    this->DeclareVectorInputPort("state", systems::BasicVector<double>(4));
    this->DeclareAbstractOutputPort(
        "geometry_pose",
        &StiffDoubleMassSpringSystemGeometry::OutputGeometryPose);

    // Wall to the left.
    geometry::GeometryId id = scene_graph->RegisterAnchoredGeometry(
        source_id_,
        std::make_unique<geometry::GeometryInstance>(
            math::RigidTransformd(Vector3d(-1.0, 0.0, 0.0)),
            std::make_unique<geometry::Box>(0.1, 20.0, 20.0), "wall"));
    scene_graph->AssignRole(source_id_, id,
                            geometry::MakePhongIllustrationProperties(orange));

    // Mass 1
    id = scene_graph->RegisterGeometry(
        source_id_, frame_id1_,
        std::make_unique<geometry::GeometryInstance>(
            math::RigidTransformd(),
            std::make_unique<geometry::Box>(1.0, 1.0, 1.0), "m1"));
    scene_graph->AssignRole(source_id_, id,
                            geometry::MakePhongIllustrationProperties(red));

    // Mass 2
    id = scene_graph->RegisterGeometry(
        source_id_, frame_id2_,
        std::make_unique<geometry::GeometryInstance>(
            math::RigidTransformd(),
            std::make_unique<geometry::Box>(1.0, 1.0, 1.0), "m2"));
    scene_graph->AssignRole(source_id_, id,
                            geometry::MakePhongIllustrationProperties(green));
  }

  void OutputGeometryPose(const systems::Context<double>& context,
                          geometry::FramePoseVector<double>* poses) const {
    DRAKE_DEMAND(frame_id1_.is_valid() && frame_id2_.is_valid());

    const auto& input =
        get_input_port(0).Eval<systems::BasicVector<double>>(context);
    const double x1 = input[0];
    const double x2 = input[1];
    const math::RigidTransformd pose1(Vector3d(x1, 0.0, 0.0));
    const math::RigidTransformd pose2(Vector3d(x2, 0.0, 0.0));

    *poses = {{frame_id1_, pose1}, {frame_id2_, pose2}};
  }

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_;
  // The id for the pendulum (arm + point mass) frame.
  geometry::FrameId frame_id1_, frame_id2_;
};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<StiffDoubleMassSpringSystem>();
  plant->set_name("plant");
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  StiffDoubleMassSpringSystemGeometry::AddToBuilder(
      &builder, plant->get_state_output_port(), scene_graph);
  ConnectDrakeVisualizer(&builder, *scene_graph, FLAGS_viz_period);
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(*plant, context.get());
  plant->SetDefaultState(plant_context, &plant_context.get_mutable_state());

  auto simulator = MakeSimulatorFromGflags(*diagram, std::move(context));

  // Additional integration parameters.
  auto& integrator = simulator->get_mutable_integrator();
  // integrator.set_requested_minimum_step_size(FLAGS_h_min);
  // integrator.set_throw_on_minimum_step_size_violation(
  //    FLAGS_throw_on_reaching_h_min);

  auto* implicit_integrator =
      dynamic_cast<systems::ImplicitIntegrator<double>*>(&integrator);
  if (implicit_integrator) {
    implicit_integrator->set_use_full_newton(FLAGS_use_full_newton);
  }

  integrator.set_maximum_step_size(0.1);
  integrator.set_loose_accuracy_band(FLAGS_h_loose_band, FLAGS_a_loose_band);

  // auto* context = simulator->get_mutable_context();
  simulator->AdvanceTo(FLAGS_duration);

  PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // namespace hertz_contact
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::hertz_contact::do_main(argc, argv);
}
