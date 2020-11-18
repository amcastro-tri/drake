
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
namespace integrators {
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
DEFINE_double(frequency, 2.0, "External force frequency, [Hz].");
DEFINE_double(F0, 10.0, "External force amplitude, [N].");
DEFINE_double(vs, 1.0e-4, "Regularization velcotiy, [m/s].");
DEFINE_string(jacobian_scheme, "forward", "Jacobian computation scheme.");

DEFINE_double(h_min, 0.0, "Minimum step we allo the integrator to take.");
DEFINE_double(viz_period, 1.0 / 60.0, "Viz period.");
DEFINE_bool(throw_on_reaching_h_min, false,
            "Whether to throw or not when h_min is reached.");
DEFINE_bool(use_full_newton, true, "Use full Newton, otherwise quasi-Newton.");
DEFINE_double(h_loose_band, 1.0e-4, "Loose band upper limit.");
DEFINE_double(a_loose_band, 0.5, "Accuracy within the loose band.");
DEFINE_bool(print_out, false, "Print monitors.");
DEFINE_bool(with_monitor, true,
            "Uses monitor to stop sim on breaking contact.");

struct Parameters {
  double m{1.0};
  double mu{0.5};
  double pi{10.0};             // Known normal force.
  double frequency{2.0};       // Oscillation frequency, [Hz].
  double force_amplitude{10};  // Applied force amplitude, [N].
  double vs{1e-4};             // [m/s]
};

template <class T>
class OscillatingBelt : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OscillatingBelt)

  OscillatingBelt(const Parameters& parameters)
      : systems::LeafSystem<T>(systems::SystemTypeTag<OscillatingBelt>{}),
        parameters_(parameters) {
    this->DeclareContinuousState(1 /* num_q */, 1 /* num_v */, 0 /* num_z */);
    this->DeclareVectorOutputPort("state", systems::BasicVector<T>(2),
                                  &OscillatingBelt::CopyStateOut,
                                  {this->all_state_ticket()});
  }

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit OscillatingBelt(const OscillatingBelt<U>& other)
      : OscillatingBelt<T>(other.parameters()) {}

  const Parameters& parameters() const { return parameters_; }

  const systems::OutputPort<T>& get_state_output_port() const {
    DRAKE_DEMAND(systems::LeafSystem<T>::num_output_ports() == 1);
    return systems::LeafSystem<T>::get_output_port(0);
  }

  void SetDefaultState(const Context<T>&,
                       systems::State<T>* state) const override {
    state->get_mutable_continuous_state()[0] = 0.0;
    state->get_mutable_continuous_state()[1] = 0.0;                         
  }

  void GetSolution(const Context<T>& context, const T& t,
                   systems::ContinuousState<T>* state) const {}

  systems::EventStatus StateMonitor(
      const Context<T>& root_context) const {
    //const Context<T>& plant_context =
    //    this->GetSubsystemContext(*this, root_context);
    const Context<T>& plant_context = this->GetMyContextFromRoot(root_context);

    const T time = plant_context.get_time();

    const T h =  time - previous_time_;
    previous_time_ = time;

    const T x = get_position(plant_context);
    const T v = get_velocity(plant_context);
    const T F = CalcAppliedForce(plant_context);
    const T beta = CalcFrictionForce(plant_context);

    if (FLAGS_print_out) {
      std::cout << time << " " << h << " " << x << " " << v << " " << beta
                << " " << F << std::endl;
    }

    // writing stuff always success.
    return systems::EventStatus::Succeeded();    
  }                   

 private:
  // Block position in non-inertial frame.
  T get_position(const Context<T>& c) const {
    return c.get_continuous_state()[0];
  }

  // Block velocity in non-inertial frame.
  T get_velocity(const Context<T>& c) const {
    return c.get_continuous_state()[1];
  }

  T CalcRegularizedFriction(const Context<T>& context) const {
    // Block velocity in non-inertial frame.
    using std::abs;
    using std::min;
    const T slip = abs(get_velocity(context));
    return parameters_.mu * min(slip / parameters_.vs, 1.0);
  }

  T CalcFrictionForce(const Context<T>& context) const {
    using std::abs;
    using std::min;
    const T v = get_velocity(context);
    const T slip = abs(v);
    const double mu = parameters_.mu;
    const double vs = parameters_.vs;
    const double pi = parameters_.pi;
    if (slip <= parameters_.vs) {  // static
      return -mu * pi * v / vs;
    } else {
      // N.B. There will never be division by zero since here v > vs.
      return -mu * pi * v / slip;
    }
  }

  T CalcAppliedForce(const Context<T>& context) const {
     using std::sin;
    const double Fo = parameters_.force_amplitude;
    const double frequency = parameters_.frequency;
    const double omega = 2 * M_PI * frequency;
    const T time = context.get_time();
    const T F = Fo * sin(omega * time);
    return F;
  }

  void DoCalcTimeDerivatives(
      const Context<T>& context,
      systems::ContinuousState<T>* deriv) const override {
    using std::sin;
    
    const double m = parameters_.m;

    const T F = CalcAppliedForce(context);
    const T beta = CalcFrictionForce(context);

    const T xdot = get_velocity(context);
    const T vdot = (F + beta) / m;    

    // Set the derivatives.
    (*deriv)[0] = xdot;
    (*deriv)[1] = vdot;
  }

 private:
  Parameters parameters_;
  mutable T previous_time_;
  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const {
    output->SetFrom(context.get_continuous_state_vector());
  }
};

class OscillatingBeltGeometry final
    : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OscillatingBeltGeometry);
  ~OscillatingBeltGeometry() = default;

  static const OscillatingBeltGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const systems::OutputPort<double>& pendulum_state_port,
      geometry::SceneGraph<double>* scene_graph) {
    DRAKE_THROW_UNLESS(builder != nullptr);
    DRAKE_THROW_UNLESS(scene_graph != nullptr);

    auto geometry =
        builder->AddSystem(std::unique_ptr<OscillatingBeltGeometry>(
            new OscillatingBeltGeometry(scene_graph)));
    builder->Connect(pendulum_state_port, geometry->get_input_port(0));
    builder->Connect(
        geometry->get_output_port(0),
        scene_graph->get_source_pose_port(geometry->source_id_));

    return geometry;
  }

 private:
  explicit OscillatingBeltGeometry(
      geometry::SceneGraph<double>* scene_graph) {
    DRAKE_THROW_UNLESS(scene_graph != nullptr);
    source_id_ = scene_graph->RegisterSource();
    frame_id_ =
        scene_graph->RegisterFrame(source_id_, geometry::GeometryFrame("box"));

    this->DeclareVectorInputPort("state", systems::BasicVector<double>(2));
    this->DeclareAbstractOutputPort(
        "geometry_pose",
        &OscillatingBeltGeometry::OutputGeometryPose);

    // Ground
    geometry::GeometryId id = scene_graph->RegisterAnchoredGeometry(
        source_id_,
        std::make_unique<geometry::GeometryInstance>(
            math::RigidTransformd(Vector3d(0.0, 0.0, -0.05)),
            std::make_unique<geometry::Box>(20.0, 20.0, 0.1), "ground"));
    scene_graph->AssignRole(source_id_, id,
                            geometry::MakePhongIllustrationProperties(orange));

    // Box
    id = scene_graph->RegisterGeometry(
        source_id_, frame_id_,
        std::make_unique<geometry::GeometryInstance>(
            math::RigidTransformd(),
            std::make_unique<geometry::Box>(0.15, 0.15, 0.15), "box"));
    scene_graph->AssignRole(source_id_, id,
                            geometry::MakePhongIllustrationProperties(blue));
  }

  void OutputGeometryPose(const systems::Context<double>& context,
                          geometry::FramePoseVector<double>* poses) const {
    DRAKE_DEMAND(frame_id_.is_valid());

    const auto& input =
        get_input_port(0).Eval<systems::BasicVector<double>>(context);
    const double x = input[0];
    const math::RigidTransformd pose(Vector3d(x, 0.0, 0.5));

    *poses = {{frame_id_, pose}};
  }

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_;
  // The id for the pendulum (arm + point mass) frame.
  geometry::FrameId frame_id_;
};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;

  Parameters parameters;
  parameters.frequency = FLAGS_frequency;
  parameters.force_amplitude = FLAGS_F0;
  parameters.vs = FLAGS_vs;

  auto plant = builder.AddSystem<OscillatingBelt>(parameters);
  plant->set_name("plant");

  // The LCM publishers are not scalar convertible.
  if (FLAGS_jacobian_scheme != "automatic") {
    auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
    OscillatingBeltGeometry::AddToBuilder(
        &builder, plant->get_state_output_port(), scene_graph);
    ConnectDrakeVisualizer(&builder, *scene_graph, FLAGS_viz_period);
  }

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
    systems::ImplicitIntegrator<double>::JacobianComputationScheme jac_scheme;
    if (FLAGS_jacobian_scheme == "forward") {
      jac_scheme = systems::ImplicitIntegrator<
          double>::JacobianComputationScheme::kForwardDifference;
    } else if (FLAGS_jacobian_scheme == "central") {
      jac_scheme = systems::ImplicitIntegrator<
          double>::JacobianComputationScheme::kCentralDifference;
    } else if (FLAGS_jacobian_scheme == "automatic") {
      jac_scheme = systems::ImplicitIntegrator<
          double>::JacobianComputationScheme::kAutomatic;
    } else {
      throw std::runtime_error("Invalid jacobian computation method");
    }
    implicit_integrator->set_jacobian_computation_scheme(jac_scheme);
    implicit_integrator->set_use_full_newton(FLAGS_use_full_newton);
  }

  integrator.set_maximum_step_size(0.1);
  integrator.set_loose_accuracy_band(FLAGS_h_loose_band, FLAGS_a_loose_band);

  // We monitor forces only for the continuous case.
  if (FLAGS_with_monitor)
    simulator->set_monitor([plant](const Context<double>& root_context) {
      return plant->StateMonitor(root_context);
    });

  // Call monitor the intial condition.
  plant->StateMonitor(simulator->get_context());

  // auto* context = simulator->get_mutable_context();
  simulator->AdvanceTo(FLAGS_duration);

  PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // namespace integrators
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::integrators::do_main(argc, argv);
}
