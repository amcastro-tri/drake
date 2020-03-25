
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
DEFINE_int32(num_spheres, 3, "Number of spheres.");
DEFINE_double(dissipation, 0.26, "Hunt & Crossly dissipation, [s/m].");
DEFINE_double(elastic_modulus, 10.0e9, "Elastic modulus, [Pa].");
DEFINE_double(initial_separation, 0.0, "Initial separation, [m]");

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
  int num_spheres{3};
  double m{1.0};  // All spheres have the same mass.
  double g{10.0}; // Acceleration of graivty.

  // Hertz model parameters.
  double sphere_radius{0.1};
  double elastic_modulus{10.0e9};          // 10 GPa
  double hunt_crossley_dissipation{0.26};  // [s/m].

  // Initial contions.
  double x0{0.0};  // Initial separation (-penetration).
};

template <class T>
class SpheresStack : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpheresStack)

  SpheresStack(const Parameters& parameters)
      : systems::LeafSystem<T>(systems::SystemTypeTag<SpheresStack>{}),
        parameters_(parameters) {
    this->DeclareContinuousState(parameters_.num_spheres /* num_q */,
                                 parameters_.num_spheres /* num_v */,
                                 0 /* num_z */);
    this->DeclareVectorOutputPort(
        "state", systems::BasicVector<T>(2 * parameters_.num_spheres),
        &SpheresStack::CopyStateOut, {this->all_state_ticket()});
  }

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit SpheresStack(const SpheresStack<U>& other)
      : SpheresStack<T>(other.parameters()) {}

  const Parameters& parameters() const { return parameters_; }

  const systems::OutputPort<T>& get_state_output_port() const {
    DRAKE_DEMAND(systems::LeafSystem<T>::num_output_ports() == 1);
    return systems::LeafSystem<T>::get_output_port(0);
  }

  void SetDefaultState(const Context<T>&,
                       systems::State<T>* state) const override {
    double R = parameters_.sphere_radius;                         
    double x0 = parameters_.x0;
    for (int sphere = 0; sphere < parameters_.num_spheres; ++sphere) {
      // position
      state->get_mutable_continuous_state()[sphere] =
          R + (2.0 * R) * sphere + (sphere + 1) * x0;
      // velocity
      state->get_mutable_continuous_state()[parameters_.num_spheres + sphere] =
          0.0;
    }
  }

  systems::EventStatus StateMonitor(
      const Context<T>& root_context) const {
    const Context<T>& plant_context = this->GetMyContextFromRoot(root_context);

    const T time = plant_context.get_time();

    const T h =  time - previous_time_;
    previous_time_ = time;

    //const T x = get_position(plant_context, sphere);
    //const T v = get_velocity(plant_context, sphere);
    //const T F = CalcAppliedForce(plant_context);
    //const T beta = CalcFrictionForce(plant_context);

    if (FLAGS_print_out) {
      std::cout << time << " " << h;
      for (int i =0; i<parameters_.num_spheres;++i) {
        const T x = get_position(plant_context, i);
        const T v = get_velocity(plant_context, i);
        std::cout << " " << x << " " << v;
      }
      std::cout << std::endl;

      //std::cout << time << " " << h << " " << x << " " << v << " " << beta
       //         << " " << F << std::endl;
    }

    // writing stuff always success.
    return systems::EventStatus::Succeeded();    
  }                   

 private:
  // Block position in non-inertial frame.
  T get_position(const Context<T>& c, int sphere) const {
    return c.get_continuous_state()[sphere];
  }

  // Block velocity in non-inertial frame.
  T get_velocity(const Context<T>& c, int sphere) const {
    return c.get_continuous_state()[parameters_.num_spheres + sphere];
  }


  // Contact force at the bottom contact point for the i-th sphere.
  // Force has the sign as applied on the i-th sphere (i.e. positive if there is
  // a repulsive contact force from the bottom.)
  T CalcBottomContactForce(const Context<T>& context, int i) const {    
    const double R = parameters_.sphere_radius;
    const double E = parameters_.elastic_modulus;
    const double d = parameters_.hunt_crossley_dissipation;

    // Penetration x (>0) and penetration velocity xdot (>0 if getting closer)
    const T zi = get_position(context, i);
    const T vi = get_velocity(context, i);
    T x, xdot;
    if (i == 0) {
      // Contact with the ground
      x = R - zi;
      xdot = -vi;
    } else {
      // Contact with the bottom sphere.
      const T zim = get_position(context, i-1);
      const T vim = get_velocity(context, i-1);
      x = -(zi - zim - 2 * R);
      xdot = -vi + vim;
    }
    
    if (x > 0) {
    // Normal forces
    using std::sqrt;
    const T fHz = 4. / 3. * E * sqrt(R * x * x * x);

    using std::max;
    const T fHC = fHz * max(0.0, 1 + 1.5 * d * xdot);
    
    return fHz + fHC;
    } else {
      return 0.0;
    }
  }

  void DoCalcTimeDerivatives(
      const Context<T>& context,
      systems::ContinuousState<T>* deriv) const override {
    using std::sin;
    
    const int nspheres = parameters_.num_spheres;
    const double m = parameters_.m;
    const double g = parameters_.g;

    // Accumulate all contact forces.
    VectorX<T> fi(nspheres);
    for (int i = 0; i < nspheres; ++i) {
      // Force on ith sphere.
      fi[i] = CalcBottomContactForce(context, i);
    }

    // Compute accelerations.    
    for (int i = 0; i < nspheres; ++i) {
      T zdd = fi[i] / m - g;
      if (i < nspheres - 1) {
        zdd -= fi[i + 1] / m;
      }

      T zd = get_velocity(context, i);

      // Set the derivatives.
      (*deriv)[i] = zd;
      (*deriv)[nspheres + i] = zdd;
    }
  }

 private:
  Parameters parameters_;
  mutable T previous_time_;
  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const {
    output->SetFrom(context.get_continuous_state_vector());
  }
};

class SpheresStackGeometry final
    : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpheresStackGeometry);
  ~SpheresStackGeometry() = default;

  static const SpheresStackGeometry* AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const SpheresStack<double>& plant,
      geometry::SceneGraph<double>* scene_graph) {
    DRAKE_THROW_UNLESS(builder != nullptr);
    DRAKE_THROW_UNLESS(scene_graph != nullptr);

    auto geometry = builder->AddSystem(std::unique_ptr<SpheresStackGeometry>(
        new SpheresStackGeometry(plant, scene_graph)));
    builder->Connect(plant.get_state_output_port(),
                     geometry->get_input_port(0));
    builder->Connect(geometry->get_output_port(0),
                     scene_graph->get_source_pose_port(geometry->source_id_));

    return geometry;
  }

 private:
  explicit SpheresStackGeometry(const SpheresStack<double>& plant,
                                geometry::SceneGraph<double>* scene_graph) {
    DRAKE_THROW_UNLESS(scene_graph != nullptr);
    source_id_ = scene_graph->RegisterSource();  

    const Parameters& params = plant.parameters();
    num_spheres_ = params.num_spheres;    

    this->DeclareVectorInputPort(
        "state", systems::BasicVector<double>(2 * num_spheres_));
    this->DeclareAbstractOutputPort(
        "geometry_pose",
        &SpheresStackGeometry::OutputGeometryPose);

    // Ground
    geometry::GeometryId id = scene_graph->RegisterAnchoredGeometry(
        source_id_,
        std::make_unique<geometry::GeometryInstance>(
            math::RigidTransformd(Vector3d(0.0, 0.0, -0.05)),
            std::make_unique<geometry::Box>(20.0, 20.0, 0.1), "ground"));
    scene_graph->AssignRole(source_id_, id,
                            geometry::MakePhongIllustrationProperties(orange));

    // Spheres
    frame_ids_.resize(num_spheres_);
    const double R = params.sphere_radius;
    for (int i = 0; i < num_spheres_; ++i) {
      std::string name = "sphere_" + std::to_string(i);
      frame_ids_[i] = scene_graph->RegisterFrame(
          source_id_, geometry::GeometryFrame(name));

      id = scene_graph->RegisterGeometry(
          source_id_, frame_ids_[i],
          std::make_unique<geometry::GeometryInstance>(
              math::RigidTransformd(),
              std::make_unique<geometry::Sphere>(R), name));
      scene_graph->AssignRole(source_id_, id,
                              geometry::MakePhongIllustrationProperties(blue));
    }
  }

  void OutputGeometryPose(const systems::Context<double>& context,
                          geometry::FramePoseVector<double>* poses) const {    
    const auto& input =
        get_input_port(0).Eval<systems::BasicVector<double>>(context);
    poses->clear();

    for (int i = 0; i < num_spheres_; ++i) {
      DRAKE_DEMAND(frame_ids_[i].is_valid());
      const double z = input[i];
      poses->set_value(frame_ids_[i],
                       math::RigidTransformd(Vector3d(0.0, 0.0, z)));
    }
  }

  // Geometry source identifier for this system to interact with SceneGraph.
  geometry::SourceId source_id_;
  int num_spheres_{0};
  std::vector<geometry::FrameId> frame_ids_;
};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  systems::DiagramBuilder<double> builder;

  Parameters parameters;
  parameters.num_spheres = FLAGS_num_spheres;
  parameters.elastic_modulus = FLAGS_elastic_modulus;
  parameters.hunt_crossley_dissipation = FLAGS_dissipation;
  parameters.x0 = FLAGS_initial_separation;

  auto plant = builder.AddSystem<SpheresStack>(parameters);
  plant->set_name("plant");

  // The LCM publishers are not scalar convertible.
  if (FLAGS_jacobian_scheme != "automatic") {
    auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
    SpheresStackGeometry::AddToBuilder(
        &builder, *plant, scene_graph);
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
