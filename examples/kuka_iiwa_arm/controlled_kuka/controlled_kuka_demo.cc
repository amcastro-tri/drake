/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to reach to a
/// position in space and then repeat this reaching task with a different joint
/// configuration constraint.

#include <iostream>
#include <memory>
#include <string>

#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"

DEFINE_double(simulation_sec, 0.01, "Number of seconds to simulate.");

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::multibody::Body;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::parsing::AddModelFromSdfFile;
using drake::multibody::RevoluteJoint;
using drake::multibody::JointActuatorIndex;
using drake::multibody::UniformGravityFieldElement;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::Serializer;
using drake::systems::rendering::PoseBundleToDrawMessage;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;
using std::make_unique;
using std::move;
using std::string;
using std::unique_ptr;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using manipulation::util::SimDiagramBuilder;
using trajectories::PiecewisePolynomial;

const char kUrdfPath[] =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

const char kSdfPath[] = "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf";

PiecewisePolynomial<double> MakePlan() {
  auto tree = make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(kUrdfPath), multibody::joints::kFixed, tree.get());

  // Creates a basic pointwise IK trajectory for moving the iiwa arm.
  // It starts in the zero configuration (straight up).
  VectorXd zero_conf = tree->getZeroConfiguration();
  VectorXd joint_lb = zero_conf - VectorXd::Constant(7, 0.01);
  VectorXd joint_ub = zero_conf + VectorXd::Constant(7, 0.01);

  PostureConstraint pc1(tree.get(), Vector2d(0, 0.5));
  VectorXi joint_idx(7);
  joint_idx << 0, 1, 2, 3, 4, 5, 6;
  pc1.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Defines an end effector constraint and makes it active for the time span
  // from 1 to 3 seconds.
  Vector3d pos_end(0.6, 0, 0.325);
  Vector3d pos_lb = pos_end - Vector3d::Constant(0.005);
  Vector3d pos_ub = pos_end + Vector3d::Constant(0.005);
  WorldPositionConstraint wpc1(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(1, 3));

  // After the end effector constraint is released, applies the straight
  // up configuration again from time 4 to 5.9.
  PostureConstraint pc2(tree.get(), Vector2d(4, 5.9));
  pc2.setJointLimits(joint_idx, joint_lb, joint_ub);

  // Apply the same end effector constraint from time 6 to 9 of the demo.
  WorldPositionConstraint wpc2(tree.get(), tree->FindBodyIndex("iiwa_link_ee"),
                               Vector3d::Zero(), pos_lb, pos_ub,
                               Vector2d(6, 9));

  // For part of the time wpc2 is active, constrains the second joint while
  // preserving the end effector constraint.
  //
  // Variable `joint_position_start_idx` below is a collection of offsets into
  // the state vector referring to the positions of the joints to be
  // constrained.
  Eigen::VectorXi joint_position_start_idx(1);
  joint_position_start_idx(0) =
      tree->FindChildBodyOfJoint("iiwa_joint_2")->get_position_start_index();
  PostureConstraint pc3(tree.get(), Vector2d(6, 8));
  pc3.setJointLimits(joint_position_start_idx, Vector1d(0.7), Vector1d(0.8));

  const std::vector<double> kTimes{0.0, 2.0, 5.0, 7.0, 9.0};
  MatrixXd q_seed(tree->get_num_positions(), kTimes.size());
  MatrixXd q_nom(tree->get_num_positions(), kTimes.size());
  for (size_t i = 0; i < kTimes.size(); ++i) {
    // Zero configuration is a bad initial guess for IK, to be solved through
    // nonlinear optimization, as the robot configuration is in singularity,
    // and the gradient is zero. So we add 0.1 as the arbitrary pertubation
    // to the zero configuration.
    q_seed.col(i) =
        zero_conf + 0.1 * Eigen::VectorXd::Ones(tree->get_num_positions());
    q_nom.col(i) = zero_conf;
  }

  std::vector<RigidBodyConstraint*> constraint_array;
  constraint_array.push_back(&pc1);
  constraint_array.push_back(&wpc1);
  constraint_array.push_back(&pc2);
  constraint_array.push_back(&pc3);
  constraint_array.push_back(&wpc2);
  IKoptions ikoptions(tree.get());
  std::vector<int> info(kTimes.size(), 0);
  MatrixXd q_sol(tree->get_num_positions(), kTimes.size());
  std::vector<std::string> infeasible_constraint;

  inverseKinPointwise(tree.get(), kTimes.size(), kTimes.data(), q_seed, q_nom,
                      constraint_array.size(), constraint_array.data(),
                      ikoptions, &q_sol, info.data(), &infeasible_constraint);
  bool info_good = true;
  for (size_t i = 0; i < kTimes.size(); ++i) {
    drake::log()->info("INFO[{}] = {} ", i, info[i]);
    if (info[i] != 1) {
      info_good = false;
    }
  }
  printf("\n");

  if (!info_good) {
    throw std::runtime_error(
        "inverseKinPointwise failed to compute a valid solution.");
  }

  std::vector<MatrixXd> knots(kTimes.size());
  for (size_t i = 0; i < kTimes.size(); ++i) {
    // We only use column 0 of the matrix in knots (for joint positions),
    // so we write a vector.
    knots[i] = q_sol.col(i);
  }

  return PiecewisePolynomial<double>::FirstOrderHold(kTimes, knots);
}

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);

  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Make and add the kuka robot model.
  MultibodyPlant<double>& kuka_plant = *builder.AddSystem<MultibodyPlant>();
  AddModelFromSdfFile(FindResourceOrThrow(kSdfPath), &kuka_plant, &scene_graph);

  // Add gravity to the model.
  kuka_plant.AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3<double>::UnitZ());

  // Now the model is complete.
  kuka_plant.Finalize();

  PRINT_VAR(kuka_plant.num_positions());
  PRINT_VAR(kuka_plant.num_actuators());
  PRINT_VAR(kuka_plant.num_actuated_dofs());
  PRINT_VAR(kuka_plant.num_visual_geometries());
  PRINT_VAR(kuka_plant.num_collision_geometries());

  DRAKE_THROW_UNLESS(kuka_plant.num_positions() == 7);

  // Boilerplate used to connect the plant to a SceneGraph for
  // visualization.
  DrakeLcm lcm;
  const PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem& publisher =
      *builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
  std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher.set_publish_period(1 / 60.0);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!kuka_plant.get_source_id());

  builder.Connect(
      kuka_plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(kuka_plant.get_source_id().value()));
  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(scene_graph);

  //(void) MakePlan();
//#if 0
  PiecewisePolynomial<double> traj = MakePlan();

  auto tree = std::make_unique<RigidBodyTree<double>>();
  CreateTreedFromFixedModelAtPose(FindResourceOrThrow(kUrdfPath), tree.get());

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  auto controller = builder.AddSystem<
      systems::controllers::InverseDynamicsController>(
      std::move(tree), iiwa_kp, iiwa_ki, iiwa_kd,
      false /* no feedforward acceleration */);

  builder.Connect(kuka_plant.get_continuous_state_output_port(),
                  controller->get_input_port_estimated_state());

  auto traj_src =
      builder.AddSystem<systems::TrajectorySource<double>>(
          traj, 1 /* outputs q + v */);
  traj_src->set_name("trajectory_source");
  builder.Connect(traj_src->get_output_port(),
                           controller->get_input_port_desired_state());

  builder.Connect(controller->get_output_port_control(),
                  kuka_plant.get_actuation_input_port());

  for (JointActuatorIndex actuator(0); actuator < kuka_plant.num_actuators(); ++ actuator) {
    PRINT_VAR(kuka_plant.model().get_joint_actuator(actuator).name());
  }

  // Log state:
  const auto& logger = *builder.AddSystem<systems::SignalLogger>(14);
  builder.Connect(kuka_plant.get_continuous_state_output_port(),
                  logger.get_input_port());

  // Log acutation:
  const auto& logger_act = *builder.AddSystem<systems::SignalLogger>(7);
  builder.Connect(controller->get_output_port_control(),
                  logger_act.get_input_port());

  // Log traj:
  const auto& logger_traj = *builder.AddSystem<systems::SignalLogger>(14);
  builder.Connect(traj_src->get_output_port(),
                  logger_traj.get_input_port());


#if 0
  // A constant source for a zero applied torque at the elbow joint.
  const VectorXd actuation = VectorXd::Constant(7, 0.1);
  auto actuation_source =
      builder.AddSystem<systems::ConstantVectorSource>(actuation);
  actuation_source->set_name("Constant Actuation");
  builder.Connect(actuation_source->get_output_port(),
                  kuka_plant.get_actuation_input_port());
#endif

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  PRINT_VAR(kuka_plant.get_actuation_input_port().get_index());
  PRINT_VAR(kuka_plant.get_num_input_ports());
  PRINT_VAR(kuka_plant.get_num_output_ports());
  PRINT_VAR(kuka_plant.get_actuation_input_port().size());

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  simulator.StepTo(FLAGS_simulation_sec);

  PRINT_VAR(logger.data().rows());
  PRINT_VAR(logger.data().cols());
  PRINT_VAR(logger.sample_times().rows());

  std::ofstream fstate("state.dat");
  MatrixXd state_out(logger.data().cols(), 15);
  state_out << logger.sample_times() , logger.data().transpose();
  fstate << state_out;
  fstate.close();

  std::ofstream fact("actuation.dat");
  MatrixXd act_out(logger_act.data().cols(), 8);
  act_out << logger_act.sample_times() , logger_act.data().transpose();
  fact << act_out;
  fact.close();

  std::ofstream ftraj("traj.dat");
  MatrixXd traj_out(logger_traj.data().cols(), 15);
  traj_out << logger_traj.sample_times() , logger_traj.data().transpose();
  ftraj << traj_out;
  ftraj.close();

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
