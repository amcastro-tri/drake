/// @file
///
/// This demo do blablabla

#include <gflags/gflags.h>

#include "drake/examples/allegro_hand/allegro_common.h"

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"  
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results.h"
#include "drake/multibody/multibody_tree/multibody_plant/contact_results_to_lcm.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {


namespace examples {
namespace allegro_hand {
namespace {

using drake::multibody::multibody_plant::MultibodyPlant;

DEFINE_double(constant_load, 0, "the constant load on all the joint. "
              "Suggested load is in the order of 0.01. When equals to "
              "0 (default), the program simulate the passive demo");

DEFINE_int32(joint_user_index, 3, "Actuate this joint for testing");

DEFINE_double(joint_torque, 0.05, "Torque applid to finger joint.");

DEFINE_double(simulation_time, 5, "Number of seconds to simulate");

DEFINE_string(test_hand, "right", "Which hand to model: 'left' or 'right'");

DEFINE_double(max_time_step, 1.0e-5,
              "Maximum time step used for the integrators. [s]. "
              "If negative, a value based on parameter penetration_allowance "
              "is used.");

// Contact parameters
DEFINE_double(penetration_allowance, 2.0e-3, "Penetration allowance [m]. "
              "See MultibodyPlant::set_penetration_allowance().");
DEFINE_double(v_stiction_tolerance, 1.0e-2,
              "The maximum slipping speed allowed during stiction. [m/s]");

// Integration parameters:
DEFINE_string(integration_scheme, "semi_explicit_euler",
              "Integration scheme to be used. Available options are: "
              "'semi_explicit_euler','runge_kutta2','runge_kutta3',"
              "'implicit_euler'");

DEFINE_bool(add_gravity, false, "Whether adding gravity in the simulation");

DEFINE_double(accuracy, 1.0e-2, "Sets the simulation accuracy for variable step"
              "size integrators with error control.");
DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_time > 0);

  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;

  const std::string HandSdfPath = "drake/manipulation/models/allegro_hand_"
      "description/sdf/allegro_hand_description_" + FLAGS_test_hand + ".sdf";

  // build the scene and the hand model
  geometry::SceneGraph<double>& scene_graph = 
      *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>
                                  (FLAGS_max_time_step);
  std::string full_name = FindResourceOrThrow(HandSdfPath);
  multibody::parsing::AddModelFromSdfFile(
                          full_name, &plant, &scene_graph);
  if (FLAGS_add_gravity)  plant.AddForceElement<
      multibody::UniformGravityFieldElement>(-9.81 * Eigen::Vector3d::UnitZ());

  plant.Finalize(&scene_graph);
  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_v_stiction_tolerance);

  // Actuation matrix.
  MatrixX<double> B = plant.model().actuation_map_matrix();
  PRINT_VARn(B);

  // Create a map from fingers dofs in a "desired order" into the order these
  // same dofs are arranged in the state vector.
  // This is the projection matrix Px for the PID controller.
  // This code was only written for the right hand now. Verify this.
  DRAKE_DEMAND(FLAGS_test_hand == "right");
  // This map defines in what order we want the PID controller to "see" the
  // states.
  std::vector<std::string> joint_names_map;

  // This maps from our desired joint order into a joint index in the MBP.
  std::vector<multibody::JointIndex> joint_index_map;

  // This maps from a JointIndex in the MBP into our "user-defined" ordering.
  std::vector<int> joint_user_index_map;

  // Index
  joint_names_map.push_back("joint_0");
  joint_names_map.push_back("joint_1");
  joint_names_map.push_back("joint_2");
  joint_names_map.push_back("joint_3");

  // Thumb
  joint_names_map.push_back("joint_12");
  joint_names_map.push_back("joint_13");
  joint_names_map.push_back("joint_14");
  joint_names_map.push_back("joint_15");

  // Projection matrix. We include "all" dofs in the hand.
  // x_tilde = Px * x;
  // where:
  //  x is the state in the MBP.
  //  x_tilde is the state in the order we want it for our better understanding.
  MatrixX<double> Px(
      plant.num_multibody_states(), plant.num_multibody_states());
  Px.setZero();
  const int nq = plant.num_positions();
  int joint_user_index = 0;
  joint_user_index_map.resize(plant.num_joints());
  for (const auto& joint_name : joint_names_map) {
    const auto& joint = plant.GetJointByName(joint_name);

    // joint user index to JointIndx map.
    joint_index_map.push_back(joint.index());

    // JointIndex to user index map.
    joint_user_index_map[joint.index()] = joint_user_index;

    const int q_index = joint.position_start();
    const int v_index = joint.velocity_start();
    Px(joint_user_index, q_index) = 1.0;
    Px(nq + joint_user_index, nq + v_index) = 1.0;
    ++joint_user_index;
  }

  PRINT_VARn(Px);
  // Verify the mapping (or "projection") matrix Px only has a single 1.0 entry
  // per row/column.
  for (int i=0;i<plant.num_multibody_states();++i) {
    DRAKE_DEMAND(Px.row(i).sum() == 1.0);
    DRAKE_DEMAND(Px.col(i).sum() == 1.0);
  }

  // Build the projection matrix Py for the PID controller. Maps u_c from
  // the controller into u for the MBP, that is, u = Py * u_c where:
  //  u_c is the output from the PID controller in our prefered order.
  //  u is the output as require by the MBP.
  MatrixX<double> Py(
      plant.num_actuated_dofs(), plant.num_velocities());
  Py.setZero();
  for (multibody::JointActuatorIndex actuator_index(0);
       actuator_index < plant.num_actuators(); ++actuator_index) {
    const auto& actuator = plant.model().get_joint_actuator(actuator_index);
    const auto& joint = actuator.joint();
    Py(actuator_index, joint_user_index_map[joint.index()]) = 1.0;
  }

  PRINT_VARn(Py);
  // Verify the mapping (or "projection") matrix Py only has a single 1.0 entry
  // per row/column.
  for (int i=0;i<plant.num_multibody_states();++i) {
    DRAKE_DEMAND(Px.row(i).sum() == 1.0);
    DRAKE_DEMAND(Px.col(i).sum() == 1.0);
  }


  // visualization of the hand model
  const systems::rendering::PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<systems::rendering::PoseBundleToDrawMessage>();
  systems::lcm::LcmPublisherSystem& publisher = *builder.template 
          AddSystem<systems::lcm::LcmPublisherSystem>("DRAKE_VIEWER_DRAW",
          std::make_unique<systems::lcm::Serializer<lcmt_viewer_draw>>(), &lcm);
  DRAKE_DEMAND(!!plant.get_source_id());
  builder.Connect(plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  std::cout<< "Model added \n";

  Eigen::VectorXd u_c = Eigen::VectorXd::Zero(nq);
  u_c(FLAGS_joint_user_index) = FLAGS_joint_torque;
  VectorX<double> u = Py * u_c;
  PRINT_VAR(u.transpose());
  systems::ConstantVectorSource<double>* const_src =
      builder.AddSystem<systems::ConstantVectorSource<double>>(u);
  const_src->set_name("constant_source");
  builder.Connect(const_src->get_output_port(),
                  plant.get_actuation_input_port());

  std::cout<<"trajectory added \n";


  //set up signal loggers 
  auto plant_state = builder.AddSystem<drake::systems::SignalLogger<double>>(
                          kAllegroNumJoints * 2, 10e5);
  builder.Connect(plant.get_continuous_state_output_port(), 
                  plant_state->get_input_port());
  // pid_state ->set_publish_period(1e-2);
  // plant_state ->set_publish_period(1e-2);


  // Dispatch the message to load geometry.
  geometry::DispatchLoadMessage(scene_graph);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  std::cout<<"Diagram built \n";

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  // systems::Context<double>& plant_context =
  //     diagram->GetMutableSubsystemContext(plant, diagram_context.get());


 

  // Set up simulator. Using semi_explicit_euler for now.
  const double max_time_step = FLAGS_max_time_step > 0 ? 
      FLAGS_max_time_step : plant.get_contact_penalty_method_time_scale() / 30;
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  systems::IntegratorBase<double>* integrator{nullptr};
  integrator =
        simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
            *diagram, max_time_step, &simulator.get_mutable_context());
  integrator->set_maximum_step_size(max_time_step);
  if (!integrator->get_fixed_step_mode())
    integrator->set_target_accuracy(FLAGS_accuracy);

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  // We'll log in our user-defined order for better understanding.
  for(long int i=0; i<= plant_state->data().cols(); i+=10)
    std::cout<< (Px * plant_state->data().col(i)).transpose()<<std::endl;

  // std::cout<<pid_state->data()<<std::endl;
  // std::cout<<pid_state->data().cols()<<std::endl;
    // std::cout<<pid_state->data().rows()<<std::endl;

  return 1;
}  // int main

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::allegro_hand::DoMain();
}
