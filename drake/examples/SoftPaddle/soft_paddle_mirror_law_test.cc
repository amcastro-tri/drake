#include "drake/common/drake_path.h"
#include "drake/examples/SoftPaddle/soft_paddle_plant.h"
#include "drake/examples/SoftPaddle/soft_paddle_state_to_bot_visualizer.h"
#include "drake/examples/SoftPaddle/mirror_law_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

#include <iostream>
#include <drake/multibody/joints/drake_joints.h>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace examples {
namespace soft_paddle {
namespace {

using std::make_unique;
using Eigen::Isometry3d;

int do_main(int argc, char* argv[]) {
  lcm::DrakeLcm lcm;

  systems::DiagramBuilder<double> builder;
  double paddle_aim = -3.0 * M_PI / 180.0;
  double stroke_strength = 0.05;
  auto mirror_system =
      builder.AddSystem<PaddleMirrorLawSystem>(paddle_aim, stroke_strength);
  auto paddle = builder.AddSystem<SoftPaddlePlant>();
  const RigidBodyTree<double>& tree = paddle->get_rigid_body_tree_model();
  auto paddle_to_viz =
      builder.AddSystem<SoftPaddleStateToBotVisualizer>(*paddle);
  auto visualizer =
      builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);

  // Feedback loop.
  builder.Connect(paddle->get_output_port(), mirror_system->get_input_port(0));
  builder.Connect(
      mirror_system->get_paddle_angle_port(), paddle->get_tau_port());

  // Visualization.
  builder.Connect(paddle->get_output_port(),
                  paddle_to_viz->get_paddle_state_port());
  builder.Connect(mirror_system->get_paddle_angle_port(),
                  paddle_to_viz->get_paddle_angle_port());
  builder.Connect(paddle_to_viz->get_bot_visualizer_port(),
                  visualizer->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);

  // Sets initial conditions.
  systems::Context<double>* paddle_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), paddle);
  paddle->set_initial_conditions(paddle_context);

  simulator.Initialize();
  simulator.StepTo(15);

  PRINT_VAR(simulator.get_num_steps_taken());

  return 0;
}

}  // namespace
}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::soft_paddle::do_main(argc, argv);
}
