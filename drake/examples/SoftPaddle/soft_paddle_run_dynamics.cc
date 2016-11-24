#include "drake/common/drake_path.h"
#include "drake/examples/SoftPaddle/soft_paddle_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace examples {
namespace soft_paddle {
namespace {

int do_main(int argc, char* argv[]) {
  lcm::DrakeLcm lcm;
  RigidBodyTree<double> tree(
      GetDrakePath() + "/examples/SoftPaddle/soft_paddle.urdf",
      multibody::joints::kFixed);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(1);

  systems::DiagramBuilder<double> builder;
  auto source = builder.AddSystem<systems::ConstantVectorSource>(tau);
  auto paddle = builder.AddSystem<SoftPaddlePlant>();
  auto visualizer =
      builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);

  PRINT_VAR(paddle->get_visualizer_output_port().get_size());
  PRINT_VAR(visualizer->get_input_port(0).get_size());

  builder.Connect(source->get_output_port(), paddle->get_tau_port());
  builder.Connect(paddle->get_visualizer_output_port(), visualizer->get_input_port(0));


  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.);  // No faster than 1X real time.
  //systems::Context<double>* pendulum_context =
  //    diagram->GetMutableSubsystemContext(
  //        simulator.get_mutable_context(), pendulum);
 // pendulum->set_theta(pendulum_context, 1.);
  //pendulum->set_thetadot(pendulum_context, 0.);

  simulator.Initialize();
  simulator.StepTo(10);
  return 0;
}

}  // namespace
}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::soft_paddle::do_main(argc, argv);
}
