#include <memory>

#include <fstream>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/contact_solvers/mp_convex_solver.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/geometry/drake_visualizer.h"

DEFINE_double(time_step, 1.0E-3,
              "If time_step > 0, the fixed-time step period (in seconds) of "
              "discrete updates for the plant (modeled as a discrete system). "
              "If time_step = 0, the plant is modeled as a continuous system "
              "and no contact forces are displayed.  time_step must be >= 0.");
DEFINE_double(simulation_time, 2.0, "Duration of the simulation in seconds.");
DEFINE_double(friction_coefficient, 0.2, "Friction coefficient.");
DEFINE_bool(use_tamsi, true,
            "If true it uses TAMSI, otherwise MpConvexSolver.");
DEFINE_string(solver, "gurobi", "Underlying solver. 'gurobi', 'scs'");

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace examples {
namespace push_box {
namespace {

using drake::multibody::contact_solvers::internal::MpConvexSolver;
using drake::multibody::contact_solvers::internal::MpConvexSolverParameters;
using drake::multibody::contact_solvers::internal::MpConvexSolverStats;
using Eigen::Vector3d;
using systems::Context;

int do_main() {
  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));

  const std::string model_file_name =
      "drake/examples/multibody/push_box/conveyor_belt.sdf";
  const std::string full_name = FindResourceOrThrow(model_file_name);

  Parser parser(&plant);
  parser.AddModelFromFile(full_name);

  plant.mutable_gravity_field().set_gravity_vector(Vector3d(0.0, 0.0, -10.0));

  // We are done defining the model. Finalize.
  plant.Finalize();

  //ConnectDrakeVisualizer(&builder, scene_graph);
  geometry::DrakeVisualizerParams viz_params;
  viz_params.publish_period = FLAGS_time_step;
  geometry::DrakeVisualizer::AddToBuilder(&builder, scene_graph, nullptr,
                                          viz_params);
//  ConnectContactResultsToDrakeVisualizer(&builder, plant);

  // Create full Diagram for with the model.
  auto diagram = builder.Build();

  // Create a context for the diagram and extract the context for the
  // strandbeest model.
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  // Add external forcing.
  DRAKE_DEMAND(plant.num_actuators() == 0);
  //plant.get_actuation_input_port().FixValue(&plant_context,
  //                                          Vector1d(FLAGS_horizontal_force));

  // Set initial conditions.
  const double phi0 = 0.0;
  const auto& z_slider = plant.GetJointByName<PrismaticJoint>("z_slider");
  z_slider.set_translation(&plant_context, phi0);

  // Ground horizontal velocity.
  const double vt0 = -0.1;
  const auto& belt_slider = plant.GetJointByName<PrismaticJoint>("belt_slider");
  belt_slider.set_translation_rate(&plant_context, vt0);
  belt_slider.set_translation(&plant_context, 0.0);

  // Set contact solver.
  MpConvexSolver<double>* solver{nullptr};
  if (!FLAGS_use_tamsi) {
    solver =
        &plant.set_contact_solver(std::make_unique<MpConvexSolver<double>>());
    MpConvexSolverParameters params;
    params.alpha = 1.0;
    params.Rt_factor = 0.01;
    // Opopt: It fails very often.
    // params.solver_id = solvers::IpoptSolver::id();

    // Nlopt: "converges", but analytical ID errors are large.
    // params.solver_id = solvers::NloptSolver::id();

    if (FLAGS_solver == "scs") {
      // ScsSolver: Shows good performance/convergence.
      params.solver_id = solvers::ScsSolver::id();
    } else if (FLAGS_solver == "gurobi") {
      // GurobiSolver.
      // Compile with: bazel run --config gurobi ....
      params.solver_id = solvers::GurobiSolver::id();
    } else {
      throw std::runtime_error("Solver not supported.");
    }
    solver->set_parameters(params);
  }

  // Create a simulator and run the simulation.
  std::unique_ptr<systems::Simulator<double>> simulator =
      systems::MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  const auto& box = plant.GetBodyByName("box");

  std::ofstream file("sol.dat");  
  //file << fmt::format("{:>18} {:>18}\n", "time", "z");
  simulator->set_monitor([&](const Context<double>& root_context) {
    const systems::Context<double>& ctxt =
        plant.GetMyContextFromRoot(root_context);
    const ContactResults<double>& contact_results =
        plant.get_contact_results_output_port().Eval<ContactResults<double>>(
            ctxt);
    DRAKE_DEMAND(contact_results.num_point_pair_contacts() == 1);
    const PointPairContactInfo<double>& point_pair_info =
        contact_results.point_pair_contact_info(0);
    const double sign =
        point_pair_info.bodyB_index() == box.index() ? 1.0 : -1.0;
    const Vector3d f_Bc_W = point_pair_info.contact_force() * sign;
    const double vn = point_pair_info.separation_speed();
    const double slip = point_pair_info.slip_speed();

    const MpConvexSolverStats& stats = solver->get_iteration_stats();

    // time, z, zdot, ft, fn, vr, vn, id_rel_err, id_abs_err
    file << fmt::format(
        "{:20.8g} {:20.8g} {:20.8g} {:20.8g} {:20.8g} {:20.8g} {:20.8g} "
        "{:20.8g} {:20.8g}\n",
        ctxt.get_time(), z_slider.get_translation(ctxt),
        z_slider.get_translation_rate(ctxt), f_Bc_W(0), f_Bc_W(2), slip, vn,
        stats.iteration_errors.id_rel_error,
        stats.iteration_errors.id_abs_error);
    return systems::EventStatus::Succeeded();
  });

  simulator->AdvanceTo(FLAGS_simulation_time);
  file.close();

  // Print some useful statistics.
  PrintSimulatorStatistics(*simulator);

  PRINT_VAR(z_slider.get_translation(plant_context));

  // Print contact solver stats.
  if (!FLAGS_use_tamsi) {
    const std::vector<MpConvexSolverStats>& stats_hist =
        solver->get_stats_history();
    std::cout << std::string(80, '-') << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    fmt::print("{:>18} {:>18} {:>18}  {:>18}\n", "num_contacts", "id_rel_err",
               "id_abs_error", "gamma_norm", "vs_max");
    for (const auto& s : stats_hist) {
      fmt::print("{:d} {:20.8g} {:20.8g} {:20.8g} {:20.8g}\n", s.num_contacts,
                 s.iteration_errors.id_rel_error,
                 s.iteration_errors.id_abs_error,
                 s.iteration_errors.gamma_norm,
                 s.iteration_errors.vs_max);
    }
  }

  return 0;
}
}  // namespace
}  // namespace push_box
}  // namespace examples
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("\nBox pushed by a constant external force.\n");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::examples::push_box::do_main();
}
