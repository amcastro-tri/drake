#include "drake/systems/analysis/simulator_gflags.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/systems/analysis/bogacki_shampine3_integrator.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/radau_integrator.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/runge_kutta5_integrator.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"

// Simulator's paramters:
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(publish_every_time_step, false,
            "Sets whether the simulation should trigger a forced-Publish event "
            "on the at the end of every trajectory-advancing step. "
            "See Simulator::set_publish_every_time_step() for details.");

// Integrator's parameters:
// N.B. The list of integrators here must be kept in sync with
// ResetIntegratorFromGflags().
DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'bogacki_shampine3',"
              "'explicit_euler','implicit_euler','semi_explicit_euler',"
              "'radau1','radau3',"
              "'runge_kutta2','runge_kutta3','runge_kutta5'");

DEFINE_double(max_time_step, 1.0E-3,
              "Maximum simulation time step used for integration.");

DEFINE_double(accuracy, 1.0e-2,
              "Sets the simulation accuracy for variable step "
              "size integrators with error control.");

namespace drake {
namespace systems {

// N.B. The list of integrators here must be kept in sync with
// FLAGS_integration_scheme defined at the top of this file.
IntegratorBase<double>& ResetIntegratorFromGflags(
    Simulator<double>* simulator) {
  DRAKE_DEMAND(simulator != nullptr);

  if (FLAGS_integration_scheme == "bogacki_shampine3") {
    simulator->reset_integrator<BogackiShampine3Integrator<double>>();
  } else if (FLAGS_integration_scheme == "explicit_euler") {
    simulator->reset_integrator<ExplicitEulerIntegrator<double>>(
        FLAGS_max_time_step);
  } else if (FLAGS_integration_scheme == "implicit_euler") {
    simulator->reset_integrator<ImplicitEulerIntegrator<double>>();
  } else if (FLAGS_integration_scheme == "semi_explicit_euler") {
    simulator->reset_integrator<SemiExplicitEulerIntegrator<double>>(
        FLAGS_max_time_step);
  } else if (FLAGS_integration_scheme == "radau1") {
    simulator->reset_integrator<RadauIntegrator<double, 1>>();
  } else if (FLAGS_integration_scheme == "radau3") {
    simulator->reset_integrator<RadauIntegrator<double>>();
  } else if (FLAGS_integration_scheme == "runge_kutta2") {
    simulator->reset_integrator<RungeKutta2Integrator<double>>(
        FLAGS_max_time_step);
  } else if (FLAGS_integration_scheme == "runge_kutta3") {
    simulator->reset_integrator<RungeKutta3Integrator<double>>();
  } else if (FLAGS_integration_scheme == "runge_kutta5") {
    simulator->reset_integrator<RungeKutta5Integrator<double>>();
  } else {
    DRAKE_UNREACHABLE();
  }
  IntegratorBase<double>& integrator = simulator->get_mutable_integrator();
  integrator.set_maximum_step_size(FLAGS_max_time_step);
  if (!integrator.get_fixed_step_mode())
    integrator.set_target_accuracy(FLAGS_accuracy);
  return integrator;
}

std::unique_ptr<Simulator<double>> MakeSimulatorFromGflags(
    const System<double>& system, std::unique_ptr<Context<double>> context) {
  auto simulator =
      std::make_unique<Simulator<double>>(system, std::move(context));
  ResetIntegratorFromGflags(simulator.get());
  simulator->set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator->Initialize();
  return simulator;
}

}  // namespace systems
}  // namespace drake
