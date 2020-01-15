#pragma once

#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/simulator.h"

// Declares integrator's gflags.
DECLARE_string(integration_scheme);
DECLARE_double(max_time_step);
DECLARE_double(accuracy);

// Declares simulator gflags.
DECLARE_double(target_realtime_rate);
DECLARE_bool(publish_every_time_step);

namespace drake {
namespace systems {

/// Resets the integrator used to advanced the continuous time dynamics of the
/// system associated with `simulator` according to the gflags declared in this
/// file.
/// @param[in,out] simulator
///   On input, a valid pointer to a Simulator. On output the
///   integrator for `simulator` is reset according to the gflags declared in
///   this file.
/// @returns The newly created integrator. Access with
/// Simulator::get_integrator().
IntegratorBase<double>& ResetIntegratorFromGflags(Simulator<double>* simulator);

/// Makes a new simulator according to the gflags declared in this file.
/// @param[in] system
///   The System to be associated with the newly crated Simulator. You must
///   ensure that `system` has a longer lifetime than the new Simulator.
/// @param[in] context
///   The Context that will be used as the initial condition for the simulation;
///   otherwise the Simulator will obtain a default Context from `system`.
/// @returns The newly created Simulator.
std::unique_ptr<Simulator<double>> MakeSimulatorFromGflags(
    const System<double>& system,
    std::unique_ptr<Context<double>> context = nullptr);

}  // namespace systems
}  // namespace drake
