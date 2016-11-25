#pragma once

#include <cstdint>
#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace examples {
namespace soft_paddle {

/// A source block with a constant output port at all times.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @ingroup primitive_systems
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
template <typename T>
class PaddleMirrorLawSystem : public systems::LeafSystem<T> {
 public:
  /// Constructs a system with a vector output that is constant and equals the
  /// supplied @p source_value at all times.
  /// @param source_value the constant value of the output so that
  /// `y = source_value` at all times.
  PaddleMirrorLawSystem(const T& phi0, const T& amplitude);

  bool has_any_direct_feedthrough() const override { return false; }

  /// Outputs a signal with a fixed value as specified by the user.
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  /// Returns the output port to the constant source.
  const systems::SystemPortDescriptor<T>& get_paddle_angle_port() const;

 private:
  T phi0_{0.0};
  T amplitude_{0.1};
};

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
