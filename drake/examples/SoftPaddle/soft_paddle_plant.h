#pragma once

#include <memory>

#include "drake/examples/SoftPaddle/soft_paddle_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace soft_paddle {

/// A model of a simple pendulum
/// @f[ ml^2 \ddot\theta + b\dot\theta + mgl\sin\theta = u @f]
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template <typename T>
class SoftPaddlePlant : public systems::LeafSystem<T> {
 public:
  SoftPaddlePlant();
  ~SoftPaddlePlant() override;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;
  using MyOutput = systems::SystemOutput<T>;

  /// The input force to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  /// Returns the input port to the externally applied torque.
  const systems::SystemPortDescriptor<T>& get_tau_port() const;

  /// Returns the port to output state.
  const systems::SystemPortDescriptor<T>& get_output_port() const;

  /// Returns the port to output state.
  const systems::SystemPortDescriptor<T>& get_visualizer_output_port() const;

#if 0
  void set_theta(MyContext* context, const T& theta) const {
    get_mutable_state(context)->set_theta(theta);
  }

  void set_thetadot(MyContext* context, const T& thetadot) const {
    get_mutable_state(context)->set_thetadot(thetadot);
  }
#endif

  void EvalOutput(const MyContext& context, MyOutput* output) const override;

  void EvalTimeDerivatives(const MyContext& context,
                           MyContinuousState* derivatives) const override;

  /// Paddle mass in Kg.
  const T& mp() const { return mp_; }
  /// Paddle length in meters.
  const T& ell() const { return ell_; }
  /// Gravity in m/s^2.
  const T& g() const { return g_; }

  explicit SoftPaddlePlant(const SoftPaddlePlant& other) = delete;
  SoftPaddlePlant& operator=(const SoftPaddlePlant& other) = delete;
  explicit SoftPaddlePlant(SoftPaddlePlant&& other) = delete;
  SoftPaddlePlant& operator=(SoftPaddlePlant&& other) = delete;

 protected:
  // LeafSystem<T> override.
  std::unique_ptr<MyContinuousState>
  AllocateContinuousState() const override;

  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;

  // System<T> override.
  SoftPaddlePlant<AutoDiffXd>* DoToAutoDiffXd() const override;

 private:
  T get_tau(const MyContext& context) const {
    return this->EvalVectorInput(context, 0)->GetAtIndex(0);
  }

  static const SoftPaddleStateVector<T>& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const SoftPaddleStateVector<T>&>(cstate.get_vector());
  }

  static SoftPaddleStateVector<T>* get_mutable_state(
      MyContinuousState* cstate) {
    return dynamic_cast<SoftPaddleStateVector<T>*>(cstate->get_mutable_vector());
  }

  static SoftPaddleStateVector<T>* get_mutable_output(MyOutput* output) {
    return dynamic_cast<SoftPaddleStateVector<T>*>(
        output->GetMutableVectorData(0));
  }

  static const SoftPaddleStateVector<T>& get_state(const MyContext& context) {
    return get_state(*context.get_continuous_state());
  }

  static SoftPaddleStateVector<T>* get_mutable_state(MyContext* context) {
    return get_mutable_state(context->get_mutable_continuous_state());
  }

  T g_{9.81};  // Acceleration of gravity.

  // Paddle parameters.
  T Ip_{0.0};  // Paddle moment of inertia. [kg * m^2]
  T ell_{0.7}; // Paddle/rubber band length. [m]
  T mp_{0.0};  // Paddle mass. [Kg]
  T T0_{10.0};  // Rubber band tension. [N]

  // Disk parameters.
  T Rd_{0.08};  // Disk's radius. [m]
  T md_{0.1};  // Disk's mass. [Kg]
  T Id_{0.5 * md_ * Rd_ * Rd_};  // Disk's moment of inertia. [Kg * m^2]
};

}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake
