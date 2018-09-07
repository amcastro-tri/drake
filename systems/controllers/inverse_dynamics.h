#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

// Forward declaration keeps us from including RBT headers that significantly
// slow compilation.
template <class T>
class RigidBodyTree;

namespace drake {
namespace systems {
namespace controllers {

/**
 * Solves inverse dynamics with no consideration for under actuation, joint
 * torque limits, or closed kinematic chains. The system also provides a pure
 * gravity compensation mode. This system provides a BasicVector
 * input port for the state `(q, v)`, where `q` is the generalized
 * position and `v` is the generalized velocity, and a BasicVector output port
 * for the computed torque. There is an additional BasicVector input port for
 * desired acceleration when configured to be **not** in pure gravity
 * compensation mode.


 There is also a pure gravity compensation mode,
 * in which torque is computed as `g(q)`. This system always has an BasicVector
 * input port for the state `(q, v)` and an BasicVector output port for the
 * computed torque. There is an additional BasicVector input port for desired
 * acceleration when configured to be not in pure gravity compensation mode.

 *
 * InverseDynamicsController uses a PID controller to generate desired
 * acceleration and uses this class to compute torque. This class should be used
 * directly if desired acceleration is computed differently.
 *
 * @tparam T The vector element type, which must be a valid Eigen scalar.
 *
 * Instantiated templates for the following kinds of T's are provided:
 * - double
 */
template <typename T>
class InverseDynamics : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InverseDynamics)

  /**
   * Computes inverse dynamics for `tree`, where the computed torque
   * is `H(q) * vd_d + c(q, v) + g(q)`, where `H` is the inertia matrix, `c` is
   * the Coriolis terms, `g` is the gravity term, `q` is the generalized
   * position, `v` is the generalized velocity and `vd_d` is the desired
   * generalized acceleration. In gravity compensation mode, torque is computed
   * as `g(q)`.
   * @param tree Pointer to the model. The life span of @p tree must be longer
   * than this instance.
   * @param pure_gravity_compensation If set to true, this instance will only
   * consider the gravity term. It also will NOT have the desired acceleration
   * input port.
   */
  InverseDynamics(const RigidBodyTree<T>* tree, bool pure_gravity_compensation);

  // @TODO(edrumwri) Find a cleaner way of approaching the consideration of
  // external forces. I like to imagine a dichotomy of approaches for
  // construction of this system: incorporating *no* external forces or all
  // forces on the plant. The current approach does neither: it only pledges to
  // account for exactly the forces that MultibodyTree does.
  /**
   * Computes inverse dynamics for `plant`, where the computed torque is
   * `H(q) * vd_d + f`, and `H` is the inertia matrix, `c` is the Coriolis term,
   * `q` is the generalized position, `v` is the generalized velocity,
   * `vd_d` is the desired generalized acceleration, and `f` is the result
   * computed using MultibodyTree::CalcForceElementsContribution() given the
   * passed-in model, `q`, and `v`. In gravity compensation mode, torque is
   * computed using MultibodyTree::CalcGravityGeneralizedForces().
   * @param plant Pointer to the plant. The life span of @p plant must be
   * longer than that of this instance.
   * @param parameters The parameters corresponding to this plant.
   * @param pure_gravity_compensation If set to true, this instance will only
   * consider the gravity term. It also will NOT have the desired acceleration
   * input port.
   * @pre The plant must be finalized (i.e., plant.is_finalized() must return
   * `true`).
   */
  InverseDynamics(const multibody::multibody_plant::MultibodyPlant<T>* plant,
                  const systems::Parameters<T>& parameters,
                  bool pure_gravity_compensation);

  /**
   * Returns the input port for the estimated state.
   */
  const InputPort<T>& get_input_port_estimated_state() const {
    return this->get_input_port(input_port_index_state_);
  }

  /**
   * Returns the input port for the desired acceleration.
   */
  const InputPort<T>& get_input_port_desired_acceleration() const {
    DRAKE_DEMAND(!pure_gravity_compensation_);
    return this->get_input_port(input_port_index_desired_acceleration_);
  }

  /**
   * Returns the output port for the actuation torques.
   */
  const OutputPort<T>& get_output_port_torque() const {
    return this->get_output_port(output_port_index_torque_);
  }

  bool is_pure_gravity_compenstation() const {
    return pure_gravity_compensation_;
  }

 private:
  // This is the calculator method for the output port.
  void CalcOutputTorque(const Context<T>& context,
                        BasicVector<T>* torque) const;

  const RigidBodyTree<T>* rigid_body_tree_{nullptr};
  const multibody::multibody_plant::MultibodyPlant<T>* multibody_plant_{
      nullptr};
  const bool pure_gravity_compensation_{false};

  // This context is used solely for setting generalized positions and
  // velocities in multibody_plant_.
  std::unique_ptr<Context<T>> multibody_plant_context_;

  int input_port_index_state_{0};
  int input_port_index_desired_acceleration_{0};
  int output_port_index_torque_{0};

  const int q_dim_{0};
  const int v_dim_{0};
  const int act_dim_{0};
};

}  // namespace controllers
}  // namespace systems
}  // namespace drake
