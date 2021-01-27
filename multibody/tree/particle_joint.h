#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/particle_mobilizer.h"

namespace drake {
namespace multibody {

/// This joint models a planar joint allowing two bodies to translate and rotate
/// relative to one another in a plane with three degrees of freedom.
/// That is, given a frame F attached to the parent body P and a frame M
/// attached to the child body B (see the Joint class's documentation), this
/// joint allows frame M to translate within the x-y plane of frame F and to
/// rotate about the z-axis, with M's z-axis Mz and F's z-axis Fz coincident at
/// all times. The translations along the x- and y-axes of F, the rotation about
/// the z-axis and their rates specify the state of the joint.
/// Zero (x, y, θ) corresponds to frames F and M being coincident and aligned.
/// Translation (x, y) is defined to be positive in the direction of the
/// respective axes and the rotation θ is defined to be positive according to
/// the right-hand-rule with the thumb aligned in the direction of frame F's
/// z-axis.
///
/// @tparam_default_scalar
template <typename T>
class ParticleJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParticleJoint)

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  /// The name for this Joint type.
  static constexpr char kTypeName[] = "particle";

  /// Constructor to create a planar joint between two bodies so that frame F
  /// attached to the parent body P and frame M attached to the child body B
  /// translate and rotate as described in the class's documentation.
  /// This constructor signature creates a joint with no joint limits, i.e. the
  /// joint position, velocity and acceleration limits are the pair `(-∞, ∞)`.
  /// These can be set using the Joint methods set_position_limits(),
  /// set_velocity_limits() and set_acceleration_limits().
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅s/m for translation and N⋅m⋅s for
  ///   rotation, used to model losses within the joint. See documentation of
  ///   damping() for details on modelling of the damping force and torque.
  /// @throws std::exception if any element of damping is negative.
  ParticleJoint(const std::string& name, const Frame<T>& frame_on_parent,
              const Frame<T>& frame_on_child)
      : Joint<T>(name, frame_on_parent, frame_on_child,
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity())) {
  }

  const std::string& type_name() const final {
    static const never_destroyed<std::string> name{kTypeName};
    return name.access();
  }

  /// @name Context-dependent value access
  /// @{

  /// Gets the position of `this` joint from `context`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @retval p_FoMo_F The position of `this` joint stored in the `context`
  ///                  ordered as (x, y). See class documentation for details.
  Vector3<T> get_position(const Context<T>& context) const {
    return get_mobilizer()->get_position(context);
  }

  /// Sets the `context` so that the position of `this` joint equals `p_FoMo_F`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] p_FoMo_F The desired position in meters to be stored in
  ///                     `context` ordered as (x, y). See class documentation
  ///                     for details.
  /// @returns a constant reference to `this` joint.
  const ParticleJoint<T>& set_position(Context<T>* context,
                                        const Vector3<T>& p_FoMo_F) const {
    get_mobilizer()->set_position(context, p_FoMo_F);
    return *this;
  }

  /// Gets the translational velocity v_FoMo_F, in meters per second, of `this`
  /// joint's Mo measured and expressed in frame F from `context`.
  /// @param[in] context The context of the model this joint belongs to.
  /// @retval v_FoMo_F The translational velocity of `this` joint as stored in
  ///                  the `context`.
  Vector3<T> get_translational_velocity(
      const systems::Context<T>& context) const {
    return get_mobilizer()->get_translational_velocity(context);
  }

  /// Sets the translational velocity, in meters per second, of this `this`
  /// joint's Mo measured and expressed in frame F  to `v_FoMo_F`. The new
  /// translational velocity gets stored in `context`.
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] v_FoMo_F The desired translational velocity of `this` joint in
  ///                     meters per second.
  /// @returns a constant reference to `this` joint.
  const ParticleJoint<T>& set_translational_velocity(
      systems::Context<T>* context, const Vector3<T>& v_FoMo_F) const {
    get_mobilizer()->set_translational_velocity(context, v_FoMo_F);
    return *this;
  }

  /// @}

  /// Gets the default position for `this` joint.
  /// @retval p_FoMo_F The default position of `this` joint.
  Vector3<double> get_default_translation() const {
    return this->default_positions();
  }

  /// Sets the default position of this joint.
  /// @param[in] p_FoMo_F The desired default position of the joint
  void set_default_translation(const Vector3<double>& p_FoMo_F) {
    this->set_default_positions(p_FoMo_F);
  }

 private:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Therefore arguments were already checked to be valid.
  /// For a ParticleJoint, we must always have `joint_dof < 3` since there are
  /// three degrees of freedom (num_velocities() == 3). `joint_tau` is the force
  /// applied along the x-axis of the parent  frame F if `joint_dof = 0`, the
  /// force applied along the y-axis of the parent frame F if `joint_dof = 1`,
  /// or the torque about the z-axis of the parent frame F if `joint_dof = 2`.
  /// The force is applied to the body declared as child (according to the
  /// planar joint's constructor) at the origin of the child frame M. The force
  /// is defined to be positive in the direction of the selected axis and the
  /// torque is defined to be positive according to the right hand rule about
  /// the selected axis. That is, a positive force causes a positive
  /// translational acceleration and a positive torque causes a positive angular
  /// acceleration (of the child body frame).
  void DoAddInOneForce(const systems::Context<T>&, int joint_dof,
                       const T& joint_tau,
                       MultibodyForces<T>* forces) const final {
    DRAKE_DEMAND(joint_dof < 3);
    Eigen::VectorBlock<Eigen::Ref<VectorX<T>>> tau_mob =
        get_mobilizer()->get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    tau_mob(joint_dof) += joint_tau;
  }

  int do_get_velocity_start() const final {
    return get_mobilizer()->velocity_start_in_v();
  }

  int do_get_num_velocities() const final { return 3; }

  int do_get_position_start() const final {
    return get_mobilizer()->position_start_in_q();
  }

  int do_get_num_positions() const final { return 3; }

  void do_set_default_positions(
      const VectorX<double>& default_positions) final {
    if (this->has_implementation()) {
      get_mutable_mobilizer()->set_default_position(default_positions);
    }
  }

  // Joint<T> overrides:
  std::unique_ptr<typename Joint<T>::BluePrint> MakeImplementationBlueprint()
      const final;

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const final;

  // Make ParticleJoint templated on every other scalar type a friend of
  // ParticleJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of ParticleJoint<T>.
  template <typename>
  friend class ParticleJoint;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::ParticleMobilizer<T>* get_mobilizer() const {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    const internal::ParticleMobilizer<T>* mobilizer =
        dynamic_cast<const internal::ParticleMobilizer<T>*>(
            this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  internal::ParticleMobilizer<T>* get_mutable_mobilizer() {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    auto* mobilizer = dynamic_cast<internal::ParticleMobilizer<T>*>(
        this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ParticleJoint)
