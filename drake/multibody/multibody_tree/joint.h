#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"

namespace drake {
namespace multibody {

template <typename T>
class Joint {
 public:
  Joint(parent_tree, const Isometry3d& X_PF, const Isometry3d& X_BM) :
      parent_tree_(parent_tree), X_PF_(X_PF), X_BM_(X_BM) { }

  virtual int get_num_dofs() const = 0;

  const Body<T>& get_inboard_body() const {
    DRAKE_ASSERT(parent_tree_ != nullptr &&
        "Joint was not added to a MultibodyTree.");
    return parent_tree_->get_joint_inboard_body(*this);
  }

  const Isometry3<T>& get_pose_in_parent() const { return X_PF; }
  const Isometry3<T>& get_pose_in_child() const { return X_BM; }

  /// Computes `qdot = N(q) * u` with `N(q)` the the kinematic coupling matrix.
  /// `N(q)` can be cached in the @p context.
  virtual void CalcQDot(
      const Context<T>& context,
      Eigen::Ref<const MatrixX<T>> u, Eigen::Ref<MatrixX<T>> qdot) const = 0;

  /// Returns the transform `X_FM` from the outboard frame `M` to the inboard
  /// frame `F`.
  /// This method is an NVI to DoCalcOutboardFameToInboardFrameTranform() so
  /// that valid cached entries do not need to be recomputed.
  Isometry3<T>& CalcOutboardFameToInboardFrameTranform(
      const Context<T>& context) {
    /*
    Cache<T>* cache = context.get_mutable_cache();
    if (!cache->is_valid(my_ticket)) {
      cache->get_mutable_entry(my_ticket).set_value(
          DoCalcOutboardFametoInboardFrameTranform(context));
    }
    return cache->get_entry(my_ticket).value<Isometry3<T>>();
    */
    return DoCalcOutboardFameToInboardFrameTranform(context);
  }

  Isometry3<T>& DoCalcOutboardFameToInboardFrameTranform(
      const Context<T>& context) = 0;

 private:
  MultibodyTree<T>* parent_tree_;
  Isometry3<T> X_PF_;
  Isometry3<T> X_BM_;
};

#if 0
// Should we template on number of dof's to allow compiler optimizations?
// See Simbody's RigidBodyNodeSpec.h
template <typename T, int dofs>
class JointWithOptions : public Joint<T> {
 public:
    int get_num_dofs() const { return dofs; }

    /// Computes `qdot = N(q) * v, where `N(q)` is the kinematic coupling
    /// matrix.
    void CalcQDot(const Context<T>&,
                  Eigen::Ref<const MatrixX<T>> u,
                  Eigen::Ref<MatrixX<T>> qdot) const override {
      qdot.segment<ndofs>() = u.segment<ndofs>(); // default says qdot=u
    }
};

template <typename T>
class RevoluteJoint : public JointWithOptions<T, 1> {
 public:
  static RevoluteJoint<T>* CreateJoint(
      MultibodyTree<T>* parent_tree,
      const Body<T>& parent_body, const Isometry3d& X_PF,
      const Body<T>& child_body, const Isometry3d& X_BM,
      const Vector3<double>& axis_F) : JointWithOptions<T, 1>(parent_tree, X_PF, X_BM) {
    DRAKE_ASSERT(parent_tree != nullptr);
    // QUESTION: what is the simplest shape you can give to this method so that
    // you can simplify the job to users writing their own joints?
    // Idea 1: rework parent joint constructor in initializer list.
    // Idea 2: rework MBT::AddJoint method below.

    auto joint = std::make_unique<RevoluteJoint<T>>(axis_F);
    // Notice that here we could probably pass more like a "topological" joint
    // just to set connectivities. Good idea? Maybe a JointNode?
    return parent_tree_->AddJoint(joint, parent_body, child_body);
  }

 private:
  // Creates a revolute joint with axis_F expressed in the inboard frame F.
  RevoluteJoint(const Vector3<double> axis_F) { }

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_F;
};
#endif

}  // namespace multibody
}  // namespace drake
