#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <typename T>
class Joint {
 public:
  /// Joint constructor. This API forces users writing their own joints having
  /// to provides at least this minimum information.
  Joint(const Body<T>& parent_body, const Body<T>& child_body,
        const Eigen::Isometry3d& X_PF, const Eigen::Isometry3d& X_BM) :
      X_PF_(X_PF), X_BM_(X_BM) {
    // Bodies must have already been added to a multibody tree.
    DRAKE_DEMAND(parent_body.get_id().is_valid());
    DRAKE_DEMAND(child_body.get_id().is_valid());
  }

  virtual int get_num_dofs() const = 0;

  /// Sets the parent tree of this body.
  /// This methods needs to be public so that MultibodyTree::AddJoint() can
  /// access it. However it is dangerous to expose it to users.
  /// Users should never call this method.
  void set_parent_tree(MultibodyTree<T>* parent);

#if 0
  const Body<T>& get_inboard_body() const {
    DRAKE_ASSERT(parent_tree_ != nullptr &&
        "Joint was not added to a MultibodyTree.");
    return parent_tree_->get_joint_inboard_body(*this);
  }
#endif

  const Isometry3<T>& get_pose_in_parent() const { return X_PF_; }
  const Isometry3<T>& get_pose_in_child() const { return X_BM_; }

#if 0
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
#endif

 protected:
  MultibodyTree<T>* parent_tree_{nullptr};
  Isometry3<T> X_PF_{Isometry3<T>::Identity()};
  Isometry3<T> X_BM_{Isometry3<T>::Identity()};
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
#endif

template <typename T>
class RevoluteJoint : public Joint<T> {
  using Joint<T>::parent_tree_;
 public:
  /// Creates a revolute joint with axis_F expressed in the inboard frame F.
  RevoluteJoint(const Body<T>& parent_body, const Body<T>& child_body,
                const Eigen::Isometry3d& X_PF, const Eigen::Isometry3d& X_BM,
                const Vector3<double> axis_F) :
      Joint<T>(parent_body, child_body, X_PF, X_BM), axis_F_(axis_F) {}

  int get_num_dofs() const final { return 1; }

 private:

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_F_;
};

}  // namespace multibody
}  // namespace drake
