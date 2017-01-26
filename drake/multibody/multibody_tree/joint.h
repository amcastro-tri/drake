#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_cache.h"
#include "drake/multibody/multibody_tree/multibody_context.h"
#include "multibody_context.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <typename T>
class Joint {
 public:
  /// This is the type of the across joint velocity Jacobian or *hinge
  /// map matrix* as defined by A. Jain. The maximim number of coluns for this
  /// Jacobian is 6.
  typedef Eigen::Matrix<T, 6, Eigen::Dynamic, Eigen::ColMajor, 6, 6>
      SpatialVelocityJacobian;

  //typedef const Eigen::Ref<const VectorX<T>>& ConstRefVectorX;

  /// Joint constructor. This API forces users writing their own joints having
  /// to provides at least this minimum information.
  Joint(const Body<T>& parent_body, const Body<T>& child_body,
        const Eigen::Isometry3d& X_PF, const Eigen::Isometry3d& X_BM) :
      X_PF_(X_PF), X_BM_(X_BM), X_MB_(X_BM.inverse()) {
    // Bodies must have already been added to a multibody tree.
    DRAKE_DEMAND(parent_body.get_id().is_valid());
    DRAKE_DEMAND(child_body.get_id().is_valid());
    DRAKE_DEMAND(parent_body.get_id() != child_body.get_id());
    topology_.inboard_body = parent_body.get_id();
    topology_.outboard_body = child_body.get_id();
  }

  virtual int get_num_qs() const = 0;

  virtual int get_num_vs() const = 0;

  /// Sets the parent tree of this body.
  /// This methods needs to be public so that MultibodyTree::AddJoint() can
  /// access it. However it is dangerous to expose it to users.
  /// Users should never call this method.
  void set_parent_tree(MultibodyTree<T>* parent);

  const JointTopology& get_topology() const { return topology_;}

  void set_start_indexes(int position_start, int velocity_start) {
    indexes_.position_start = position_start;
    indexes_.num_positions = get_num_qs();
    indexes_.velocity_start = velocity_start;
    indexes_.num_velocities = get_num_vs();
  }

  //const Eigen::VectorBlock<const VectorX<T>>
  auto get_positions_slice(
      const Eigen::Ref<const VectorX<T>>& q) const {
    return q.segment(indexes_.position_start, get_num_qs());
  }

#if 0
  const Body<T>& get_inboard_body() const {
    DRAKE_ASSERT(parent_tree_ != nullptr &&
        "Joint was not added to a MultibodyTree.");
    return parent_tree_->get_joint_inboard_body(*this);
  }
#endif

  const Isometry3<T>& getX_PF() const { return X_PF_; }
  const Isometry3<T>& getX_BM() const { return X_BM_; }
  const Isometry3<T>& getX_MB() const { return X_MB_; }

  /// Computes the across-joint transform `X_FM(q)` ginven the vector of
  /// generalized postions `q`.
  /// This method can be considered the *definition* of a given mobilizer.
  virtual Isometry3<T> CalcAcrossJointTransform(
      const Eigen::Ref<const VectorX<T>>& q) const = 0;

  /// Computes the across joint velocity jacobian @p Ht as defined by A. Jain.
  /// This Jacobian defines the spatial velocity subspace so that the spatial
  /// velocity of frame `M` with respect to frame `F`, and expressed in `F`, is:
  ///   `V_FM_F = Ht * v`
  /// with `v` the vector of generalized velocities for this mobilizer.
  virtual void CalcAcrossJointVelocityJacobian(
      const Eigen::Ref<const VectorX<T>>& q,
      Eigen::Ref<MatrixX<T>> Ht) const = 0;

  virtual void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const = 0;

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
  // Joint inboard frame F in inboard body frame P.
  Isometry3<T> X_PF_{Isometry3<T>::Identity()};
  // Joint outboard frame M in outboard body B.
  Isometry3<T> X_BM_{Isometry3<T>::Identity()};
  Isometry3<T> X_MB_{Isometry3<T>::Identity()};

  // Joint's topology.
  MultibodyTree<T>* parent_tree_{nullptr};
  JointTopology topology_;
  JointIndexesInfo indexes_;
};

}  // namespace multibody
}  // namespace drake
