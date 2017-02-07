#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <typename T>
class Mobilizer : public MultibodyTreeElement<Mobilizer<T>, MobilizerIndex> {
 public:
  /// Mobilizer constructor.
  Mobilizer(const MaterialFrame<T>& inboard_frame,
            const MaterialFrame<T>& outboard_frame) {
    // Bodies must have already been added to a multibody tree.
    DRAKE_DEMAND(inboard_frame.get_id().is_valid());
    DRAKE_DEMAND(outboard_frame.get_id().is_valid());
    DRAKE_DEMAND(inboard_frame.get_id() != outboard_frame.get_id());
    topology_.inboard_frame = inboard_frame.get_id();
    topology_.outboard_frame = outboard_frame.get_id();
    topology_.inboard_body = inboard_frame.get_body_id();
    topology_.outboard_body = outboard_frame.get_body_id();
  }

  virtual int get_num_positions() const = 0;

  virtual int get_num_velocities() const = 0;

  BodyIndex get_inboard_body_id() const { return topology_.inboard_body; }

  BodyIndex get_outboard_body_id() const { return topology_.outboard_body; }

  BodyIndex get_inboard_frame_id() const { return topology_.inboard_frame; }

  BodyIndex get_outboard_frame_id() const { return topology_.outboard_frame; }

  virtual void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const = 0;

  /// Computes the across-Mobilizer transform `X_FM(q)` ginven the vector of
  /// generalized postions `q`.
  /// This method can be considered the *definition* of a given mobilizer.
  virtual void CalcAcrossMobilizerTransform(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const = 0;

  virtual void CalcAcrossMobilizerVelocityJacobian(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const = 0;

  /// Computes the across Mobilizer velocity jacobian @p Ht as defined by A. Jain.
  /// This Jacobian defines the spatial velocity subspace so that the spatial
  /// velocity of frame `M` with respect to frame `F`, and expressed in `F`, is:
  ///   `V_FM_F = Ht * v`
  /// with `v` the vector of generalized velocities for this mobilizer.
  //virtual void CalcAcrossMobilizerVelocityJacobian(
  //    const MultibodyTreeContext<T>& context,
  //    PositionKinematicsCache<T>* pc) const = 0;

#if 0
  virtual void UpdatePositionKinematicsCache(
      const MultibodyTreeContext<T>& context) const = 0;

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

  MobilizerIndex get_id() const override {
    return topology_.id;
  }

  const MobilizerTopology& get_topology() const { return topology_;}

  /// Sets the topological information for this mobilizer. Topological
  /// information includes connectivity such as inboard and outboard frames and
  /// bodies as well as degrees of freedom indexing information.
  /// This is an implementation detail. User code should never call it.
  void SetTopology(const MobilizerTopology& topology) {
    topology_ = topology;
  }

  void PrintTopology() const {
    std::cout << "Mobilizer id: " << topology_.id << std::endl;
    std::cout << "Frames (inboard, outboard): (" <<
              topology_.inboard_frame << ", " << topology_.outboard_frame
              << ")" << std::endl;
    std::cout << "Bodies (inboard, outboard): (" <<
              topology_.inboard_body << ", " << topology_.outboard_body
              << ")" << std::endl;
    std::cout << "Body node: " << topology_.body_node << std::endl;
  }

 protected:
  MobilizerTopology topology_;

  int get_positions_start() const { return topology_.positions_start; }
  int get_velocities_start() const { return topology_.velocities_start; }

  void set_id(MobilizerIndex id) override {
    MultibodyTreeElement<Mobilizer<T>, MobilizerIndex>::set_id(id);
    topology_.id = id;
  }
};

}  // namespace multibody
}  // namespace drake
