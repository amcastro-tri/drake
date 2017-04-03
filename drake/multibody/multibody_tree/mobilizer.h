#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class Body;
template<typename T> class BodyNode;

template <typename T>
class Mobilizer : public MultibodyTreeElement<Mobilizer<T>, MobilizerIndex> {
 public:
  /// Mobilizer constructor.
  Mobilizer(const MaterialFrame<T>& inboard_frame,
            const MaterialFrame<T>& outboard_frame) :
      inboard_frame_(inboard_frame), outboard_frame_(outboard_frame) {
    // Bodies must have already been added to a multibody tree.
    DRAKE_DEMAND(inboard_frame.get_index().is_valid());
    DRAKE_DEMAND(outboard_frame.get_index().is_valid());
  }

  MobilizerIndex get_index() const final { return topology_.id;}

  virtual int get_num_positions() const = 0;

  virtual int get_num_velocities() const = 0;

  BodyIndex get_inboard_body_id() const { return inboard_frame_.get_body().get_index(); }

  BodyIndex get_outboard_body_id() const { return outboard_frame_.get_body().get_index(); }

  FrameIndex get_inboard_frame_id() const { return inboard_frame_.get_index(); }

  FrameIndex get_outboard_frame_id() const { return outboard_frame_.get_index(); }


  const MaterialFrame<T>& get_inboard_frame() const {
    return inboard_frame_;
  }

  const MaterialFrame<T>& get_outboard_frame() const {
    return outboard_frame_;
  }

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

  /// Computes the time derivative of H_FM in the F frame.
  virtual void CalcAcrossMobilizerVelocityJacobianDot(
      const MultibodyTreeContext<T>& context,
      VelocityKinematicsCache<T>* vc) const = 0;

  virtual void CalcQDot(const MultibodyTreeContext<T>& context,
                        Eigen::Ref<VectorX<T>> qdot) const = 0;

  const MobilizerTopology& get_topology() const { return topology_;}

  /// Sets the topological information for this mobilizer. Topological
  /// information includes connectivity such as inboard and outboard frames and
  /// bodies as well as degrees of freedom indexing information.
  /// This is an implementation detail. User code should never call it.
  void SetTopology(const MobilizerTopology& topology) {
    topology_ = topology;
  }

  virtual std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const BodyNodeTopology& topology,
      const Body<T>* body, const Mobilizer<T>* mobilizer) const = 0;

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
  const MaterialFrame<T>& inboard_frame_;
  const MaterialFrame<T>& outboard_frame_;
  MobilizerTopology topology_{MobilizerIndex(0), FrameIndex(0), FrameIndex(0)};

  int get_positions_start() const { return topology_.positions_start; }
  int get_velocities_start() const { return topology_.velocities_start; }

  void set_index(MobilizerIndex id) override {
    MultibodyTreeElement<Mobilizer<T>, MobilizerIndex>::set_index(id);
    topology_.id = id;
  }
};

}  // namespace multibody
}  // namespace drake
