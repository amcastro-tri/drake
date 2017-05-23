#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

/// Base class for specific Mobilizer implementations with the number of
/// generalized positions and velocities resolved at compile time as template
/// parameters. This allows specific mobilizer implementations to only work on
/// fixed-size Eigen expressions therefore allowing for optimized operations on
/// fixed-size matrices. In addition, this layer discourages the proliferation
/// of dynamic-sized Eigen matrices that would otherwise lead to run-time
/// dynamic memory allocations.
/// %MobilizerImpl also provides a number of size specific methods to retrieve
/// multibody quantities of interest from caching structures. These are common
/// to all mobilizer implementations and therefore they live in this class.
/// Users should not need to interact with this class directly unless they need
/// to implement a custom Mobilizer class.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T, int num_positions, int num_velocities>
class MobilizerImpl : public Mobilizer<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilizerImpl)

  /// As with Mobilizer this the only constructor available for this base class.
  /// The minimum amount of information that we need to define a mobilizer is
  /// the knowledge of the inboard and outboard frames it connects.
  /// Subclasses of %MobilizerImpl are therefore forced to provide this
  /// information in their respective constructors.
  MobilizerImpl(const Frame<T>& inboard_frame,
                const Frame<T>& outboard_frame) :
      Mobilizer<T>(inboard_frame, outboard_frame) {}

  /// Returns the number of generalized coordinates granted by this mobilizer.
  int get_num_positions() const final { return nq;}

  /// Returns the number of generalized velocities granted by this mobilizer.
  int get_num_velocities() const final { return nv;}

  /// Sets the what is considered the _zero_ configuration for this mobilizer.
  /// By default this method sets all degrees of freedom related to this
  /// mobilizer to zero.
  /// In general setting all generalized coordinates to zero does not represent
  /// the _zero_ configuration and it might even not represent a mathematicaly
  /// valid configuration. Consider for instance a QuaternionMobilizer, for
  /// which its _zero_ configuration corresponds to the quaternion [1, 0, 0, 0].
  /// For those cases the specific mobilizers must override this method.
  virtual void set_zero_configuration(systems::Context<T>* context) const {
    auto mbt_context = dynamic_cast<MultibodyTreeContext<T>*>(context);
    DRAKE_DEMAND(mbt_context != nullptr);
    get_mutable_positions(mbt_context).setZero();
  }

 protected:
  // Handy enum to grant specific implementations compile time sizes.
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  enum : int {nq = num_positions, nv = num_velocities};

  /// Given a mutable MultibodyTreeContext this method regurns a mutable
  /// reference vector to the portion of the generalized coordinates vector for
  /// the entire MultibodyTree that correspods to this mobilizer.
  /// The returned vector has the proper static size for fast computations.
  Eigen::VectorBlock<VectorX<T>, nq> get_mutable_positions(
      MultibodyTreeContext<T>* context) const {
    return context->template get_mutable_positions_segment<nq>(this->get_positions_start());
  }
};

}  // namespace multibody
}  // namespace drake
