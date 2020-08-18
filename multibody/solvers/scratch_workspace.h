#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace solvers {

template <typename T>
class GrantScratchWorkspaceAccess;

template <typename T>
class ScratchWorkspace {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScratchWorkspace)  

  ScratchWorkspace(int nv, int nc, int max_vectors)
      : nv_(nv), nc_(nc), max_vectors_(max_vectors) {}

  /// Number of vectors of size 3 * nc currently being referenced.
  int num_xc_vectors() const { return xc_top_; }

  /// The current capacity for xc vectors. The maximum number of xc vectors so
  /// far requested.
  int xc_vectors_capacity() const { return xc_pool_.size(); }

 private:
  friend class GrantScratchWorkspaceAccess<T>;

  // Pushes a vector of size 3*nc into the stack and returns a reference to
  // it.
  VectorX<T>& push_xc_sized_vector() {
    // PRINT_VAR(xc_top_);
    return AccessVectorPoolAndMaybeAllocate(xc_top_++, 3 * nc_, &xc_pool_);
  }

  // Pops `count`  xc sized vectors from the stack.
  void pop_xc_sized_vectors(int count) { xc_top_ -= count; }

  VectorX<T>& push_xn_sized_vector() {
    // PRINT_VAR(xn_top_);
    return AccessVectorPoolAndMaybeAllocate(xn_top_++, nc_, &xn_pool_);
  }
  void pop_xn_sized_vectors(int count) { xn_top_ -= count; }

  VectorX<T>& push_xt_sized_vector() {
    // PRINT_VAR(xt_top_);
    return AccessVectorPoolAndMaybeAllocate(xt_top_++, 2 * nc_, &xt_pool_);
  }
  void pop_xt_sized_vectors(int count) { xt_top_ -= count; }

  VectorX<T>& push_v_sized_vector() {
    // PRINT_VAR(v_top_);
    return AccessVectorPoolAndMaybeAllocate(v_top_++, nv_, &v_pool_);
  }
  void pop_v_sized_vectors(int count) { v_top_ -= count; }

  VectorX<T>& AccessVectorPoolAndMaybeAllocate(
      int i, int vector_size, std::vector<std::unique_ptr<VectorX<T>>>* pool) {
    if (i < static_cast<int>(pool->size())) {
      return *pool->at(i);
    } else {
      if (max_vectors_ == static_cast<int>(pool->size())) {
        throw std::runtime_error("Workspace reached maximum capacity.");
      }
      return *pool->emplace_back(std::make_unique<VectorX<T>>(vector_size));
    }
  }

  int nv_, nc_;
  int max_vectors_;
  int xc_top_{0};
  int xn_top_{0};
  int xt_top_{0};
  int v_top_{0};
  // We save pointers instead of the actual objects so that whenever a
  // .push_back() happens the actual vectors are not destroyed.
  std::vector<std::unique_ptr<VectorX<T>>>
      xc_pool_;  // pool of vectors of size 3*nc.
  std::vector<std::unique_ptr<VectorX<T>>>
      xn_pool_;  // pool of vectors of size nc.
  std::vector<std::unique_ptr<VectorX<T>>>
      xt_pool_;  // pool of vectors of size 2*nc.
  std::vector<std::unique_ptr<VectorX<T>>>
      v_pool_;  // pool of vectors of size nv.
};

template <typename T>
class GrantScratchWorkspaceAccess {
 public:
  /// Resets counter access for w.
  GrantScratchWorkspaceAccess(ScratchWorkspace<T>& w) : w_(w) {}

  /// Resets counter access for w.
  ~GrantScratchWorkspaceAccess() {
    w_.pop_xc_sized_vectors(xc_counter_);
    w_.pop_xn_sized_vectors(xn_counter_);
    w_.pop_xt_sized_vectors(xt_counter_);
    w_.pop_v_sized_vectors(v_counter_);
  }

  // Access an i-th pool for vectors of size 3*nc.
  VectorX<T>& xc_sized_vector() {
    ++xc_counter_;
    return w_.push_xc_sized_vector();
  }

  VectorX<T>& xn_sized_vector() {
    ++xn_counter_;
    return w_.push_xn_sized_vector();
  }

  VectorX<T>& xt_sized_vector() {
    ++xt_counter_;
    return w_.push_xt_sized_vector();
  }

  VectorX<T>& v_sized_vector() {
    ++v_counter_;
    return w_.push_v_sized_vector();
  }

 private:
  ScratchWorkspace<T>& w_;
  int xc_counter_{0};
  int xn_counter_{0};
  int xt_counter_{0};
  int v_counter_{0};
};

}  // namespace solvers
}  // namespace multibody
}  // namespace drake