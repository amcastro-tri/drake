#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace soft_robots {
namespace drake_fem {

template <typename T>
class Mesh {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Mesh);

  Mesh(const MatrixX<T>& nodes, const MatrixX<int> connectivities) :
      nodes_(nodes), connectivities_(connectivities) {}

  int get_num_elements() const { return connectivities_.cols(); }

  int get_num_nodes() const { return nodes_.cols(); }

  int get_num_physical_dimensions() const { return nodes_.rows(); }

 private:
  // Each column is a node in the mesh.
  MatrixX<T> nodes_;
  // Each column holds the nodes for a given element.
  MatrixX<int> connectivities_;
};

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake