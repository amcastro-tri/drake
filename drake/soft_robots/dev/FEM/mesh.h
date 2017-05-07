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

  int get_num_element_nodes() const { return connectivities_.rows(); }

  int get_num_nodes() const { return nodes_.cols(); }

  int get_num_physical_dimensions() const { return nodes_.rows(); }

  Eigen::Block<const MatrixX<int>> get_element_connectivities(int element) const
  {
    return connectivities_.block(0, element, get_num_element_nodes(), 1);
  }

  void GetNodes(
      const Eigen::Ref<const VectorX<int>> &nodes,
      Eigen::Ref<MatrixX < T>> xa) const {
    DRAKE_ASSERT(xa.rows() == get_num_physical_dimensions());
    DRAKE_ASSERT(xa.cols() == nodes.size());
    for (int i = 0; i < nodes.size(); ++i) {
      xa.col(i) = nodes_.col(nodes(i));
    }
  }

  void GetElementNodes(
      int element,
      Eigen::Ref<MatrixX < T>> xa) const {
    return GetNodes(connectivities_.col(element), xa);
  }

 private:
  // Each column is a node in the mesh.
  MatrixX<T> nodes_;
  // Each column holds the nodes for a given element.
  MatrixX<int> connectivities_;
};

}  // namespace drake_fem
}  // namespace soft_robots
}  // namespace drake