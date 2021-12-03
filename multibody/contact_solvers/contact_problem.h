#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Cliques are the nodes.
// Edges are a bundle of constraints that connect two cliques.
class ContactProblemGraph {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactProblemGraph);

  struct Edge {
    Edge(const drake::SortedPair<int>& cliques_in,
         std::vector<int>&& constraints_in)
        : cliques(cliques_in), constraints_index(std::move(constraints_in)) {}
    Edge(const drake::SortedPair<int>& cliques_in,
         const std::vector<int>& constraints_in)
        : cliques(cliques_in), constraints_index(constraints_in) {}
    drake::SortedPair<int> cliques;
    std::vector<int> constraints_index;
  };

  ContactProblemGraph(int num_cliques, int num_edges)
      : num_cliques_(num_cliques) {
    edges_.reserve(num_edges);
  }

  int AddEdge(Edge&& e) {
    const int edge_index = edges_.size();
    num_constraints_ += e.constraints_index.size();
    edges_.emplace_back(e);
    return edge_index;
  }

  int num_cliques() const { return num_cliques_; }
  int num_edges() const { return static_cast<int>(edges_.size()); }
  int num_constraints() const { return num_constraints_; }

  const std::vector<Edge>& edges() const { return edges_;  }

  const Edge& get_edge(int e) const {
    DRAKE_DEMAND(0 <= e && e < num_edges());
    return edges_[e];
  }

  ContactProblemGraph MakeGraphOfParticipatingCliques(
      std::vector<int>* participating_cliques) const;

 private:
  int num_cliques_{0};
  int num_constraints_{0};
  std::vector<Edge> edges_;
};

template <typename T>
class SapConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapConstraint);

  // Default constructor allow us to place SapConstraint objects in STL
  // containers.
  SapConstraint() = default;

  virtual ~SapConstraint() = default;

  SapConstraint(int clique, const MatrixX<T>& J)
      : clique0_(clique), J0_(J) {
    DRAKE_DEMAND(clique >= 0);
    num_constrained_dofs_ = J.rows();
  }

  SapConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                const MatrixX<T>& J1)
      : clique0_(clique0),
        clique1_(clique1),
        J0_(J0),
        J1_(J1) {
    DRAKE_DEMAND(clique0 >= 0);
    DRAKE_DEMAND(clique1 >= 0);
    DRAKE_DEMAND(clique0 != clique1);
    DRAKE_DEMAND(J0.rows() == J1.rows());
    num_constrained_dofs_ = J0.rows();
  }

  int num_cliques() const { return clique1_ < 0 ? 1 : 2; }

  int clique0() const { return clique0_; }
  int clique1() const { return clique1_; }

  const MatrixX<T>& clique0_jacobian() const { return J0_; }
  const MatrixX<T>& clique1_jacobian() const { return J1_; }

  virtual VectorX<T> Project(const Eigen::Ref<const VectorX<T>>& y,
                             std::optional<MatrixX<T>> dPdy) const = 0;

  int num_constrained_dofs() const { return num_constrained_dofs_; }

  // VectorX<T> CalcConstraintBias(const T& wi);
  // VectorX<T> CalcRegularizationParameters(const T& wi);

 private:
  // For now we limit ourselves to constraints between two cliques only.
  int clique0_{-1};
  int clique1_{-1};
  MatrixX<T> J0_;
  MatrixX<T> J1_;
  int num_constrained_dofs_{0};
};

template <typename T>
class SapFrictionConeConstraint final : public SapConstraint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapFrictionConeConstraint);

  SapFrictionConeConstraint(int clique, const MatrixX<T>& J, const T& mu)
      : SapConstraint<T>(clique, J), mu_(mu) {
    DRAKE_DEMAND(mu >= 0.0);
    DRAKE_DEMAND(this->clique0_jacobian().rows() == 3);
  }

  SapFrictionConeConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                            const MatrixX<T>& J1, const T& mu)
      : SapConstraint<T>(clique0, clique1, J0, J1),
        mu_(mu) {
    DRAKE_DEMAND(mu >= 0.0);
    DRAKE_DEMAND(this->clique0_jacobian().rows() == 3);
    DRAKE_DEMAND(this->clique1_jacobian().rows() == 3);
  }

  VectorX<T> Project(const Eigen::Ref<const VectorX<T>>&,
                     std::optional<MatrixX<T>> dPdy) const final {
    if (dPdy) {
      DRAKE_ASSERT(dPdy->rows() == 3);
      DRAKE_ASSERT(dPdy->cols() == 3);
      dPdy->setZero();
    }
    return Vector3<T>();
  }

 private:
  T mu_{0.0};
};

// SAP Problem defined by:
//   - A⋅(v−v*) = Jᵀ⋅γ
//   - Constraints.
template <typename T>
class SapContactProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapContactProblem);

  SapContactProblem(
      std::vector<MatrixX<T>>&& A, VectorX<T>&& v_star)
      : A_(std::move(A)),
        v_star_(std::move(v_star)) {
    nv_ = 0;
    for (const auto& Ac : A_) {
      DRAKE_DEMAND(Ac.rows() == Ac.cols());
      PRINT_VAR(Ac.rows());
      nv_ += Ac.rows();
    }
    PRINT_VAR(nv_);
    PRINT_VAR(v_star_.size());
    DRAKE_DEMAND(v_star_.size() == nv_);
  }

  SapContactProblem(
      std::vector<MatrixX<T>>&& A, VectorX<T>&& v_star,
      std::vector<std::unique_ptr<SapConstraint<T>>>&& constraints)
      : A_(std::move(A)),
        v_star_(std::move(v_star)),
        constraints_(std::move(constraints)) {
    nv_ = 0;
    for (const auto& Ac : A_) {
      DRAKE_DEMAND(Ac.rows() == Ac.cols());
      nv_ += Ac.rows();
    }
    PRINT_VAR(nv_);
    PRINT_VAR(v_star_.size());
    DRAKE_DEMAND(v_star_.size() == nv_);
  }

  void AddConstraint(std::unique_ptr<SapConstraint<T>> c) {
    DRAKE_DEMAND(c->clique0() < num_cliques());
    DRAKE_DEMAND(c->clique1() < num_cliques());
    DRAKE_DEMAND(c->clique0_jacobian().cols() == num_velocities(c->clique0()));
    if (c->clique1() >= 0) {
      DRAKE_DEMAND(c->clique1_jacobian().cols() ==
                   num_velocities(c->clique1()));
    }
    constraints_.push_back(std::move(c));
  }

  int num_cliques() const { return A_.size(); }

  int num_constraints() const { return constraints_.size(); }

  int num_velocities() const { return nv_; }

  int num_velocities(int clique) const {
    DRAKE_DEMAND(0 <= clique && clique < num_cliques());
    return A_[clique].rows();
  }

  const SapConstraint<T>& get_constraint(int k) const {
    DRAKE_DEMAND(0 <= k && k < num_constraints());
    return *constraints_[k];
  }

  ContactProblemGraph MakeGraph() const;

 private:
  int nv_{0};
  std::vector<MatrixX<T>> A_;
  VectorX<T> v_star_;
  std::vector<std::unique_ptr<SapConstraint<T>>> constraints_;
};

// TODO: this might be better to be a SapSolver method. `graph` must make
// reference to all constraints in `problem`. However, `graph` might only make
// reference to a subset of cliques in `problem` (for participating cliques
// only.)
template <typename T>
BlockSparseMatrix<T> MakeConstraintsBundleJacobian(
    const SapContactProblem<T>& problem,
    const std::vector<int>& participating_cliques,
    const ContactProblemGraph& participating_cliques_graph) {
  // We have at most two blocks per row, and one row per edge in the graph.
  const int non_zero_blocks_capacity =
      2 * participating_cliques_graph.num_edges();
  BlockSparseMatrixBuilder<T> builder(participating_cliques_graph.num_edges(),
                                      participating_cliques_graph.num_cliques(),
                                      non_zero_blocks_capacity);

  std::vector<int> edge_dofs(participating_cliques_graph.num_edges(), 0);
  for (int e = 0; e < participating_cliques_graph.num_edges(); ++e) {
    const auto& edge = participating_cliques_graph.get_edge(e);
    for (int k : edge.constraints_index) {
      const SapConstraint<T>& c = problem.get_constraint(k);
      edge_dofs[e] += c.num_constrained_dofs();
    }
  }

  for (int block_row = 0; block_row < participating_cliques_graph.num_edges();
       ++block_row) {
    const auto& e = participating_cliques_graph.get_edge(block_row);
    const int participating_c0 = e.cliques.first();
    const int participating_c1 = e.cliques.second();
    const int c0 =
        participating_c0 >= 0 ? participating_cliques[participating_c0] : -1;
    // At least one clique must be valid per graph edge.
    // Since c0 < c1, then c1 must always be valid.
    DRAKE_DEMAND(participating_c1 >= 0);
    const int c1 = participating_cliques[participating_c1];

    // Allocate Jacobian blocks for this edge.
    MatrixX<T> J0, J1;
    const int num_rows = edge_dofs[block_row];
    if (c0 >= 0) {
      const int nv0 = problem.num_velocities(c0);
      J0.resize(num_rows, nv0);
    }
    const int nv1 = problem.num_velocities(c1);
    J1.resize(num_rows, nv1);

    int row_start = 0;
    for (int k : e.constraints_index) {
      const SapConstraint<T>& c = problem.get_constraint(k);
      const int nk = c.num_constrained_dofs();

      // N.B. Each edge stores its cliques as a sorted pair. However, the pair
      // of cliques in the original constraints can be in arbitrary order.
      // Therefore below we must check to what clique in the original constraint
      // the edge's cliques correspond to.

      if (c0 >= 0) {
        J0.middleRows(row_start, nk) =
            c0 == c.clique0() ? c.clique0_jacobian() : c.clique1_jacobian();
      }

      J1.middleRows(row_start, nk) =
          c1 == c.clique0() ? c.clique0_jacobian() : c.clique1_jacobian();

      row_start += nk;
    }

    if (c0 >= 0) {
      builder.PushBlock(block_row, participating_c0, J0);
    }
    builder.PushBlock(block_row, participating_c1, J1);
  }

  return builder.Build();
}

// TODO: Move this to the SapSolver source. It'd seem specific to SAP to
// rearrange constraints in the order it needs them. I just need to come up with
// a good way to document it and describe its invariants.
template <typename T>
class SapConstraintsBundle {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapConstraintsBundle);

  // We keep a reference to `problem` and its data.
  SapConstraintsBundle(const SapContactProblem<T>* problem,
                       const ContactProblemGraph& graph);

 private:
#if 0
  const SapContactProblem<T>* problem_{nullptr};
  // Graph corresponding to `problem_`.
  ContactProblemGraph graph_;
#endif  
  // Jacobian for the entire bundle, with graph_.num_edges() block rows and
  // graph_.num_cliques() block columns.
  BlockSparseMatrix<T> J_;
  // problem_ constraint references in the order dictated by the
  // ContactProblemGraph.
  std::vector<SapConstraint<T>*> constraints_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
