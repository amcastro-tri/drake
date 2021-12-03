#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"

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

  SapConstraint(int clique, MatrixX<T>&& J)
      : clique0_(clique), J0_(std::move(J)) {
    DRAKE_DEMAND(clique >= 0);
  }

  SapConstraint(int clique0, int clique1, MatrixX<T>&& J0, MatrixX<T>&& J1)
      : clique0_(clique0),
        clique1_(clique1),
        J0_(std::move(J0)),
        J1_(std::move(J1)) {
    DRAKE_DEMAND(clique0 >= 0);
    DRAKE_DEMAND(clique1 >= 0);
    DRAKE_DEMAND(clique0 != clique1);
    DRAKE_DEMAND(J0.rows() == J1.rows());
  }

  int num_cliques() const { return clique1_ < 0 ? 1 : 2; }

  int clique0() const { return clique0_; }
  int clique1() const { return clique1_; }

  const MatrixX<T>& clique0_jacobian() const { return J0_; }
  const MatrixX<T>& clique1_jacobian() const { return J1_; }

  virtual VectorX<T> Project(const Eigen::Ref<const VectorX<T>>& y,
                             std::optional<MatrixX<T>> dPdy) const = 0;

  // VectorX<T> CalcConstraintBias(const T& wi);
  // VectorX<T> CalcRegularizationParameters(const T& wi);

 private:
  // For now we limit ourselves to constraints between two cliques only.
  int clique0_{-1};
  int clique1_{-1};
  MatrixX<T> J0_;
  MatrixX<T> J1_;
};

template <typename T>
class SapFrictionConeConstraint final : public SapConstraint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapFrictionConeConstraint);

  SapFrictionConeConstraint(int clique, MatrixX<T>&& J, const T& mu)
      : SapConstraint<T>(clique, std::move(J)), mu_(mu) {
    DRAKE_DEMAND(mu >= 0.0);
    DRAKE_DEMAND(this->clique0_jacobian().rows() == 3);
  }

  SapFrictionConeConstraint(int clique0, int clique1, MatrixX<T>&& J0,
                            MatrixX<T>&& J1, const T& mu)
      : SapConstraint<T>(clique0, clique1, std::move(J0), std::move(J1)),
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
    for (const auto& Ac : A_) {
      DRAKE_DEMAND(Ac.rows() == Ac.cols());
      nv_ += Ac.rows();
    }
    DRAKE_DEMAND(v_star_.size() == nv_);
  }

  SapContactProblem(
      std::vector<MatrixX<T>>&& A, VectorX<T>&& v_star,
      std::vector<std::unique_ptr<SapConstraint<T>>>&& constraints)
      : A_(std::move(A)),
        v_star_(std::move(v_star)),
        constraints_(std::move(constraints)) {
    for (const auto& Ac : A_) {
      DRAKE_DEMAND(Ac.rows() == Ac.cols());
      nv_ += Ac.rows();
    }
    DRAKE_DEMAND(v_star_.size() == nv_);
  }

  void AddConstraint(std::unique_ptr<SapConstraint<T>> c) {
    DRAKE_DEMAND(c->clique0() < num_cliques());
    DRAKE_DEMAND(c->clique1() < num_cliques());
    DRAKE_DEMAND(c->clique0_jacobian().cols() == num_velocities(c->clique0()));
    if (c->clique1() >= 0) {
      DRAKE_DEMAND(c->clique0_jacobian().cols() ==
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

  ContactProblemGraph MakeGraph() const;

 private:
  int nv_{0};
  std::vector<MatrixX<T>> A_;
  VectorX<T> v_star_;
  std::vector<std::unique_ptr<SapConstraint<T>>> constraints_;
};

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
  const SapContactProblem<T>* problem_{nullptr};
  // Graph corresponding to `problem_`.
  ContactProblemGraph graph_;
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
