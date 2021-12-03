#include "drake/multibody/contact_solvers/contact_problem.h"

#include "drake/common/sorted_pair.h"

using drake::SortedPair;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
ContactProblemGraph SapContactProblem<T>::MakeGraph() const {
  std::unordered_map<SortedPair<int>, std::vector<int>> edge_constraints;
  for (size_t k = 0; k < constraints_.size(); ++k) {
    const auto& c = constraints_[k];
    const auto cliques_pair = drake::MakeSortedPair(c->clique0(), c->clique1());
    edge_constraints[cliques_pair].push_back(k);
  }

  const int num_edges = edge_constraints.size();
  ContactProblemGraph graph(num_cliques(), num_edges);
  for (auto& e : edge_constraints) {
    graph.AddEdge(ContactProblemGraph::Edge(e.first, std::move(e.second)));
  }

  return graph;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::SapContactProblem<
    double>;
