#include "drake/multibody/contact_solvers/contact_problem.h"

#include <algorithm>

#include "drake/common/sorted_pair.h"

using drake::SortedPair;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

ContactProblemGraph ContactProblemGraph::MakeGraphOfParticipatingCliques(
    std::vector<int>* participating_cliques) const {
  std::vector<bool> clique_participates(num_cliques(), false);
  for (const auto& e : edges_) {
    if (e.cliques.first() >= 0) clique_participates[e.cliques.first()] = true;
    if (e.cliques.second() >= 0) clique_participates[e.cliques.second()] = true;
  }
  const int num_participating_cliques =
      std::count(clique_participates.begin(), clique_participates.end(), true);
  participating_cliques->resize(num_participating_cliques);

  // We build a map from participating clique index to original index in `this`
  // graph. We mark non-participating cliques with -1.
  std::vector<int> participating_clique_index(num_cliques(), -1);
  int c_participating = 0;
  for (int c = 0; c < num_cliques(); ++c) {
    if (clique_participates[c]) {
      participating_clique_index[c] = c_participating;
      (*participating_cliques)[c_participating++] = c;
    }
  }
  if (num_participating_cliques == num_cliques()) return *this;

  ContactProblemGraph participating_cliques_graph(num_participating_cliques,
                                                  num_edges());
  for (const auto& e : edges_) {
    const int c0 = e.cliques.first() >= 0
                       ? participating_clique_index[e.cliques.first()]
                       : -1;
    const int c1 = e.cliques.second() >= 0
                       ? participating_clique_index[e.cliques.second()]
                       : -1;
    Edge participating_edge({c0, c1}, e.constraints_index);
    participating_cliques_graph.AddEdge(std::move(participating_edge));
  }
  return participating_cliques_graph;
}

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
