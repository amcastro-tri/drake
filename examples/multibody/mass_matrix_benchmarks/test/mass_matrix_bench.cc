#include <iostream>

#include <benchmark/benchmark.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/tools/performance/fixture_common.h"
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

using drake::multibody::MultibodyPlant;
using drake::symbolic::Expression;
using drake::systems::Context;
using drake::test::LimitMalloc;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace {

// @note LimitMalloc: This program uses LimitMalloc to indicate the count of
// malloc calls measured at the time the benchmark cases were written. At best,
// they are empirical observations. If there is a good reason to exceed these
// limits, maintainers should not hesitate to change them. If they are exceeded
// without a good reason, maintainers should revisit their changes to see why
// heap usage has increased.

// Track and report simple streaming statistics on allocations. Variance
// tracking is adapted from:
// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
class AllocationTracker {
 public:
  AllocationTracker() {}

  void Report(benchmark::State* state) {
    state->counters["Allocs.min"] = min_;
    state->counters["Allocs.max"] = max_;
    state->counters["Allocs.mean"] = mean_;
    state->counters["Allocs.stddev"] =
        updates_ < 2 ? std::numeric_limits<double>::quiet_NaN()
                     : std::sqrt(m2_ / (updates_ - 1));
  }

  void Update(int allocs) {
    min_ = std::min(min_, allocs);
    max_ = std::max(max_, allocs);
    ++updates_;
    double delta = allocs - mean_;
    mean_ += delta / updates_;
    m2_ += delta * (allocs - mean_);
  }

 private:
  int min_{std::numeric_limits<int>::max()};
  int max_{std::numeric_limits<int>::min()};
  int updates_{};
  double mean_{};
  double m2_{};
};

void CalcLtdlInPlace(const std::vector<int>& lambda, MatrixXd* Ainout) {
  DRAKE_DEMAND(Ainout->rows() == Ainout->cols());
  auto& A = *Ainout;
  const int n = A.rows();
  for (int k = n - 1; k >= 0; --k) {
    int i = lambda[k];
    while (i >= 0) {
      const double a = A(k, i) / A(k, k);
      int j = i;
      while (j >= 0) {
        A(i, j) -= a * A(k, j);
        j = lambda[j];
      }
      A(k, i) = a;
      i = lambda[i];
    }
  }
}

// Fixture that holds a Cassie robot model in a MultibodyPlant<double>. The
// class also holds a default context for the plant, and dimensions of its
// state and inputs.
class AllegroHandFixture : public benchmark::Fixture {
 public:
  AllegroHandFixture() { tools::performance::AddMinMaxStatistics(this); }

  // This apparently futile using statement works around "overloaded virtual"
  // errors in g++. All of this is a consequence of the weird deprecation of
  // const-ref State versions of SetUp() and TearDown() in benchmark.h.
  using benchmark::Fixture::SetUp;
  void SetUp(benchmark::State&) override {
    plant_ = std::make_unique<MultibodyPlant<double>>(0);

    multibody::Parser parser(plant_.get());
    const auto& model =
        "drake/manipulation/models/"
        "allegro_hand_description/sdf/allegro_hand_description_right.sdf";
    parser.AddModelFromFile(FindResourceOrThrow(model));
    // Weld the hand to the world frame
    plant_->WeldFrames(plant_->world_frame(),
                       plant_->GetBodyByName("hand_root").body_frame());
    plant_->Finalize();

    nq_ = plant_->num_positions();
    nv_ = plant_->num_velocities();
    nu_ = plant_->num_actuators();

    context_ = plant_->CreateDefaultContext();

    u_ = VectorXd::Zero(nu_);

    // Use default state to avoid problems with all-zero quaternions.
    x_ = context_->get_continuous_state_vector().CopyToVector();
  }

  // Use this method to invalidate state-dependent computations within each
  // benchmarked step. Disabling the cache entirely could affect the performance
  // differently because it would suppress any internal use of the cache during
  // complicated computations like forward dynamics. For example, if there are
  // multiple places in forward dynamics that access body positions, currently
  // those would get computed once and re-used (like in real applications) but
  // with caching off they would get recalculated repeatedly, affecting the
  // timing results.
  virtual void InvalidateState() { context_->NoteContinuousStateChange(); }

  // Makes array lambda(i) described in [Featherstone, 2014].
  // For node i in the tree, lambda(i) corresponds to the parent node of node i.
  std::vector<int> MakeParentArray() const {
    const multibody::internal::MultibodyTreeTopology& topology =
        multibody::internal::GetInternalTree(*plant_).get_topology();
    // Lambda does not include the root (node 0).
    std::vector<int> lambda(topology.get_num_body_nodes() - 1);
    for (multibody::internal::BodyNodeIndex node_index(1);
         node_index < topology.get_num_body_nodes(); ++node_index) {
      const multibody::internal::BodyNodeTopology& node =
          topology.get_body_node(node_index);
      lambda[node_index - 1] = node.parent_body_node - 1;
    }
    return lambda;
  }

 protected:
  AllocationTracker tracker_;
  std::unique_ptr<MultibodyPlant<double>> plant_{};
  std::unique_ptr<Context<double>> context_;
  int nq_{};
  int nv_{};
  int nu_{};
  VectorXd u_{};
  VectorXd x_{};
};

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AllegroHandFixture, MassMatrix)(benchmark::State& state) {
  PRINT_VAR(nq_);
  PRINT_VAR(nv_);
  PRINT_VAR(nu_);
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);
  PRINT_VARn(M);
  const auto lambda = MakeParentArray();

  CalcLtdlInPlace(lambda, &M);
  PRINT_VARn(M);

  for (size_t i = 0; i < lambda.size(); ++i) {
    std::cout << fmt::format("lambda({}) = {}\n", i, lambda[i]);
  }
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard({.max_num_allocations = 0});
    InvalidateState();
    plant_->CalcMassMatrix(*context_, &M);
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AllegroHandFixture, CalcLtdlInPlace)(benchmark::State& state) {
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);
  const auto lambda = MakeParentArray();
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard({.max_num_allocations = 0});
    CalcLtdlInPlace(lambda, &M);
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

}  // namespace
}  // namespace examples
}  // namespace drake

BENCHMARK_MAIN();
