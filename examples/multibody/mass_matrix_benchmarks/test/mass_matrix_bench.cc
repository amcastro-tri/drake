#include <iostream>

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <benchmark/benchmark.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/examples/multibody/mass_matrix_benchmarks/test/featherstone_ltdl.h"
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

/* Bench for a model of the anthropomorphic Allegro hand sketched below along with its connectivity graph.
There are 4 finger with 4 degrees of freedom (DOFs) each, for a total of 16 DOFs. DOFs are assigned as in [Featherstone, 2005] so that the parent array satisfies λ(i) < i. In the case of the hand model welded to the world by the palm, the parent array is:
   λ = {-1, -1, -1, -1, 
         0,  1,  2,  3, 
         4,  5,  6,  7,
         8,  9, 10, 11,
        12, 13, 14, 15}
 
where we use 0-based index and the invalid index -1 refers to the world.

                                    12● 13● 14● 15●
         ┌─┐   ┌─┐   ┌─┐              │   │   │   │
         │ │   │ │   │ │              │   │   │   │
       12├─┤ 13├─┤ 14├─┤             8●  9● 10● 11●
         │ │   │ │   │ │              │   │   │   │
        8├─┤  9├─┤ 10├─┤              │   │   │   │
         │ │   │ │   │ │             4●  5●  6●  7●
        4├─┤  5├─┤  6├─┤              │   │   │   │
         ├─┴───┴─┴───┴─┤              │   │   │   │
         │0     1     2│             0●  1●  2●  3●
         │             ├┬──┬──┬──┐    │   │   │   │
         │            3││  │  │  │    └───┴─┬─┴───┘
         └─────────────┴┴──┴──┴──┘          │
                        7  11 15          -1◯
       a) Allegro hand schematic    b) Connectivity graph.
 
When the model is not welded to the world there is a 6-DOFs joint connecting the palm to the world. Therefore the "extended" parent array λ(i) looks like:


                     18● 19● 20● 21●
                       │   │   │   │
                       │   │   │   │
                     14● 15● 16● 17●
                       │   │   │   │
                       │   │   │   │
                     10● 11● 12● 13●
                       │   │   │   │
                       │   │   │   │
                      6●  7●  8●  9●
                       │   │   │   │
                       └───┴─┬─┴───┘
                             │              0
                             ●──●──●──●──●──●
                             5  4  3  2  1  │
                                            │
                                          -1◯

where the first 6-DOFs branch between the world and the fingers correspond to the free floating palm of the hand. Refer to [Featherstone, 2005] for details.


[Featherstone, 2005] Efficient factorization of the joint-space inertia matrix
for branched kinematic trees. The International Journal of Robotics Research,
24(6), pp.487-500.

*/
class AllegroHandFixture : public benchmark::Fixture {
 public:
  AllegroHandFixture() { tools::performance::AddMinMaxStatistics(this); }

  void MakeModel(bool weld) {
    plant_ = std::make_unique<MultibodyPlant<double>>(0);

    multibody::Parser parser(plant_.get());
    const auto& model =
        "drake/manipulation/models/"
        "allegro_hand_description/sdf/allegro_hand_description_right.sdf";
    parser.AddModelFromFile(FindResourceOrThrow(model));
    if (weld) {
      plant_->WeldFrames(plant_->world_frame(),
                         plant_->GetBodyByName("hand_root").body_frame());
    }
    plant_->Finalize();

    nq_ = plant_->num_positions();
    nv_ = plant_->num_velocities();
    nu_ = plant_->num_actuators();

    context_ = plant_->CreateDefaultContext();

    u_ = VectorXd::Zero(nu_);

    // Use default state to avoid problems with all-zero quaternions.
    x_ = context_->get_continuous_state_vector().CopyToVector();
  }

  // This apparently futile using statement works around "overloaded virtual"
  // errors in g++. All of this is a consequence of the weird deprecation of
  // const-ref State versions of SetUp() and TearDown() in benchmark.h.
  using benchmark::Fixture::SetUp;
  void SetUp(benchmark::State&) override {}

  // Use this method to invalidate state-dependent computations within each
  // benchmarked step. Disabling the cache entirely could affect the performance
  // differently because it would suppress any internal use of the cache during
  // complicated computations like forward dynamics. For example, if there are
  // multiple places in forward dynamics that access body positions, currently
  // those would get computed once and re-used (like in real applications) but
  // with caching off they would get recalculated repeatedly, affecting the
  // timing results.
  virtual void InvalidateState() { context_->NoteContinuousStateChange(); }

  // Makes expanded array lambda(i) as described in [Featherstone, 2005].
  // Unlike [Featherstone, 2005], we use 0 based indexing. The root of the tree
  // (the world) is marked with DOF -1.
  // For node i in the (expanded) tree, lambda(i) corresponds to the parent node
  // of node i.
  // For details refere to Section 2 in [Featherstone, 2005] and Figures 1 and
  // 2.
  std::vector<int> MakeExpandedParentArray() const {
    const multibody::internal::MultibodyTreeTopology& topology =
        multibody::internal::GetInternalTree(*plant_).get_topology();
    std::vector<int> lambda;
    lambda.reserve(topology.num_velocities());

    for (multibody::internal::BodyNodeIndex node_index(1);
         node_index < topology.get_num_body_nodes(); ++node_index) {
      const multibody::internal::BodyNodeTopology& node =
          topology.get_body_node(node_index);
      const multibody::internal::BodyNodeTopology& parent_node =
          topology.get_body_node(node.parent_body_node);
      for (int m = 0; m < node.num_mobilizer_velocities; ++m) {
        const int parent_node_last_dof =
            parent_node.mobilizer_velocities_start_in_v +
            parent_node.num_mobilizer_velocities - 1;
        lambda.push_back(parent_node_last_dof + m);
      }
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
  MakeModel(true);
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);

  const auto lambda = MakeExpandedParentArray();
  test::CalcLtdlInPlace(lambda, &M);

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
  MakeModel(true);
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);
  const auto lambda = MakeExpandedParentArray();
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard({.max_num_allocations = 0});
    test::CalcLtdlInPlace(lambda, &M);
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AllegroHandFixture, EigenDenseCholesky)(benchmark::State& state) {
  MakeModel(true);
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard(
        {.max_num_allocations = -1});  // Do not limit allocations.
    M.ldlt();
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AllegroHandFixture, EigenSparseCholesky)(benchmark::State& state) {
  MakeModel(true);
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);
  Eigen::SparseMatrix<double> Msp = M.sparseView(1.0, 1.0e-14);
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard(
        {.max_num_allocations = -1});  // Do not limit allocations.
    solver.compute(Msp);
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AllegroHandFixture, FloatingBaseMassMatrix)
(benchmark::State& state) {
  MakeModel(false);
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);

  const auto lambda = MakeExpandedParentArray();
  test::CalcLtdlInPlace(lambda, &M);

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
BENCHMARK_F(AllegroHandFixture, FloatingBaseCalcLtdlInPlace)
(benchmark::State& state) {
  MakeModel(false);
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);
  const auto lambda = MakeExpandedParentArray();
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard({.max_num_allocations = 0});
    test::CalcLtdlInPlace(lambda, &M);
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AllegroHandFixture, FloatingBaseEigenDenseCholesky)
(benchmark::State& state) {
  MakeModel(false);
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard(
        {.max_num_allocations = -1});  // Do not limit allocations.
    M.ldlt();
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(AllegroHandFixture, FloatingBaseEigenSparseCholesky)
(benchmark::State& state) {
  MakeModel(false);
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);
  Eigen::SparseMatrix<double> Msp = M.sparseView(1.0, 1.0e-14);
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  for (auto _ : state) {
    // @see LimitMalloc note above.
    LimitMalloc guard(
        {.max_num_allocations = -1});  // Do not limit allocations.
    solver.compute(Msp);
    tracker_.Update(guard.num_allocations());
  }
  tracker_.Report(&state);
}

}  // namespace
}  // namespace examples
}  // namespace drake

BENCHMARK_MAIN();
