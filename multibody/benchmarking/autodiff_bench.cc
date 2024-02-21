#include <benchmark/benchmark.h>

#include "drake/common/drake_assert.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/tools/performance/fixture_common.h"
#include "drake/tools/performance/fixture_memory.h"

namespace drake {
namespace multibody {
namespace {

using math::RigidTransform;
using math::RollPitchYaw;
using math::RotationMatrix;

// In the benchmark case instantiations at the bottom of this file, we'll use
// a bitmask for the case's "Arg" to denote which quantities are in scope as
// either gradients (for T=AutoDiffXd) or variables (for T=Expression).
constexpr int kWantNoGrad   = 0x0;
constexpr int kWantGradQ    = 0x1;

template <typename T>
class nPendulum {
 public:
  nPendulum(int num_links, double link_length)
      : num_links_(num_links), link_length_(link_length) {}

  void CalcForwardKinematics(const VectorX<T>& q,
                             std::vector<RigidTransform<T>>* kinematics) const {
    DRAKE_DEMAND(q.size() == num_links_);
    std::vector<RigidTransform<T>>& X_WB = *kinematics;
    X_WB[0] = CalcJointRigidTransform(q[0]);
    for (int i = 1; i < num_links_; ++i) {
      const RigidTransform<T> X_PB = CalcJointRigidTransform(q[i]);
      X_WB[i] = X_WB[i - 1] * X_PB;
    }
  }

  void CalcRotationKinematics(
      const VectorX<T>& q, std::vector<RotationMatrix<T>>* kinematics) const {
    DRAKE_DEMAND(q.size() == num_links_);
    std::vector<RotationMatrix<T>>& R_WB = *kinematics;
    R_WB[0] = CalcJointRotationMatrix(q[0]);
    for (int i = 1; i < num_links_; ++i) {
      const RotationMatrix<T> R_PB = CalcJointRotationMatrix(q[i]);
      R_WB[i] = R_WB[i - 1] * R_PB;
    }
  }

 private:
  RigidTransform<T> CalcJointRigidTransform(const T& theta) const {
    return RigidTransform<T>(RotationMatrix<T>::MakeZRotation(theta),
                             Vector3<T>(T(link_length_), 0, 0));
  }

  RotationMatrix<T> CalcJointRotationMatrix(const T& theta) const {
    return RotationMatrix<T>::MakeZRotation(theta);
  }

  int num_links_;
  double link_length_;
};

// Fixture that holds an n-pendulum robot model and offers helper functions to
// configure the benchmark case.
template <typename T>
class nPendulumBench : public benchmark::Fixture {
 public:
  nPendulumBench() {
    tools::performance::AddMinMaxStatistics(this);
  }

  void SetUp(benchmark::State& state) override {
    SetUpNonZeroState();
    SetUpGradientsOrVariables(state);
    tools::performance::TareMemoryManager();
  }

 protected:
  // Sets the plant to have non-zero state and input. In some cases, computing
  // using zeros will not tickle the relevant paths through the code.
  void SetUpNonZeroState();

  // In the benchmark case instantiations at the bottom of this file, we'll use
  // a bitmask for the case's "Arg" to denote which quantities are in scope as
  // either gradients (for T=AutoDiffXd) or variables (for T=Expression).
  static bool want_grad_q(const benchmark::State& state) {
    return state.range(0) & kWantGradQ;
  }

  void InvalidateState() {
    q_[0] = 0.1;
  }

  // Using the "Arg" from the given benchmark state, sets up the MbP
  // state and/or input to use gradients and/or symbolic variables
  // as configured in this benchmark case.
  //
  // For T=double, any request for gradients is an error.
  // For T=AutoDiffXd, sets the specified gradients to the identity matrix.
  // For T=Expression, sets the specified quantities to symbolic variables.
  // NOLINTNEXTLINE(runtime/references)
  void SetUpGradientsOrVariables(benchmark::State& state);

  // Runs the kinematics benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoForwardKinematics(benchmark::State& state) {    
    for (auto _ : state) {
      InvalidateState();
      pendulum_.CalcForwardKinematics(q_, &kinematics_);
    }
  }

  // Runs rotations only benchmark.
  // NOLINTNEXTLINE(runtime/references)
  void DoRotationKinematics(benchmark::State& state) {    
    for (auto _ : state) {
      InvalidateState();
      pendulum_.CalcRotationKinematics(q_, &rotation_kinematics_);
    }
  }

  // The plant itself.
  const int kNumLinks{20};
  const double kLinkLength{0.2};
  const nPendulum<T> pendulum_{kNumLinks, kLinkLength};

  // The state.
  VectorX<T> q_{VectorX<T>::Zero(kNumLinks)};

  // Data used in the forward kinematics.
  std::vector<RigidTransform<T>> kinematics_;
  std::vector<RotationMatrix<T>> rotation_kinematics_;
};

using nPendulumBenchDouble = nPendulumBench<double>;
using nPendulumBenchAutoDiff = nPendulumBench<AutoDiffXd>;

template <typename T>
void nPendulumBench<T>::SetUpNonZeroState() {
  q_ = VectorX<T>::LinSpaced(kNumLinks, 0.1, 0.9);  

  // Reset temporaries.
  kinematics_ =
      std::vector<RigidTransform<T>>(kNumLinks, RigidTransform<T>::Identity());
  rotation_kinematics_ =
      std::vector<RotationMatrix<T>>(kNumLinks, RotationMatrix<T>::Identity());
}

template <>  // NOLINTNEXTLINE(runtime/references)
void nPendulumBench<double>::SetUpGradientsOrVariables(
    benchmark::State& state) {
  DRAKE_DEMAND(want_grad_q(state) == false);
}

template <>  // NOLINTNEXTLINE(runtime/references)
void nPendulumBench<AutoDiffXd>::SetUpGradientsOrVariables(
    benchmark::State& state) {
  // For the quantities destined for InitializeAutoDiff, read their default
  // values (without any gradients). For the others, leave the matrix empty.
  VectorX<double> q;
  if (want_grad_q(state)) {
    q = math::DiscardGradient(q_);
  }

  // Initialize the desired gradients.
  VectorX<AutoDiffXd> q_grad;
  q_grad = math::InitializeAutoDiff(q);

  // Write the gradients back to the plant.
  if (want_grad_q(state)) {
    q_ = q_grad;
  }
}

// All that remains is to add the sensible combinations of benchmark configs.
//
// For T=double, there's only a single config. We still use a range arg so
// that its correspondence with the non-double cases is apparent.
//
// For T=AutoDiff, the range arg sets which gradients to use, using a bitmask.
//
// For T=Expression, the range arg sets which variables to use, using a bitmask.

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(nPendulumBenchDouble, RotationKinematics)
(benchmark::State& state) {
  DoRotationKinematics(state);
}
BENCHMARK_REGISTER_F(nPendulumBenchDouble, RotationKinematics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(nPendulumBenchAutoDiff, RotationKinematics)
(benchmark::State& state) {
  DoRotationKinematics(state);
}
BENCHMARK_REGISTER_F(nPendulumBenchAutoDiff, RotationKinematics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ);        

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(nPendulumBenchDouble, ForwardKinematics)
(benchmark::State& state) {
  DoForwardKinematics(state);
}
BENCHMARK_REGISTER_F(nPendulumBenchDouble, ForwardKinematics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad);

// NOLINTNEXTLINE(runtime/references)
BENCHMARK_DEFINE_F(nPendulumBenchAutoDiff, ForwardKinematics)
(benchmark::State& state) {
  DoForwardKinematics(state);
}
BENCHMARK_REGISTER_F(nPendulumBenchAutoDiff, ForwardKinematics)
    ->Unit(benchmark::kMicrosecond)
    ->Arg(kWantNoGrad)
    ->Arg(kWantGradQ);    

}  // namespace
}  // namespace multibody
}  // namespace drake
