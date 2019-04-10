#include <algorithm>
#include <iostream>
#include <fstream>
#include <functional>
#include <memory>
#include <chrono>
#include <random>
#include <type_traits>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/examples/multibody/benchmarks/flops_estimator.h"

#include "drake/common/autodiff.h"
#include "drake/common/drake_optional.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"

#if 0
#include "drake/common/find_resource.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsing/parser.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::SpatialVelocity;

DEFINE_bool(mbp_inertia, false, "Compute M with MBP.");
DEFINE_bool(mbp_inverse_dyn, true, "Compute ID with MBP.");
DEFINE_bool(mbp_enable_caching, true, "Enable caching for MBP.");

#endif

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace examples {
namespace {

using drake::math::RigidTransform;
using drake::multibody::MultibodyForces;
using drake::multibody::MultibodyPlant;
using drake::multibody::UnitInertia;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::RotationalInertia;
using drake::multibody::SpatialInertia;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::SpatialVelocity;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using systems::Context;

double EstimateFLOPS() {
  // We'll perform a FLOPS estimation to later on normalize our performance
  // measurements.
  FlopsEstimator estimator;

  // One billion of floating point operations would usually take in the order
  // of a second. We perform several tries with a large number of operations
  // each in order to collects statistics.
  const int num_tries = 15;
  const int num_ops = 1e7;
  estimator.RunTests(num_tries, num_ops);

  // Some stats printed out.
  std::cout << " Timer resolution [s]: " << BenchTimer::resolution()
            << std::endl;

  std::cout << "Addition:\n";
  std::cout << fmt::format(" Total: {:.5f} s\n", estimator.add_timer().total());
  std::cout << fmt::format(" Mean:  {:.5f} s\n", estimator.add_timer().mean());
  std::cout << fmt::format(" StDv.: {:.5f} %\n",
      estimator.add_timer().std_dev() / estimator.add_timer().mean() * 100);
  std::cout << fmt::format(" Best:  {:.5f} s\n", estimator.add_timer().best());
  std::cout << fmt::format(" Worst: {:.5f} s\n", estimator.add_timer().worst());
  std::cout << fmt::format(" FLOPS: {:.3e}\n", estimator.add_flops());

  std::cout << "Multiplication:\n";
  std::cout << fmt::format(" Total: {:.5f} s\n", estimator.mul_timer().total());
  std::cout << fmt::format(" Mean:  {:.5f} s\n", estimator.mul_timer().mean());
  std::cout << fmt::format(" StDv.: {:.5f} %\n",
      estimator.mul_timer().std_dev() / estimator.mul_timer().mean() * 100);
  std::cout << fmt::format(" Best:  {:.5f} s\n", estimator.mul_timer().best());
  std::cout << fmt::format(" Worst: {:.5f} s\n", estimator.mul_timer().worst());
  std::cout << fmt::format(" FLOPS: {:.3e}\n", estimator.mul_flops());

  std::cout << "Division:\n";
  std::cout << fmt::format(" Total: {:.5f} s\n", estimator.div_timer().total());
  std::cout << fmt::format(" Mean:  {:.5f} s\n", estimator.div_timer().mean());
  std::cout << fmt::format(" StDv.: {:.5f} %\n",
      estimator.div_timer().std_dev() / estimator.div_timer().mean() * 100);
  std::cout << fmt::format(" Best:  {:.5f} s\n", estimator.div_timer().best());
  std::cout << fmt::format(" Worst: {:.5f} s\n", estimator.div_timer().worst());
  std::cout << fmt::format(" FLOPS: {:.3e}\n", estimator.div_flops());

  // Multiplication and addition cost the same in most (all) systems. We'll use
  // their average as our estimation of FLOPS.
  const double flops = 0.5 * (estimator.add_flops() + estimator.mul_flops());
  std::cout << fmt::format("Estimated FLOPS: {:.2e} s\n", flops);

  return flops;
}

// Make a MultibodyPlant model of a num_links chain of links joined by revolute
// joints. The chain moves in the xy plane with the revolute joints's axes
// aligned with the z axis.
std::unique_ptr<MultibodyPlant<double>> MakeChainWithRevoluteJoints(int num_links) {
  // Parameters:
  const double link_mass = 0.020;   // kg.
  const double link_length = 0.04;  // m.

  const RotationalInertia<double> I_Bcm =
      link_mass * UnitInertia<double>::ThinRod(link_length, Vector3d::UnitY());
  const Vector3d p_BoBcm(0.0, -link_length / 2.0, 0.0);
  const SpatialInertia<double> M_Bo =
      SpatialInertia<double>::MakeFromCentralInertia(link_mass, p_BoBcm, I_Bcm);

  auto plant = std::make_unique<MultibodyPlant<double>>();
  plant->set_name("ChainWithRevoluteJoints_" + std::to_string(num_links));

  // RigidTransform's implicit conversion confuses the compiler and we cannot
  // pass {} or nullopt directly. See #9865.
  const optional<RigidTransform<double>> X_BJc = nullopt;

  const RigidBody<double>* previous_body = &plant->world_body();
  for (int link_index = 0; link_index < num_links; ++link_index) {
    const std::string link_name = "link_" + std::to_string(link_index);
    const RigidBody<double>& link = plant->AddRigidBody(link_name, M_Bo);

    // We build the model so that the first link pivots about the world's
    // origin. This is the pose of the joint's child frame Jc in the "parent"
    // body frame.
    const RigidTransform<double> X_PJc =
        link_index == 0
            ? RigidTransform<double>::Identity()
            : RigidTransform<double>(Vector3d(0.0, -link_length, 0.0));

    const std::string joint_name = "joint_" + std::to_string(link_index);
    plant->AddJoint<RevoluteJoint>(
      joint_name,
      /* Shoulder inboard frame Si IS the the world frame W. */
      *previous_body, X_BJc,
      /* Link's frame IS the child body joint frame. */
      link, X_PJc,
      Vector3d::UnitZ());

    previous_body = &link;
  }

  plant->Finalize();

  return plant;
}

// Explicitly declaring benchmark_method helps the compiler figure out
// template parameters.
#define TIME_BENCHMARK(plant, benchmark, params)    \
  {                                               \
    Benchmark<T> benchmark_method = benchmark<T>; \
    TimeBenchmark(#benchmark, plant, benchmark_method, params); \
  }

struct BenchmarkParams {
  int num_tries{5};  
  int num_reps{5000};
  // The benchmarks will not be run if the model's dof's is larger than
  // max_dofs. Useful to dissable expensive tests on specific scalar types.
  int max_dofs{64};
  double max_time_secs{0.2};
};

class MultibodyBench {
 public:
  MultibodyBench() { flops_ = EstimateFLOPS(); }

  template <typename T>
  void RunBenchmarks(const MultibodyPlant<T>& plant,
                     const BenchmarkParams& params) const {
    std::cout << std::endl;
    std::cout << fmt::format(" {}<{}>: nq = {}. nv = {}\n", plant.get_name(),
                             drake::NiceTypeName::Get<T>(),
                             plant.num_positions(), plant.num_velocities());
    //if (plant.num_multibody_states() > params.max_dofs) {
    //  std::cout << fmt::format(
    //      " Benchmark is too large and won't get executed\n");
    //  return;
    //}

    TIME_BENCHMARK(plant, CalcPositionKinematics, params);
    TIME_BENCHMARK(plant, CalcVelocityKinematics, params);
    TIME_BENCHMARK(plant, CalcInverseDynamics, params);
    TIME_BENCHMARK(plant, CalcMassMatrixViaInverseDynamics, params);   
  }

 private:
  template <typename T>
  using Benchmark = std::function<void(const MultibodyPlant<T>&, Context<T>*)>;

  template <typename T>
  static void CalcPositionKinematics(const MultibodyPlant<T>& plant,
                                     Context<T>* context) {
    plant.GetMutablePositionsAndVelocities(context);
    plant.EvalPositionKinematics(*context);
  }

  template <typename T>
  static void CalcVelocityKinematics(const MultibodyPlant<T>& plant,
                                     Context<T>* context) {
    // Force invalidation.                                       
    plant.GetMutableVelocities(context);
    plant.EvalVelocityKinematics(*context);
  }

  template <typename T>
  static void CalcInverseDynamics(const MultibodyPlant<T>& plant,
                                  Context<T>* context) {
    // Force invalidation.
    //plant.GetMutablePositionsAndVelocities(context);
    const VectorX<T> vdot = VectorX<T>::Ones(plant.num_velocities());
    const MultibodyForces<T> external_forces(plant);
    plant.CalcInverseDynamics(*context, vdot, external_forces);
  }

  template <typename T>
  static void CalcMassMatrixViaInverseDynamics(const MultibodyPlant<T>& plant,
                                               Context<T>* context) {
    // Force invalidation.
    plant.GetMutablePositionsAndVelocities(context);
    MatrixX<T> M(plant.num_velocities(), plant.num_velocities());
    plant.CalcMassMatrixViaInverseDynamics(*context, &M);
  }

  template <typename T>
  static std::unique_ptr<Context<T>> CreateContext(
      const MultibodyPlant<T>& plant) {
    throw std::runtime_error(
        "No implementation available for scalar of type '" +
        drake::NiceTypeName::Get<T>() + "'.");
  }

  template <typename T>
  void TimeBenchmark(const std::string& name, const MultibodyPlant<T>& plant,
                     Benchmark<T> benchmark, const BenchmarkParams& params) const {
    auto context = CreateContext(plant);
    context->EnableCaching();
    BenchTimer timer;
    int num_reps = params.num_reps;
    for (int try_number = 1; try_number <= params.num_tries; ++try_number) {
      timer.start();
      int rep;
      BenchTimer reps_timer;
      reps_timer.start();
      for (rep = 1; rep <= num_reps; ++rep) {
        benchmark(plant, context.get());

        if (try_number == 1) {
          if (reps_timer.elapsed() > params.max_time_secs) {
            // overwrite num_reps so that the next tries use the same number.
            num_reps = rep;
            break;
          }
        }
      }
      timer.stop();      
    }

    const double benchmark_time = timer.best()  / num_reps;
    const double benchmark_flops = benchmark_time * flops_;
    const double benchmark_flops_per_dof =
        benchmark_flops / plant.num_velocities();

    std::cout << fmt::format(
        "   {}: {} reps. {:.2e} s. {:.2e} s/rep. {:.2f} reps/s. {:.3f} FLOPS/dof\n",
        name, num_reps, timer.total(), benchmark_time, 1.0 / benchmark_time,
        benchmark_flops_per_dof);
  }

  double flops_;
};

// Specialization for T = double.
template <>
std::unique_ptr<Context<double>> MultibodyBench::CreateContext<double>(
    const MultibodyPlant<double>& plant) {
  return plant.CreateDefaultContext();
}

// Specialization for T = AutoDiffXd.
// Set the state x so that we take gradients with respect to it.
template <>
std::unique_ptr<Context<AutoDiffXd>> MultibodyBench::CreateContext<AutoDiffXd>(
    const MultibodyPlant<AutoDiffXd>& plant) {
  auto context = plant.CreateDefaultContext();
  const VectorX<double> x =
      math::autoDiffToValueMatrix(plant.GetPositionsAndVelocities(*context));
  VectorX<AutoDiffXd> x_ad(plant.num_multibody_states());
  math::initializeAutoDiff(x, x_ad);
  plant.SetPositionsAndVelocities(context.get(), x_ad);
  return context;
}

// Specialization for T = symbolic::Expression.
// Set the state x so that each entry is an independent variable.
template <>
std::unique_ptr<Context<symbolic::Expression>>
MultibodyBench::CreateContext<symbolic::Expression>(
    const MultibodyPlant<symbolic::Expression>& plant) {
  auto context = plant.CreateDefaultContext();
  VectorX<symbolic::Expression> x_sym(plant.num_multibody_states());
  for (int i = 0; i < plant.num_multibody_states(); ++i) {
    x_sym[i] = symbolic::Variable("x_" + std::to_string(i));
  }
  plant.SetPositionsAndVelocities(context.get(), x_sym);
  return context;
}

template <typename T>
std::unique_ptr<MultibodyPlant<T>> MakePlantOnT(
    std::unique_ptr<MultibodyPlant<double>> plant_on_double) {
  throw std::runtime_error("No implementation available for this type.");
}

// Specialization for double.
template <>
std::unique_ptr<MultibodyPlant<double>> MakePlantOnT<double>(
    std::unique_ptr<MultibodyPlant<double>> plant_on_double) {
  return plant_on_double;
}

// Specialization for AutoDiffXd.
template <>
std::unique_ptr<MultibodyPlant<AutoDiffXd>> MakePlantOnT<AutoDiffXd>(
    std::unique_ptr<MultibodyPlant<double>> plant_on_double) {
  return systems::System<double>::ToAutoDiffXd(*plant_on_double);
}

// Specialization for symbolic::Expression.
template <>
std::unique_ptr<MultibodyPlant<symbolic::Expression>>
MakePlantOnT<symbolic::Expression>(
    std::unique_ptr<MultibodyPlant<double>> plant_on_double) {
  return systems::System<double>::ToSymbolic(*plant_on_double);
}

template <typename T>
void BenchTestAllModels(const MultibodyBench& bench,
                        const BenchmarkParams& params) {
  // ChainWithRevoluteJoints.
  for (int num_links = 8; num_links <= 64; num_links *= 2) {
    auto plant = MakeChainWithRevoluteJoints(num_links);
    auto plant_on_T = MakePlantOnT<T>(std::move(plant));
    bench.RunBenchmarks(*plant_on_T, params);
  }
}

int do_main() {
  // const double flops = EstimateFLOPS();
  
  MultibodyBench bench;

  BenchTestAllModels<double>(bench, BenchmarkParams{5, 500, 1024, 0.1});
  BenchTestAllModels<AutoDiffXd>(bench, BenchmarkParams{5, 50, 256, 1.0});
  BenchTestAllModels<symbolic::Expression>(bench, BenchmarkParams{5, 5, 256, 0.1});

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}

#if 0
int do_main() {
  const int nq = 7;
  const int num_reps = 10000;
  //const int num_autodiff_reps = 50;
  const int id_num_reps = 10 * num_reps;

  // Load a model of an Iiwa arm.
  MultibodyPlant<double> plant;
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_primitive_collision.urdf"));
  plant.AddForceElement<drake::multibody::UniformGravityFieldElement>(
      -9.81 * Eigen::Vector3d::UnitZ());
  plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("base"),
    math::RigidTransform<double>(Eigen::Vector3d::Zero()).GetAsIsometry3());
  plant.Finalize();

  auto context = plant.CreateDefaultContext();
  if (FLAGS_mbp_enable_caching) {
    context->EnableCaching();
  }

  // ===========================================================================
  // MBP MASS MATRIX
  // ===========================================================================
  if (FLAGS_mbp_inertia) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2 * nq);
    Eigen::MatrixXd M(nq, nq);
    {
      x.setLinSpaced(1, 5);
      x.segment(nq, nq).setZero();
      context->get_mutable_continuous_state_vector().SetFromVector(x);
      plant.CalcMassMatrixViaInverseDynamics(*context, &M);
      PRINT_VARn(M);
      PRINT_VAR((M - M.transpose()).norm());
    }

    auto start = my_clock::now();
    for (int i = 0; i < num_reps; i++) {
      x(0) = i;
      context->get_mutable_continuous_state_vector().SetFromVector(x);
      plant.CalcMassMatrixViaInverseDynamics(*context, &M);
    }
    auto stop = my_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(plant)" << std::to_string(num_reps) << "x inertia "
                                                                    "calculations took "
              <<
              duration.count() << " milliseconds." << std::endl;
  }


  // Build and test multibody plant w/autodiff
  // std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
  //   plant.ToAutoDiffXd();

#if 0
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(plant);

  auto context_autodiff =
    plant_autodiff->CreateDefaultContext();
  context_autodiff->EnableCaching();

  start =  my_clock::now();
  MatrixX<AutoDiffXd> M_autodiff(nq, nq);
  for (int i = 0; i < num_autodiff_reps; i++) {
    x(0) = i;
    context_autodiff->get_mutable_continuous_state_vector().
        SetFromVector(math::initializeAutoDiff(x));
    plant_autodiff->CalcMassMatrixViaInverseDynamics(
        *context_autodiff, &M_autodiff);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(plant)" << std::to_string(num_autodiff_reps) << "xinertia autodiff calculations took " <<
      duration.count() << " milliseconds." << std::endl;
#endif

  // ===========================================================================
  // RBT INVERSE DYNAMICS
  // ===========================================================================
  if (FLAGS_rbt_inverse_dyn) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2 * nq);
    Eigen::VectorXd desired_vdot(nq);
    auto start = my_clock::now();
    RigidBodyTree<double>::BodyToWrenchMap external_wrenches;

    for (int i = 0; i < id_num_reps; i++) {
      x = Eigen::VectorXd::Constant(2 * nq, i);
      desired_vdot = Eigen::VectorXd::Constant(nq, i);
      auto cache =
          rigid_body_plant.get_rigid_body_tree().doKinematics(x.head(nq),
                                                              x.tail(nq));
      rigid_body_plant.get_rigid_body_tree().inverseDynamics(cache,
                                                             external_wrenches,
                                                             desired_vdot);
    }
    auto stop = my_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(rigid_body_plant)" << std::to_string(id_num_reps) <<
              "x inverse dynamics calculations took " <<
              duration.count() << " milliseconds." << std::endl;
  }
#if 0
  start =  my_clock::now();
    RigidBodyTree<AutoDiffXd>::BodyToWrenchMap external_wrenches_autodiff;

  for (int i = 0; i < num_autodiff_reps; i++) {
    x = Eigen::VectorXd::Constant(2*nq, i);
    desired_vdot = Eigen::VectorXd::Constant(nq, i);
    auto cache = rigid_body_plant.get_rigid_body_tree().doKinematics(
        math::initializeAutoDiff(x.head(nq)),
        math::initializeAutoDiff(x.tail(nq)));
    rigid_body_plant.get_rigid_body_tree().inverseDynamics(cache,
        external_wrenches_autodiff, math::initializeAutoDiff(desired_vdot));
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(rigid_body_plant)" << std::to_string(num_autodiff_reps) <<
  "x autodiff inverse dynamics calculations took "<<
      duration.count() << " milliseconds." << std::endl;
#endif

  // ===========================================================================
  // MBP INVERSE DYNAMICS
  // ===========================================================================
  if (FLAGS_mbp_inverse_dyn) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2 * nq);
    Eigen::VectorXd desired_vdot(nq);
    multibody::MultibodyForces<double> external_forces(plant);
    std::vector<SpatialAcceleration<double>> A_WB_array(
        plant.num_bodies());
    // Creates problems with drake::SpatialForce in eigen_types.h!!
    std::vector<drake::multibody::SpatialForce<double>> F_BMo_W_array(
        plant.num_bodies());
    VectorX<double> tau_id(plant.num_velocities());

    //CALLGRIND_START_INSTRUMENTATION;
    ///////////////////////////////////
    // ID does NO heap allocation.
    auto start = my_clock::now();
    for (int i = 0; i < id_num_reps; i++) {
      x = Eigen::VectorXd::Constant(2 * nq, i);
      desired_vdot = Eigen::VectorXd::Constant(nq, i);
      context->get_mutable_continuous_state_vector().SetFromVector(x);
      {
        // I only put this as proof that no heap allocation is being done.
        test::DisableMalloc guard;
        plant.CalcInverseDynamics(*context, desired_vdot,
                                            external_forces, &A_WB_array,
                                            &F_BMo_W_array, &tau_id);
      }
    }
    auto stop = my_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(plant (no heap))" << std::to_string(id_num_reps)
              << "x inverse dynamics calculations took " << duration.count()
              << " milliseconds." << std::endl;
    //CALLGRIND_STOP_INSTRUMENTATION;
    //CALLGRIND_DUMP_STATS;

    ///////////////////////////////////
    // ID WITH heap allocation.
    start = my_clock::now();
    for (int i = 0; i < id_num_reps; i++) {
      x = Eigen::VectorXd::Constant(2 * nq, i);
      desired_vdot = Eigen::VectorXd::Constant(nq, i);
      context->get_mutable_continuous_state_vector().SetFromVector(x);
      {
        // test::DisableMalloc guard; // This guard would not pass.
        plant.CalcInverseDynamics(*context, desired_vdot,
                                            external_forces);
      }
    }
    stop = my_clock::now();
    duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(plant (with heap))" << std::to_string(id_num_reps)
              << "x inverse dynamics calculations took " << duration.count()
              << " milliseconds." << std::endl;
  }

#if 0
  start =  my_clock::now();
  multibody::MultibodyForces<AutoDiffXd> external_forces_autodiff(
      *plant_autodiff);

  for (int i = 0; i < num_autodiff_reps; i++) {
    x = Eigen::VectorXd::Constant(2*nq, i);
    desired_vdot = Eigen::VectorXd::Constant(nq, i);
    context_autodiff->get_mutable_continuous_state_vector().SetFromVector(math::initializeAutoDiff(x));
    plant_autodiff->CalcInverseDynamics(*context_autodiff, math::initializeAutoDiff(desired_vdot),
                                        external_forces_autodiff);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(plant)" << std::to_string(num_autodiff_reps) << "xautodiff inverse dynamics calculations took " <<
      duration.count() << " milliseconds." << std::endl;
#endif

  return 0;
}
#endif

