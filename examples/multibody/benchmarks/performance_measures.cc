#include <algorithm>
#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <random>

#include <fmt/format.h>
#include <gflags/gflags.h>

#include "drake/examples/multibody/benchmarks/flops_estimator.h"

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

int do_main() {
  FlopsEstimator estimator;

  // One billion of floating point operations would usually take in the order
  // of a second. We perform 10 tries of 1e8 operations each in order to
  // collects statistics.
  estimator.RunTests(10, 1e8);

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

