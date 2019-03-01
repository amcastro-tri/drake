#include <iostream>
#include <memory>
#include <chrono>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/disable_malloc.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"

#include <valgrind/callgrind.h>

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

//using drake::solvers::SolutionResult;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::SpatialVelocity;

DEFINE_bool(rbt_inertia, false, "Compute M with RBT.");
DEFINE_bool(mbp_inertia, false, "Compute M with MBP.");
DEFINE_bool(rbt_inverse_dyn, false, "Compute ID with RBT.");
DEFINE_bool(mbp_inverse_dyn, true, "Compute ID with MBP.");
DEFINE_bool(mbp_enable_caching, true, "Enable caching for MBP.");

namespace drake {
namespace examples {
namespace {

typedef std::chrono::steady_clock my_clock;
//typedef std::chrono::high_resolution_clock my_clock;

int do_main() {
  const int nq = 7;
  const int num_reps = 100000;
  //const int num_autodiff_reps = 50;

  // Build and test RigidBodyPlant
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_primitive_collision.urdf"),
    multibody::joints::kFixed, tree.get());
  systems::RigidBodyPlant<double> rigid_body_plant(std::move(tree));

  // ===========================================================================
  // RBT MASS MATRIX
  // ===========================================================================
  if (FLAGS_rbt_inertia) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2 * nq);
    {
      x.setLinSpaced(1, 5);
      auto
          cache =
          rigid_body_plant.get_rigid_body_tree().doKinematics(x.head(nq));
      auto Mrbt = rigid_body_plant.get_rigid_body_tree().massMatrix(cache);
      PRINT_VARn(Mrbt);
    }

    auto start = my_clock::now();
    for (int i = 0; i < num_reps; i++) {
      x(0) = i;
      auto cache =
          rigid_body_plant.get_rigid_body_tree().doKinematics(x.head(nq));
      rigid_body_plant.get_rigid_body_tree().massMatrix(cache);
    }
    auto stop = my_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(rigid_body_plant) " << std::to_string(num_reps) << "x "
                                                                      "inertia calculations took "
              << duration.count() << " milliseconds." <<
              std::endl;
  }

#if 0
  start =  my_clock::now();
  for (int i = 0; i < num_autodiff_reps; i++) {
    x(0) = i;
    auto cache = rigid_body_plant.get_rigid_body_tree().
        doKinematics(math::initializeAutoDiff(x.head(nq)));
    rigid_body_plant.get_rigid_body_tree().massMatrix(cache);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(rigid_body_plant)" << std::to_string(num_autodiff_reps) <<
  "x inertia autodiff calculations took "
      << duration.count() << " milliseconds." << std::endl;
#endif

  // Build and test multibody plant
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double>& multibody_plant =
      *builder.AddSystem<MultibodyPlant>();

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  multibody::Parser parser(&multibody_plant, &scene_graph);
  parser.AddModelFromFile(FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_primitive_collision.urdf"));

  multibody_plant.AddForceElement<drake::multibody::UniformGravityFieldElement>(
      -9.81 * Eigen::Vector3d::UnitZ());

  multibody_plant.WeldFrames(
    multibody_plant.world_frame(), multibody_plant.GetFrameByName("base"),
    math::RigidTransform<double>(Eigen::Vector3d::Zero()).GetAsIsometry3());
  multibody_plant.Finalize();

  auto multibody_context = multibody_plant.CreateDefaultContext();
  if (FLAGS_mbp_enable_caching) {
    multibody_context->EnableCaching();
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
      multibody_context->get_mutable_continuous_state_vector().SetFromVector(x);
      multibody_plant.CalcMassMatrixViaInverseDynamics(*multibody_context, &M);
      PRINT_VARn(M);
      PRINT_VAR((M - M.transpose()).norm());
    }

    auto start = my_clock::now();
    for (int i = 0; i < num_reps; i++) {
      x(0) = i;
      multibody_context->get_mutable_continuous_state_vector().SetFromVector(x);
      multibody_plant.CalcMassMatrixViaInverseDynamics(*multibody_context, &M);
    }
    auto stop = my_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(multibody_plant)" << std::to_string(num_reps) << "x inertia "
                                                                    "calculations took "
              <<
              duration.count() << " milliseconds." << std::endl;
  }


  // Build and test multibody plant w/autodiff
  // std::unique_ptr<MultibodyPlant<AutoDiffXd>> multibody_plant_autodiff =
  //   multibody_plant.ToAutoDiffXd();

#if 0
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> multibody_plant_autodiff =
      systems::System<double>::ToAutoDiffXd(multibody_plant);

  auto multibody_context_autodiff =
    multibody_plant_autodiff->CreateDefaultContext();
  multibody_context_autodiff->EnableCaching();

  start =  my_clock::now();
  MatrixX<AutoDiffXd> M_autodiff(nq, nq);
  for (int i = 0; i < num_autodiff_reps; i++) {
    x(0) = i;
    multibody_context_autodiff->get_mutable_continuous_state_vector().
        SetFromVector(math::initializeAutoDiff(x));
    multibody_plant_autodiff->CalcMassMatrixViaInverseDynamics(
        *multibody_context_autodiff, &M_autodiff);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant)" << std::to_string(num_autodiff_reps) << "xinertia autodiff calculations took " <<
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

    for (int i = 0; i < num_reps; i++) {
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
    std::cout << "(rigid_body_plant)" << std::to_string(num_reps) <<
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
    multibody::MultibodyForces<double> external_forces(multibody_plant);
    std::vector<SpatialAcceleration<double>> A_WB_array(
        multibody_plant.num_bodies());
    // Creates problems with drake::SpatialForce in eigen_types.h!!
    std::vector<drake::multibody::SpatialForce<double>> F_BMo_W_array(
        multibody_plant.num_bodies());
    VectorX<double> tau_id(multibody_plant.num_velocities());

    //CALLGRIND_START_INSTRUMENTATION;
    ///////////////////////////////////
    // ID does NO heap allocation.
    auto start = my_clock::now();
    for (int i = 0; i < num_reps; i++) {
      x = Eigen::VectorXd::Constant(2 * nq, i);
      desired_vdot = Eigen::VectorXd::Constant(nq, i);
      multibody_context->get_mutable_continuous_state_vector().SetFromVector(x);
      {
        // I only put this as proof that no heap allocation is being done.
        test::DisableMalloc guard;
        multibody_plant.CalcInverseDynamics(*multibody_context, desired_vdot,
                                            external_forces, &A_WB_array,
                                            &F_BMo_W_array, &tau_id);
      }
    }
    auto stop = my_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(multibody_plant (no heap))" << std::to_string(num_reps)
              << "x inverse dynamics calculations took " << duration.count()
              << " milliseconds." << std::endl;
    //CALLGRIND_STOP_INSTRUMENTATION;
    //CALLGRIND_DUMP_STATS;

    ///////////////////////////////////
    // ID WITH heap allocation.
    start = my_clock::now();
    for (int i = 0; i < num_reps; i++) {
      x = Eigen::VectorXd::Constant(2 * nq, i);
      desired_vdot = Eigen::VectorXd::Constant(nq, i);
      multibody_context->get_mutable_continuous_state_vector().SetFromVector(x);
      {
        // test::DisableMalloc guard; // This guard would not pass.
        multibody_plant.CalcInverseDynamics(*multibody_context, desired_vdot,
                                            external_forces);
      }
    }
    stop = my_clock::now();
    duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(multibody_plant (with heap))" << std::to_string(num_reps)
              << "x inverse dynamics calculations took " << duration.count()
              << " milliseconds." << std::endl;
  }

#if 0
  start =  my_clock::now();
  multibody::MultibodyForces<AutoDiffXd> external_forces_autodiff(
      *multibody_plant_autodiff);

  for (int i = 0; i < num_autodiff_reps; i++) {
    x = Eigen::VectorXd::Constant(2*nq, i);
    desired_vdot = Eigen::VectorXd::Constant(nq, i);
    multibody_context_autodiff->get_mutable_continuous_state_vector().SetFromVector(math::initializeAutoDiff(x));
    multibody_plant_autodiff->CalcInverseDynamics(*multibody_context_autodiff, math::initializeAutoDiff(desired_vdot),
                                        external_forces_autodiff);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(multibody_plant)" << std::to_string(num_autodiff_reps) << "xautodiff inverse dynamics calculations took " <<
      duration.count() << " milliseconds." << std::endl;
#endif

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
