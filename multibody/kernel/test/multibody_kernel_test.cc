#include "drake/multibody/kernel/multibody_kernel.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/kernel/make_multibody_kernel.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/revolute_joint.h"

constexpr double kEps = std::numeric_limits<double>::epsilon();

using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::multibody::internal::ForestBuildingOptions;
using drake::multibody::internal::JointTraitsIndex;
using drake::multibody::internal::LinkJointGraph;
using drake::multibody::internal::SpanningForest;

namespace drake {
namespace multibody {
namespace internal {
namespace {

std::unique_ptr<MultibodyPlant<double>> MakePlant() {
  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  Parser parser(plant.get());
  const auto& model = "drake/multibody/kernel/test/cassie_v2.urdf";
  parser.AddModels(FindResourceOrThrow(model));
  // plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("pelvis"));
  plant->Finalize();
  return plant;
}

GTEST_TEST(MultibodyKernel, Construction) {
  auto plant = MakePlant();
  const LinkJointGraph graph = MakeLinkJointGraph(*plant);
  std::cout << fmt::format("nq = {}, nv = {}, nb = {}, nj = {}.\n",
                           plant->num_positions(), plant->num_velocities(),
                           plant->num_bodies(), plant->num_joints());

  std::cout << "Registered joint types:\n";
  for (const auto& trait : graph.joint_traits()) {
    std::cout << fmt::format("{}\n", trait.name);
  }
  std::cout << fmt::format("Graph with nl = {}, nj = {}.\n",
                           graph.num_user_links(), graph.num_user_joints());

  const SpanningForest& forest = graph.forest();
  EXPECT_TRUE(forest.is_valid());

  std::cout << "Forest info:\n";
  std::cout << fmt::format("nq = {}, nv = {}, nmb = {}\n",
                           forest.num_positions(), forest.num_velocities(),
                           forest.mobods().size());
  std::cout << fmt::format("nt = {}, h = {}, nwmb = {}\n",
                           forest.trees().size(), forest.height(),
                           forest.welded_mobods().size());

  auto [kernel, parameters] = MakeMultibodyKernel(*plant, graph);
  EXPECT_EQ(kernel.num_mobods(), forest.mobods().size());
  EXPECT_EQ(kernel.num_velocities(), forest.num_velocities());
  EXPECT_EQ(kernel.num_positions(), forest.num_positions());
}

GTEST_TEST(MultibodyKernel, PositionKinematics) {
  auto plant = MakePlant();
  std::cout << fmt::format("nq = {}, nv = {}, nb = {}, nj = {}.\n",
                           plant->num_positions(), plant->num_velocities(),
                           plant->num_bodies(), plant->num_joints());

  LinkJointGraph graph = MakeLinkJointGraph(*plant);
  std::cout << fmt::format("Graph with nl = {}, nj = {}.\n",
                           graph.num_user_links(), graph.num_user_joints());

  const SpanningForest& forest = graph.forest();
  std::cout << "Forest info:\n";
  std::cout << fmt::format("nq = {}, nv = {}, nmb = {}\n",
                           forest.num_positions(), forest.num_velocities(),
                           forest.mobods().size());

  auto [kernel, parameters] = MakeMultibodyKernel(*plant, graph);

  // Arbitrary configuration vector.
  VectorXd q = VectorXd::LinSpaced(kernel.num_positions(), -5.0, 12.0);
  q.head<4>().normalize();
  PositionKinematicsData<double> pkd = kernel.MakePositionKinematicsData();
  kernel.CalcPositionKinematicsData(parameters, q, &pkd);

  std::vector<drake::math::RigidTransformd> X_WB(kernel.num_mobods());
  kernel.CalcBodyPosesInWorld(parameters, pkd, &X_WB);

  auto context = plant->CreateDefaultContext();
  plant->SetPositions(context.get(), q);

  for (const SpanningForest::Mobod& mobod : forest.mobods()) {
    const auto body_index = mobod.link();
    const auto& body = plant->get_body(body_index);
    const auto& X_WB_plant = plant->EvalBodyPoseInWorld(*context, body);
    const auto& X_WB_kernel = X_WB[mobod.index()];

    EXPECT_TRUE(CompareMatrices(X_WB_kernel.GetAsMatrix34(),
                                X_WB_plant.GetAsMatrix34(), kEps,
                                MatrixCompareType::relative));
  }
}

GTEST_TEST(MultibodyKernel, VelocityKinematics) {
  auto plant = MakePlant();
  LinkJointGraph graph = MakeLinkJointGraph(*plant);
  auto [kernel, parameters] = MakeMultibodyKernel(*plant, graph);

  // Arbitrary configuration and velocity vectors vector.
  VectorXd q = VectorXd::LinSpaced(kernel.num_positions(), -5.0, 12.0);
  q.head<4>().normalize();
  VectorXd v = VectorXd::LinSpaced(kernel.num_velocities(), -12.0, 5.0);

  // Position kinematics.
  PositionKinematicsData<double> pk = kernel.MakePositionKinematicsData();
  kernel.CalcPositionKinematicsData(parameters, q, &pk);
  std::vector<drake::math::RigidTransformd> X_WB(kernel.num_mobods());
  kernel.CalcBodyPosesInWorld(parameters, pk, &X_WB);

  // Velocity kinematics.
  VelocityKinematicsData<double> vk = kernel.MakeVelocityKinematicsData();
  kernel.CalcVelocityKinematicsData(parameters, q, v, &pk, &vk);
  std::vector<SpatialVelocity<double>> V_WB(kernel.num_mobods());
  kernel.CalcBodySpatialVelocitiesInWorld(parameters, X_WB, vk, &V_WB);

  // MbP's quaternion mobilizer is different from the kernel. In MbP v = V_WB
  // and in the kernel v = V_WM_M. We create the right v for MbP from the
  // computed V_WB with the kernel.
  v.head<6>() = V_WB[1].get_coeffs();

  // Set the plant's context.
  auto context = plant->CreateDefaultContext();
  plant->SetPositions(context.get(), q);
  plant->SetVelocities(context.get(), v);

  const SpanningForest& forest = graph.forest();
  for (const SpanningForest::Mobod& mobod : forest.mobods()) {
    const auto body_index = mobod.link();
    const auto& body = plant->get_body(body_index);
    const auto& V_WB_plant =
        plant->EvalBodySpatialVelocityInWorld(*context, body);
    const auto& V_WB_kernel = V_WB[mobod.index()];

    // std::cout << fmt::format("i: {}, name: {}, V: {}.\n", mobod.index(),
    //                          body.name(), V_WB_kernel);

    EXPECT_TRUE(CompareMatrices(V_WB_plant.get_coeffs(),
                                V_WB_kernel.get_coeffs(), 32 * kEps,
                                MatrixCompareType::relative));
  }
}

GTEST_TEST(MultibodyKernel, AccelerationKinematics) {
  auto plant = MakePlant();
  LinkJointGraph graph = MakeLinkJointGraph(*plant);
  auto [kernel, parameters] = MakeMultibodyKernel(*plant, graph);

  // Arbitrary configuration and velocity vectors vector.
  VectorXd q = VectorXd::LinSpaced(kernel.num_positions(), -5.0, 12.0);
  q.head<4>().normalize();
  VectorXd v = VectorXd::LinSpaced(kernel.num_velocities(), -12.0, 5.0);
  VectorXd vdot = VectorXd::LinSpaced(kernel.num_velocities(), -1.0, 2.0);

  // Acceleration kinematics.
  PositionKinematicsData<double> pk = kernel.MakePositionKinematicsData();
  VelocityKinematicsData<double> vk = kernel.MakeVelocityKinematicsData();
  AccelerationKinematicsData<double> ak =
      kernel.MakeAccelerationKinematicsData();
  kernel.CalcAccelerationKinematicsData(parameters, q, v, vdot, &pk, &vk, &ak);

  // Calc body kinematics in the world frame X_WB.
  std::vector<drake::math::RigidTransformd> X_WB(kernel.num_mobods());
  kernel.CalcBodyPosesInWorld(parameters, pk, &X_WB);
  std::vector<SpatialVelocity<double>> V_WB(kernel.num_mobods());
  kernel.CalcBodySpatialVelocitiesInWorld(parameters, X_WB, vk, &V_WB);
  std::vector<SpatialAcceleration<double>> A_WB_kernel(kernel.num_mobods());
  kernel.CalcBodySpatialAccelerationsInWorld(parameters, X_WB, vk, ak,
                                             &A_WB_kernel);

  // MbP's quaternion mobilizer is different from the kernel. In MbP v = V_WB
  // and in the kernel v = V_WM_M. We create the right v for MbP from the
  // computed V_WB with the kernel.
  v.head<6>() = V_WB[1].get_coeffs();
  vdot.head<6>() = A_WB_kernel[1].get_coeffs();

  // Set the plant's context.
  auto context = plant->CreateDefaultContext();
  plant->SetPositions(context.get(), q);
  plant->SetVelocities(context.get(), v);
  std::vector<SpatialAcceleration<double>> A_WB_plant(plant->num_bodies());
  plant->CalcSpatialAccelerationsFromVdot(*context, vdot, &A_WB_plant);

  const SpanningForest& forest = graph.forest();
  for (const SpanningForest::Mobod& mobod : forest.mobods()) {
    const auto body_index = mobod.link();
    const auto& A_WBi_plant = A_WB_plant[body_index];
    const auto& A_WBi_kernel = A_WB_kernel[mobod.index()];

    // const auto& body = plant->get_body(body_index);
    // std::cout << fmt::format("i: {}, name: {}, A: {}\n", mobod.index(),
    //                          body.name(), A_WBi_kernel);

    EXPECT_TRUE(CompareMatrices(A_WBi_kernel.get_coeffs(),
                                A_WBi_plant.get_coeffs(), 64 * kEps,
                                MatrixCompareType::relative));
  }
}

GTEST_TEST(MultibodyKernel, InverseDynamics) {
  auto plant = MakePlant();
  const LinkJointGraph graph = MakeLinkJointGraph(*plant);
  auto [kernel, parameters] = MakeMultibodyKernel(*plant, graph);

  // Arbitrary configuration and velocity vectors vector.
  VectorXd q = VectorXd::LinSpaced(kernel.num_positions(), -5.0, 12.0);
  q.head<4>().normalize();
  VectorXd v = VectorXd::LinSpaced(kernel.num_velocities(), -12.0, 5.0);
  v.setZero();
  VectorXd vdot = VectorXd::LinSpaced(kernel.num_velocities(), -1.0, 2.0);

  // Acceleration kinematics.
  PositionKinematicsData<double> pk = kernel.MakePositionKinematicsData();
  VelocityKinematicsData<double> vk = kernel.MakeVelocityKinematicsData();
  AccelerationKinematicsData<double> ak =
      kernel.MakeAccelerationKinematicsData();
  kernel.CalcAccelerationKinematicsData(parameters, q, v, vdot, &pk, &vk, &ak);

  // Arbitrary applied forces, Fapp_BMo_M.
  const VectorXd applied_generalized_forces =
      VectorXd::LinSpaced(kernel.num_velocities(), -3.0, 1.2);
  std::vector<SpatialForce<double>> applied_spatial_forces(kernel.num_mobods());
  int i = 0;
  for (auto& F : applied_spatial_forces) {
    F = SpatialForce<double>((i + 0.3) * Vector6d::LinSpaced(1.0, 6.0));
    ++i;
  }

  // Inverse dynamics.
  std::vector<SpatialForce<double>> mobod_spatial_forces(kernel.num_mobods());
  VectorXd mobod_generalized_forces(kernel.num_velocities());
  kernel.CalcInverseDynamics(parameters, pk, vk, ak, applied_spatial_forces,
                             applied_generalized_forces, &mobod_spatial_forces,
                             &mobod_generalized_forces);

  // Calc body kinematics so that we can go from F_BMo_M to F_BBo_W.
  std::vector<drake::math::RigidTransformd> X_WB(kernel.num_mobods());
  kernel.CalcBodyPosesInWorld(parameters, pk, &X_WB);
  std::vector<SpatialVelocity<double>> V_WB(kernel.num_mobods());
  kernel.CalcBodySpatialVelocitiesInWorld(parameters, X_WB, vk, &V_WB);
  std::vector<SpatialAcceleration<double>> A_WB(kernel.num_mobods());
  kernel.CalcBodySpatialAccelerationsInWorld(parameters, X_WB, vk, ak, &A_WB);

  // Transforms spatial force F_BMo_M to F_BBo_W.
  const RigidTransformd& X_MB = parameters.X_MB(1);
  const RotationMatrixd R_BM = X_MB.rotation().transpose();
  const Vector3d& p_MB_M = X_MB.translation();
  const RotationMatrixd R_WM = X_WB[1].rotation() * R_BM;
  auto TransformFromMtoB = [&](const SpatialForce<double>& F_BMo_M) {
    const SpatialForce<double> F_BBo_M = F_BMo_M.Shift(p_MB_M);
    const SpatialForce<double> F_BBo_W = R_WM * F_BBo_M;
    return F_BBo_W;
  };

  // MbP's quaternion mobilizer is different from the kernel. In MbP v = V_WB
  // and in the kernel v = V_WM_M. We create the right v for MbP from the
  // computed V_WB with the kernel.
  v.head<6>() = V_WB[1].get_coeffs();
  vdot.head<6>() = A_WB[1].get_coeffs();
  // Similarly with the generalized forces on the free floating body.
  const SpatialForce<double> Ftau_BMo_M(applied_generalized_forces.head<6>());
  const SpatialForce<double> Ftau_BBo_W = TransformFromMtoB(Ftau_BMo_M);

  // Set the plant's context.
  auto context = plant->CreateDefaultContext();
  plant->SetPositions(context.get(), q);
  plant->SetVelocities(context.get(), v);
  MultibodyForces<double> forces(*plant);
  forces.mutable_generalized_forces() = applied_generalized_forces;
  forces.mutable_generalized_forces().head<6>() = Ftau_BBo_W.get_coeffs();
  kernel.CalcBodySpatialForcesInWorld(parameters, X_WB, applied_spatial_forces,
                                      &forces.mutable_body_forces());
  const VectorXd tau_plant = plant->CalcInverseDynamics(*context, vdot, forces);

  // For the free floating body, mobod_generalized_forces stores F_BMo_M, while
  // MbP stores F_BBo_W.
  const SpatialForce<double> F_BMo_M(mobod_generalized_forces.head<6>());
  const SpatialForce<double> Fkernel_BBo_W = TransformFromMtoB(F_BMo_M);
  const SpatialForce<double> Fplant_BBo_W(tau_plant.head<6>());
  EXPECT_TRUE(CompareMatrices(Fkernel_BBo_W.get_coeffs(),
                              Fplant_BBo_W.get_coeffs(), 16 * kEps,
                              MatrixCompareType::relative));

  // Compares all generalized forces but the ones from the floating mobod.
  ASSERT_EQ(kernel.num_velocities(), 22);
  EXPECT_TRUE(CompareMatrices(mobod_generalized_forces.tail<16>(),
                              tau_plant.tail<16>(), 16 * kEps,
                              MatrixCompareType::relative));
}

GTEST_TEST(MultibodyKernel, MassMatrix) {
  auto plant = MakePlant();
  const LinkJointGraph graph = MakeLinkJointGraph(*plant);
  auto [kernel, parameters] = MakeMultibodyKernel(*plant, graph);

  // Arbitrary configuration.
  VectorXd q = VectorXd::LinSpaced(kernel.num_positions(), -5.0, 12.0);
  q.head<4>().normalize();

  PositionKinematicsData<double> pk = kernel.MakePositionKinematicsData();
  kernel.CalcPositionKinematicsData(parameters, q, &pk);

  // Calc mass matrix with the kernel.
  const int nv = kernel.num_velocities();
  MatrixXd Mkernel(nv, nv);
  kernel.CalcMassMatrixViaInverseDynamics(parameters, q, pk, &Mkernel);

  // Use MbP to get a reference solution.
  auto context = plant->CreateDefaultContext();
  plant->SetPositions(context.get(), q);
  MatrixXd Mplant(nv, nv);
  plant->CalcMassMatrix(*context, &Mplant);

  // The generalized velocities for the floating base are different between the
  // kernel and the plant. Therefore we can only compare the remaining entries
  // of the mass matrix.
  EXPECT_TRUE(CompareMatrices(Mkernel.block<16, 16>(6, 6),
                              Mplant.block<16, 16>(6, 6), 2.0 * kEps,
                              MatrixCompareType::relative));

  // We'll use an arbitrary set of velocities to compute kinetic energy.
  const VectorXd v = VectorXd::LinSpaced(kernel.num_velocities(), -12.0, 5.0);
  const double ke_kernel = v.transpose() * Mkernel * v;

  // For the plant v, we need to transform the velocities of the floating base
  // from V_WM_M to V_WB_W.
  std::vector<drake::math::RigidTransformd> X_WB(kernel.num_mobods());
  kernel.CalcBodyPosesInWorld(parameters, pk, &X_WB);
  VelocityKinematicsData<double> vk = kernel.MakeVelocityKinematicsData();
  kernel.CalcVelocityKinematicsData(parameters, q, v, &pk, &vk);
  std::vector<SpatialVelocity<double>> V_WB(kernel.num_mobods());
  kernel.CalcBodySpatialVelocitiesInWorld(parameters, X_WB, vk, &V_WB);
  VectorXd v_plant = v;
  v_plant.head<6>() = V_WB[1].get_coeffs();
  const double ke_plant = v_plant.transpose() * Mplant * v_plant;

  const double absolute_tolerance = 2.0 * kEps * ke_kernel;
  EXPECT_NEAR(ke_kernel, ke_plant, absolute_tolerance);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
