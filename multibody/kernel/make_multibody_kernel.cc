#include "drake/multibody/kernel/make_multibody_kernel.h"

#include <iostream>
#include <string>
#include <utility>

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/kernel/multibody_kernel.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/revolute_joint.h"

using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::multibody::internal::LinkJointGraph;
using drake::multibody::internal::SpanningForest;
using SpatialInertiad = drake::multibody::SpatialInertia<double>;
using drake::multibody::internal::ForestBuildingOptions;
using drake::multibody::internal::JointTraitsIndex;
using drake::multibody::internal::MobodIndex;

namespace drake {
namespace multibody {
namespace internal {

LinkJointGraph MakeLinkJointGraph(const MultibodyPlant<double>& plant) {
  LinkJointGraph graph;
  graph.RegisterJointType("revolute", 1, 1);

  // N.B. Skip the world body, already part of the graph.
  for (BodyIndex b(1); b < plant.num_bodies(); ++b) {
    const auto& body = plant.get_body(b);
    graph.AddLink(body.name(), body.model_instance());
  }

  for (JointIndex j(0); j < plant.num_joints(); ++j) {
    const auto& joint = plant.get_joint(j);
    graph.AddJoint(joint.name(), joint.model_instance(), joint.type_name(),
                   joint.parent_body().index(), joint.child_body().index());
  }

  // This flag today has no effect and composites and not yet combined.
  graph.SetGlobalForestBuildingOptions(
      ForestBuildingOptions::kCombineLinkComposites);

  if (!graph.BuildForest()) {
    throw std::logic_error(
        "The resulting spanning forest is not valid for dynamics.");
  }

  return graph;
}

// Computes X_PF for the mobilizer associted with the given joint.
RigidTransformd CalcInboardJointFramePoseInParentBodyFrame(
    const MultibodyPlant<double>& plant, JointIndex joint_index) {
  const Joint<double>& joint = plant.get_joint(joint_index);
  const Frame<double>& frame_F = joint.frame_on_parent();
  RigidTransformd X_PF = frame_F.GetFixedPoseInBodyFrame();
  return X_PF;
}

// Returns the pose X_MB of body B in its mobilized frame M.
RigidTransformd CalcBodyPoseInMobilizedFrame(
    const MultibodyPlant<double>& plant, JointIndex joint_index) {
  const Joint<double>& joint = plant.get_joint(joint_index);
  const Frame<double>& frame_M = joint.frame_on_child();
  const RigidTransformd X_BM = frame_M.GetFixedPoseInBodyFrame();
  return X_BM.inverse();
}

// Computes the spatial inertia M_BMo_M of body B for the given mobod, about the
// mobilized frame's origin Mo, expressed in M.
SpatialInertiad CalcBodySpatialInertia(const MultibodyPlant<double>& plant,
                                       const SpanningForest::Mobod& mobod) {
  const BodyIndex body_index = mobod.link();

  const auto& body =
      dynamic_cast<const RigidBody<double>&>(plant.get_body(body_index));

  // N.B. This body frame B is as defined by the plant, with a generic,
  // non-identity, pose X_MB and X_PF.
  // For the kernel, we got to compute the inertia in the mobilized frame M,
  // that is, M_BMo_M, the inertia of B, about Mo, expressed in M.
  const SpatialInertiad& M_Bo_B = body.default_spatial_inertia();

  const JointIndex joint_index = mobod.joint();
  const Joint<double>& joint = plant.get_joint(joint_index);
  const Frame<double>& frame_M = joint.frame_on_child();
  const RigidTransformd X_BM = frame_M.GetFixedPoseInBodyFrame();

  // Shift to Mo.
  const Vector3d p_BoMo_B = X_BM.translation();
  const SpatialInertiad M_BMo_B = M_Bo_B.Shift(p_BoMo_B);

  // Re-express in M.
  const RotationMatrixd R_MB = X_BM.rotation().transpose();
  const SpatialInertiad M_BMo_M = M_BMo_B.ReExpress(R_MB);

  return M_BMo_M;
}

// For normal basis vectors eᵢ, returns the index of the single i-th component
// that equals to one, or otherwise throws an exception.
int GetAxisOrThrow(const Vector3d& axis) {
  int i = 0;
  while (i < 2) {
    if (axis(i) == 1.0) break;
    ++i;
  }
  if (i == 3) {
    throw std::logic_error("Axis not aligned with either x, y, or z.");
  }
  return i;
}

void AddRevoluteMobod(const MultibodyPlant<double>& plant,
                      const SpanningForest::Mobod& mobod,
                      MobodParameters<double> mobod_parameters,
                      MultibodyKernel<double>* kernel,
                      MultibodyKernelParameters<double>* kernel_parameters) {
  const auto& joint = dynamic_cast<const RevoluteJoint<double>&>(
      plant.get_joint(mobod.joint()));
  const Vector3d& axis_M = joint.revolute_axis();
  const int i = GetAxisOrThrow(axis_M);

  MobodInfo info{mobod.index(), mobod.inboard(), mobod.outboards(),
                 mobod.q_start(), mobod.v_start()};

  switch (i) {
    case 0: {
      kernel->AddMobod<RevoluteMobod<double, 0>>(
          std::move(info), std::move(mobod_parameters), kernel_parameters);
      break;
    }
    case 1: {
      kernel->AddMobod<RevoluteMobod<double, 1>>(
          std::move(info), std::move(mobod_parameters), kernel_parameters);
      break;
    }
    case 2: {
      kernel->AddMobod<RevoluteMobod<double, 2>>(
          std::move(info), std::move(mobod_parameters), kernel_parameters);
      break;
    }
  }
}

void AddWeldMobod(const MultibodyPlant<double>&,
                  const SpanningForest::Mobod& mobod,
                  MobodParameters<double> mobod_parameters,
                  MultibodyKernel<double>* kernel,
                  MultibodyKernelParameters<double>* kernel_parameters) {
  MobodInfo info{mobod.index(), mobod.inboard(), mobod.outboards(),
                 mobod.q_start(), mobod.v_start()};
  kernel->AddMobod<WeldMobod<double>>(
      std::move(info), std::move(mobod_parameters), kernel_parameters);
}

void AddQuaternionFloatingMobod(
    const MultibodyPlant<double>&, const SpanningForest::Mobod& mobod,
    MobodParameters<double> mobod_parameters, MultibodyKernel<double>* kernel,
    MultibodyKernelParameters<double>* kernel_parameters) {
  MobodInfo info{mobod.index(), mobod.inboard(), mobod.outboards(),
                 mobod.q_start(), mobod.v_start()};
  kernel->AddMobod<QuaternionFloatingMobod<double>>(
      std::move(info), std::move(mobod_parameters), kernel_parameters);
}

std::pair<MultibodyKernel<double>, MultibodyKernelParameters<double>>
MakeMultibodyKernel(const MultibodyPlant<double>& plant,
                    const LinkJointGraph& graph) {
  const SpanningForest& forest = graph.forest();
  MultibodyKernel<double> kernel;
  MultibodyKernelParameters<double> kernel_parameters(forest.mobods().size());

  for (const SpanningForest::Mobod& mobod : forest.mobods()) {
    if (mobod.is_world()) continue;
    const JointIndex joint_index = mobod.joint();
    const JointTraitsIndex joint_traits_index =
        graph.joints(joint_index).traits_index();
    // const JointTraits& joint_traits = graph.joint_traits(joint_traits_index);
    // const Joint<double>& joint = plant.get_joint(joint_index);
    // const std::string& type_name = joint.type_name();

    // N.B. DEMANDs below are listed in the order MbP registers joint types at
    // construction, with graph default types (weld, quaternion_floating,
    // rpy_floating) listed first.
    const LinkJointGraph::JointTraits& joint_traits =
        graph.joint_traits(joint_traits_index);
    const std::string& type_name = joint_traits.name;

    // TODO: DEMAND your assumption that parent joint frame is inboard and joint
    // child frame is outboard.

    // TODO: Collect generic data an parameters:
    //  1. inboard mobod index
    //  2. X_PF
    //  3. M_BMo_M
    const MobodIndex inboard_mobod = mobod.inboard();
    const RigidTransformd X_PF =
        CalcInboardJointFramePoseInParentBodyFrame(plant, joint_index);
    const RigidTransformd X_MB =
        CalcBodyPoseInMobilizedFrame(plant, joint_index);
    const SpatialInertiad M_BMo_M = CalcBodySpatialInertia(plant, mobod);

    const auto body_index = mobod.link();
    const auto& body = plant.get_body(body_index);

    (void)body;
    // std::cout << fmt::format("Mobod: {}, Body: {}, Joint: {}, type: {}\n",
    //                          mobod.index(), body.name(), joint_index,
    //                          type_name);

    MobodParameters<double> mobod_parameters{std::move(X_PF), std::move(X_MB),
                                             std::move(M_BMo_M)};

    (void)inboard_mobod;
    (void)X_PF;
    (void)M_BMo_M;

    switch (joint_traits_index) {
      case 0: {
        DRAKE_DEMAND(type_name == "weld");
        AddWeldMobod(plant, mobod, std::move(mobod_parameters), &kernel,
                     &kernel_parameters);
        break;
      }
      case 1: {
        DRAKE_DEMAND(type_name == "quaternion_floating");
        AddQuaternionFloatingMobod(plant, mobod, std::move(mobod_parameters),
                                   &kernel, &kernel_parameters);
        break;
      }
      case 2: {
        DRAKE_DEMAND(type_name == "rpy_floating");
        throw std::logic_error("rpy_floating not supported.");
        break;
      }
      case 3: {
        DRAKE_DEMAND(type_name == "revolute");
        AddRevoluteMobod(plant, mobod, std::move(mobod_parameters), &kernel,
                         &kernel_parameters);
        break;
      }
      default: {
        throw std::logic_error("Joint type not considered: '" + type_name +
                               "'.");
      }
    }
  }

  return std::make_pair(std::move(kernel), std::move(kernel_parameters));
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
