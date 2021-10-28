#include "drake/examples/multibody/mass_matrix_benchmarks/test/featherstone_ltdl.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace {

class AllegroHandModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
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

    // Use default state to avoid problems with all-zero quaternions.
    x_ = context_->get_continuous_state_vector().CopyToVector();
  }

  // Makes expanded array lambda(i) described in [Featherstone, 2014].
  // For node i in the tree, lambda(i) corresponds to the parent node of node i.
  std::vector<int> MakeParentArray() const {
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
        lambda.push_back(parent_node_last_dof);
      }
    }

    return lambda;
  }

  std::unique_ptr<MultibodyPlant<double>> plant_{};
  std::unique_ptr<Context<double>> context_;
  int nq_{};
  int nv_{};
  int nu_{};
  VectorXd x_{};
};

TEST_F(AllegroHandModelTest, Ltdl) {
  PRINT_VAR(nq_);
  PRINT_VAR(nv_);
  PRINT_VAR(nu_);
  MatrixXd M(nv_, nv_);
  plant_->CalcMassMatrix(*context_, &M);
  PRINT_VARn(M);

  const auto lambda = MakeParentArray();
  for (size_t i = 0; i < lambda.size(); ++i) {
    std::cout << fmt::format("lambda({}) = {}\n", i, lambda[i]);
  }

  MatrixXd LTDL = M;
  test::CalcLtdlInPlace(lambda, &LTDL);
  PRINT_VARn(LTDL);

  // Extract lower triangular matrix L, with ones in the diagonal.
  const MatrixXd L = LTDL.triangularView<Eigen::UnitLower>();

  // Extract diagonal matrix D.
  const VectorXd D = LTDL.diagonal();

  // Reconstruct matrix M.
  const MatrixXd M_reconstructed = L.transpose() * D.asDiagonal() * L;

  // Verify result.
  EXPECT_TRUE(CompareMatrices(M, M_reconstructed, 1.0e-14,
                              MatrixCompareType::relative));
  PRINT_VAR((M - M_reconstructed).norm());
}

}  // namespace
}  // namespace examples
}  // namespace drake
