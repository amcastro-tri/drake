#include "drake/multibody/plant/multibody_plant.h"

#include <limits>
#include <memory>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/disable_malloc.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::Parser;
using systems::Context;
using std::unique_ptr;

namespace multibody {
namespace {

// Unit test fixture for a model of Kuka Iiwa arm parametrized on the periodic
// update period of the plant. This allows us to test some of the plant's
// functionality for both continuous and discrete models.
class KukaArmTest : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() override {
    const char kSdfPath[] =
        "drake/manipulation/models/iiwa_description/sdf/"
            "iiwa14_no_collision.sdf";
    plant_ = std::make_unique<MultibodyPlant<double>>(this->GetParam());
    Parser(plant_.get()).AddModelFromFile(FindResourceOrThrow(kSdfPath));
    const Joint<double>& weld =
        plant_->WeldFrames(plant_->world_frame(),
                           plant_->GetFrameByName("iiwa_link_0"));
    plant_->Finalize();

    EXPECT_EQ(plant_->num_positions(), 7);
    EXPECT_EQ(plant_->num_velocities(), 7);

    // We expect the last joint to be the one WeldJoint fixing the model to the
    // world, since we added it last above with the call to WeldFrames().
    // We verify this assumption.
    ASSERT_EQ(weld.index(), plant_->num_joints() - 1);

    context_ = plant_->CreateDefaultContext();
  }

  // Helper to set the multibody state x to x[i] = i for each i-th entry in the
  // state vector.
  // We use RevoluteJoint's methods to set the state in order to independently
  // unit test the proper workings of
  // MultibodyTree::get_multibody_state_vector() and its mutable counterpart.
  void SetState(const VectorX<double>& xc) {
    const int nq = plant_->num_positions();
    for (JointIndex joint_index(0);
         joint_index < plant_->num_joints() - 1 /* Skip "weld" joint. */;
         ++joint_index) {
      // We know all joints in our model, besides the first joint welding the
      // model to the world, are revolute joints.
      const auto& joint = plant_->GetJointByName<RevoluteJoint>(
          "iiwa_joint_" + std::to_string(joint_index + 1));

      // For this simple model we do know the order in which variables are
      // stored in the state vector.
      const double angle = xc[joint_index];
      const double angle_rate = xc[nq + joint_index];

      // We simply set each entry in the state with the value of its index.
      joint.set_angle(context_.get(), angle);
      joint.set_angular_rate(context_.get(), angle_rate);
    }
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;
};

// This test verifies we can easily access the multibody state vector x = [q, v]
// for either a discrete or continuous multibody model.
TEST_P(KukaArmTest, InverseDynamicsNoHeapAllocation) {
    VectorX<double> vdot = VectorX<double>::Ones(plant_->num_velocities());
    VectorX<double> x = VectorX<double>::Ones(plant_->num_multibody_states());
    plant_->SetPositionsAndVelocities(context_.get(), x);
    context_->EnableCaching();

    MultibodyForces<double> forces(*plant_);
    std::vector<SpatialAcceleration<double>> A_WB_array(plant_->num_bodies());
    std::vector<SpatialForce<double>> F_BMo_W_array(plant_->num_bodies());
    VectorX<double> tau_id(plant_->num_velocities());
    {
      test::DisableMalloc guard;      
      plant_->CalcInverseDynamics(*context_, vdot, forces, &A_WB_array,
                                  &F_BMo_W_array, &tau_id);
    }
}

TEST_P(KukaArmTest, InverseDynamicsWithHeapAllocation) {
    VectorX<double> vdot = VectorX<double>::Ones(plant_->num_velocities());
    VectorX<double> x = VectorX<double>::Ones(plant_->num_multibody_states());
    plant_->SetPositionsAndVelocities(context_.get(), x);
    MultibodyForces<double> forces(*plant_);

    ASSERT_DEATH(
        {
          test::DisableMalloc guard;
          plant_->CalcInverseDynamics(*context_, vdot, forces);
        },
        "abort due to malloc while DisableMalloc is in effect");
}

INSTANTIATE_TEST_CASE_P(
    Blank, KukaArmTest,
    testing::Values(0.0 /* continuous state */, 1e-3 /* discrete state */));

}  // namespace
}  // namespace multibody
}  // namespace drake
