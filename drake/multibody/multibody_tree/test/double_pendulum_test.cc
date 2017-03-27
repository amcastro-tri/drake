#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {
namespace {

using Eigen::Isometry3d;
using Eigen::Matrix4d;
using Eigen::Translation3d;
using std::make_unique;
using std::unique_ptr;
using systems::Context;

// Tests the logic to create a multibody tree model for a double pendulum.
// This double pendulum is similar to the acrobot model described in Section 3.1
// of the Underactuated Robotics notes available online at
// http://underactuated.csail.mit.edu/underactuated.html?chapter=3.
// The only difference is that this model has no actuation.
// This double pendulum is defined in the x-y plane with gravity acting in the
// negative y-axis direction.
// In this model the two links of the pendulum have the same length and their
// body frames are located at the links' centroids.
class PendulumTests : public ::testing::Test {
 public:
  void SetUp() override {
    MultibodyTree<double>* model = &model_;

    // Retrieves the world body.
    world_body_ = &model->get_world_body();

    // Adds the upper and lower links of the pendulum.
    upper_link_ = &RigidBody<double>::Create(model);
    lower_link_ = &RigidBody<double>::Create(model);
  }
 protected:
  MultibodyTree<double> model_;
  const Body<double>* world_body_;
  const RigidBody<double>* upper_link_;
  const RigidBody<double>* lower_link_;
  const double link_length = 1.0;
  const double half_link_length = link_length / 2;
  Isometry3d X_WLu_{Translation3d(0.0, -half_link_length, 0.0)};

  // For testing whether we can retrieve/set cache entries, this method
  // initializes the poses of each link in their corresponding cache entries.
  void SetPendulumPoses(Context<double>* context) {
    DRAKE_DEMAND(context != nullptr);
    auto mbt_context = dynamic_cast<MultibodyTreeContext<double>*>(context);
    DRAKE_DEMAND(mbt_context != nullptr);
    PositionKinematicsCache<double>* pc =
        mbt_context->get_mutable_position_kinematics();
    pc->get_mutable_X_WB(BodyNodeIndex(1)) = X_WLu_;
    // MultibodyTree methods re-computing the PositionKinematicsCache will
    // validate as so:
    mbt_context->validate_position_kinematics_cache();
  }
};

TEST_F(PendulumTests, CreateContext) {
  MultibodyTree<double>* model = &model_;

  // Verifies the number of multibody elements is correct.
  EXPECT_EQ(model->get_num_bodies(), 3);

  // Verify we cannot create a Context until we have a valid topology.
  EXPECT_FALSE(model->topology_is_valid());  // Not valid before Compile().
  EXPECT_ANY_THROW(model->CreateDefaultContext());

  // Compile() stage.
  EXPECT_NO_THROW(model->Compile());
  EXPECT_TRUE(model->topology_is_valid());  // Valid after Compile().

  // Create Context.
  std::unique_ptr<Context<double>> context;
  EXPECT_NO_THROW(context = model->CreateDefaultContext());

  SetPendulumPoses(context.get());

  const Isometry3d& X_WW = world_body_->get_pose_in_world(*context);
  const Isometry3d& X_WLu = upper_link_->get_pose_in_world(*context);

  EXPECT_TRUE(X_WW.matrix().isApprox(Matrix4d::Identity()));
  EXPECT_TRUE(X_WLu.matrix().isApprox(X_WLu_.matrix()));
}

#ifndef NDEBUG
TEST_F(PendulumTests, AssertEigenDynamicMemoryAllocation) {
  // Compile() stage.
  EXPECT_NO_THROW(model_.Compile());
  EXPECT_TRUE(model_.topology_is_valid());  // Valid after Compile().

  // Create Context.
  std::unique_ptr<Context<double>> context;
  EXPECT_NO_THROW(context = model_.CreateDefaultContext());

  // After this point MultibodyTree queries should not allocate memory.
  Eigen::internal::set_is_malloc_allowed(false);
  ASSERT_DEATH({ auto context2 = model_.CreateDefaultContext(); },
               R"(double_pendulum_test: .*Memory.h:...: )"
               R"(void Eigen::internal::check_that_malloc_is_allowed\(\):)");
}
#endif

}  // namespace
}  // namespace multibody
}  // namespace drake
