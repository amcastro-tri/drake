#include "drake/geometry/geometry_world.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/test/expect_error_message.h"

namespace drake {
namespace geometry {
namespace {

using drake::Isometry3;
using drake::systems::AbstractValues;
using std::make_unique;
using std::move;
using std::unique_ptr;

class GeometryWorldTest : public ::testing::Test {
 protected:
  void SetUp() {
    world_ = make_unique<GeometryWorld<double>>();
    context_ = make_unique<GeometryContext<double>>();
    auto abstract_values = world_->AllocateAbstractValues();
    context_->set_abstract_state(
        make_unique<AbstractValues>(move(abstract_values)));
    geometry_state_ =
        &context_->get_mutable_abstract_state<GeometryState<double>>(0);
  }

  // Members owned by the test class.
  unique_ptr<GeometryWorld<double>> world_;
  unique_ptr<GeometryContext<double>> context_;
  // The GeometryState instance is actually owned by the context; this is simply
  // a convenience handle.
  GeometryState<double>* geometry_state_;
};

// Confirm that the state is extracted from the context without any copying.
TEST_F(GeometryWorldTest, PullStateFromContext) {
  auto state_ptr = world_->get_mutable_state(context_.get());
  EXPECT_EQ(state_ptr, geometry_state_);
  auto& state = world_->get_state(*context_);
  EXPECT_EQ(&state, geometry_state_);
}

// Confirms the behavior of creating a FrameKinematicsSet from a bad source id.
// In release build, this is allowed (it'll throw an exception later when the
// frame kinematics set is used. In Debug build, an exception is thrown at the
// invocation. This tests confirms *both* behaviors.
TEST_F(GeometryWorldTest, CreateFrameKinematicsSetFromBadSource) {
  SourceId s_id = SourceId::get_new_id();
#ifdef DRAKE_ASSERT_IS_DISARMED
  // In release mode, this would be considered valid. No exception thrown.
  auto fks = world_->GetFrameKinematicsSet(s_id);
#else
  EXPECT_ERROR_MESSAGE(world_->GetFrameKinematicsSet(s_id),
                       std::logic_error,
                       "Invalid source id: \\d+.");
#endif
}

}  // namespace
}  // namespace geometry
}  // namespace drake
