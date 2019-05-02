#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace multibody_model {
namespace {

using benchmarks::kuka_iiwa_robot::MakeKukaIiwaModel;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using systems::Context;

const double kEpsilon = std::numeric_limits<double>::epsilon();

// Fixture to perform forward dynamics tests on the KUKA Iiwa model.
class KukaIiwaModelForwardDynamicsTests : public ::testing::Test {
 public:
  /// Creates MultibodyTree for a KUKA Iiwa robot arm.
  void SetUp() override {
    {
      auto tree =
          MakeKukaIiwaModel<double>(true /* Finalize model */, gravity_);

      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_1"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_2"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_3"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_4"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_5"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_6"));
      joints_.push_back(&tree->GetJointByName<RevoluteJoint>("iiwa_joint_7"));

      // We are done adding modeling elements.
      system_ = std::make_unique<MultibodyTreeSystem<double>>(std::move(tree));
    }
    context_ = system_->CreateDefaultContext();    
  }

  /// Sets the configuration of this KUKA Iiwa robot arm.
  ///
  /// @param[in] q robot's joint angles (generalized coordinates).
  /// @param[in] v robot's joint velocities (generalized velocities).
  void SetConfiguration(const Eigen::Ref<const VectorX<double>>& q,
                        const Eigen::Ref<const VectorX<double>>& v) {
    int angle_index = 0;
    for (const RevoluteJoint<double>* joint : joints_) {
      joint->set_angle(context_.get(), q[angle_index]);
      joint->set_angular_rate(context_.get(), v[angle_index]);
      angle_index++;
    }
  }

  const MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*system_);
  }

 protected:
  void VerifyMultiplyByMassMatrixInverse(
      const Eigen::Ref<const VectorX<double>>& q,
      const Eigen::Ref<const VectorX<double>>& v) {
    SetConfiguration(q, v);

    // Get model parameters.
    const int nv = tree().num_velocities();

    // Construct M, the mass matrix.
    MatrixX<double> M(nv, nv);
    tree().CalcMassMatrixViaInverseDynamics(*context_, &M);

    VectorX<double> x_expected(nv);
    x_expected << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;
    // Compute b for the known x
    const VectorX<double> b = M * x_expected; 

    VectorX<double> x(nv);
    tree().MultiplyByMassMatrixInverse(*context_, b, &x);
    const double kTolerance = 50 * kEpsilon;
    EXPECT_TRUE(CompareMatrices(
        x, x_expected, kTolerance, MatrixCompareType::relative));
  }

  // Acceleration of gravity:
  const double gravity_{9.81};
  // The model:
  std::unique_ptr<internal::MultibodyTreeSystem<double>> system_;
  // Workspace including context and derivatives vector:
  std::unique_ptr<Context<double>> context_;
  // Non-owning pointers to the joints:
  std::vector<const RevoluteJoint<double>*> joints_;
};

TEST_F(KukaIiwaModelForwardDynamicsTests, VerifyMultiplyByMassMatrixInverse) {
  // State variables and helper angles.
  Vector7d q, qdot;
  double q30 = M_PI / 6, q45 = M_PI / 4, q60 = M_PI / 3;

  // Test 1: Static configuration.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  qdot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  VerifyMultiplyByMassMatrixInverse(q, qdot);

  // Test 2: Another static configuration.
  q << q30, -q45, q60, -q30, q45, -q60, q30;
  qdot << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  VerifyMultiplyByMassMatrixInverse(q, qdot);

  // Test 3: Non-static configuration.
  q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  qdot << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  VerifyMultiplyByMassMatrixInverse(q, qdot);

  // Test 4: Another non-static configuration.
  q << -q45, q60, -q30, q45, -q60, q30, -q45;
  qdot << 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1;
  VerifyMultiplyByMassMatrixInverse(q, qdot);

  // Test 5: Another non-static configuration.
  q << q30, q45, q60, -q30, -q45, -q60, 0;
  qdot << 0.3, -0.1, 0.4, -0.1, 0.5, -0.9, 0.2;
  VerifyMultiplyByMassMatrixInverse(q, qdot);
}

}  // namespace
}  // namespace multibody_model
}  // namespace internal
}  // namespace multibody
}  // namespace drake
