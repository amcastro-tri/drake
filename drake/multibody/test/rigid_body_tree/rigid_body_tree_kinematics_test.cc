#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::Vector3d;

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace multibody {

class AcrobotTests : public ::testing::Test {
 protected:
  void SetUp() {
    std::string file_name =
        GetDrakePath() + "/multibody/test/rigid_body_tree/double_pendulum.urdf";
    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        file_name, multibody::joints::kFixed, robot_.get());
    cache_ = std::make_unique<KinematicsCache<double>>(
        robot_->CreateKinematicsCache());

    link1_ = robot_->FindBody("link1");
    link2_ = robot_->FindBody("link2");
    frame1_ = robot_->findFrame("frame_on_link1").get();
    link1_com_ = robot_->findFrame("link1_com").get();
    link2_com_ = robot_->findFrame("link2_com").get();
    X_L1F1_ = frame1_->get_transform_to_body();

    world_id_ = robot_->FindBody("world")->get_body_index();
    base_id_ = robot_->FindBody("base")->get_body_index();
    link1_id_ = link1_->get_body_index();
    link2_id_ = link2_->get_body_index();

    axis1_index_ =
        link1_->get_position_start_index();
    axis2_index_ =
        link2_->get_position_start_index();

    q_.resize(robot_->get_num_positions());
    v_.resize(robot_->get_num_velocities());
  }

  void SetState(double theta1_rad, double theta2_rad) {
    q_(axis1_index_) = theta1_rad;
    q_(axis2_index_) = theta2_rad;
    cache_->initialize(q_);
    robot_->doKinematics(*cache_);
  }

  // Tests CalcPoseInWorld(body, offset) == CalcPoseInWorld(frame),
  // assuming the underlying function relativeTransform is correct.
  void RunPoseTest(double theta1_deg, double theta2_deg) {
    const double deg_to_rad = M_PI / 180.0;
    const double theta1_rad = theta1_deg * deg_to_rad;
    const double theta2_rad = theta2_deg * deg_to_rad;
    SetState(theta1_rad, theta2_rad);

    Isometry3<double> X_WL1cm =
        robot_->CalcPoseInWorld(*cache_, *link1_com_);
    Isometry3<double> X_WL1cm_exact =
        acrobot_benchmark_.CalcLink1PoseInWorldFrame(theta1_rad, theta2_rad);
    EXPECT_TRUE(X_WL1cm.isApprox(X_WL1cm_exact,
                                 Eigen::NumTraits<double>::epsilon()));
    Isometry3<double> X_WL2cm =
        robot_->CalcPoseInWorld(*cache_, *link2_com_);
    Isometry3<double> X_WL2cm_exact =
        acrobot_benchmark_.CalcLink2PoseInWorldFrame(theta1_rad, theta2_rad);
    EXPECT_TRUE(X_WL2cm.isApprox(X_WL2cm_exact,
                                 Eigen::NumTraits<double>::epsilon()));
  }

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<KinematicsCache<double>> cache_;
  const RigidBody<double> *link1_, *link2_;
  const RigidBodyFrame<double> *frame1_, *link1_com_, *link2_com_;
  int world_id_, base_id_, link1_id_, link2_id_;
  int axis1_index_, axis2_index_;
  int frame_id_;
  VectorX<double> q_;
  VectorX<double> v_;
  Isometry3<double> X_L1F1_;  // Transform from frame F1 to body frame of link1.
  benchmarks::Acrobot<double> acrobot_benchmark_{Vector3d::UnitZ(),
                                                 Vector3d::UnitY()};
};

TEST_F(AcrobotTests, PoseTests) {
  const double theta_max = 45.0;
  const int ntheta = 3;
  const double dtheta = 2 * theta_max / ntheta;
  for (double theta1 = -theta_max; theta1 <= theta_max; theta1 += dtheta) {
    for (double theta2 = -theta_max; theta2 <= theta_max; theta2 += dtheta) {
      RunPoseTest(theta1, theta2);
    }
  }
}

// Test the following utility functions assuming their underlying functions
// are correct:
// CalcPoseInWorld,
// CalcTwistInWorld,
// CalcTwistInWorldAlignedBody,
// CalcTwistInWorldAlignedBody,
// CalcJacobianDotTimesVForWorldAlignedBody.
//
// A fixed frame named "test_frame" is added to
// "multibody/test/rigid_body_tree/two_dof_robot.urdf".
// All the following tests are essentially checking the equality between:
// robot.CalcPoseInWorld(cache, body, offset) and
// robot.CalcPoseInWorld(cache, frame), where "test_frame" is attached to
// "body" with an "offset".
class RBTDifferentialKinematicsHelperTest : public ::testing::Test {
 protected:
  void SetUpWithType(drake::multibody::joints::FloatingBaseType type) {
    std::string file_name =
        GetDrakePath() + "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(file_name, type,
                                                       robot_.get());
    cache_ = std::make_unique<KinematicsCache<double>>(
        robot_->CreateKinematicsCache());

    frame_ptr_ = robot_->findFrame("test_frame").get();
    body_ptr_ = &(frame_ptr_->get_rigid_body());
    offset_ = frame_ptr_->get_transform_to_body();
    frame_id_ = frame_ptr_->get_frame_index();

    q_ = VectorX<double>(robot_->get_num_positions());
    v_ = VectorX<double>(robot_->get_num_velocities());

    for (int i = 0; i < q_.size(); ++i) q_[i] = 0.2 * i;
    for (int i = 0; i < v_.size(); ++i) v_[i] = -0.2 * i;

    cache_->initialize(q_, v_);
    robot_->doKinematics(*cache_, true);
  }

  // Tests CalcPoseInWorld(body, offset) == CalcPoseInWorld(frame),
  // assuming the underlying function relativeTransform is correct.
  void TestPose() {
    Isometry3<double> pose =
        robot_->CalcPoseInWorld(*cache_, *body_ptr_, offset_);
    Isometry3<double> pose_as_frame =
        robot_->CalcPoseInWorld(*cache_, *frame_ptr_);

    EXPECT_TRUE(drake::CompareMatrices(pose.linear(), pose_as_frame.linear(),
                                       1e-14,
                                       drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(pose.translation(),
                                       pose_as_frame.translation(), 1e-14,
                                       drake::MatrixCompareType::absolute));
  }

  // Tests CalcTwistInWorld(body, offset) == CalcTwistInWorld(frame),
  // assuming the underlying function relativeTwist is correct.
  void TestTwistInWorld() {
    TwistVector<double> twist =
        robot_->CalcTwistInWorld(*cache_, *body_ptr_, offset_);
    TwistVector<double> twist_as_frame =
        robot_->CalcTwistInWorld(*cache_, *frame_ptr_);

    EXPECT_TRUE(drake::CompareMatrices(twist, twist_as_frame, 1e-14,
                                       drake::MatrixCompareType::absolute));
  }

  // Tests
  // CalcTwistInWorldAlignedBody(body, offset) ==
  // CalcTwistInWorldAlignedBody(frame),
  // and
  // CalcTwistInWorldAlignedBody(frame) ==
  // [R, 0; 0, R] * relativeTwist(world, frame, frame), where R is the rotation
  // from frame to world.
  // This assumes relativeTwist and CalcPoseInWorld are correct.
  void TestTwistInWorldAlignedBody() {
    TwistVector<double> xdot =
        robot_->CalcTwistInWorldAlignedBody(*cache_, *body_ptr_, offset_);
    TwistVector<double> xdot_as_frame =
        robot_->CalcTwistInWorldAlignedBody(*cache_, *frame_ptr_);

    EXPECT_TRUE(drake::CompareMatrices(xdot, xdot_as_frame, 1e-14,
                                       drake::MatrixCompareType::absolute));

    TwistVector<double> xdot_b = robot_->relativeTwist(
        *cache_, robot_->world().get_body_index(), frame_id_, frame_id_);
    Isometry3<double> pose = robot_->CalcPoseInWorld(*cache_, *frame_ptr_);
    TwistVector<double> xdot_w;
    xdot_w.head<3>() = pose.linear() * xdot_b.head<3>();
    xdot_w.tail<3>() = pose.linear() * xdot_b.tail<3>();

    EXPECT_TRUE(drake::CompareMatrices(xdot, xdot_w, 1e-14,
                                       drake::MatrixCompareType::absolute));
  }

  // Tests
  // CalcTwistInWorldAlignedBody(body, offset) ==
  // CalcTwistInWorldAlignedBody(frame),
  // and
  // CalcTwistInWorldAlignedBody(frame) * v ==
  // CalcTwistInWorldAlignedBody(frame),
  // [R, 0; 0, R] * geometricJacobian(world, frame, frame) ==
  // CalcTwistInWorldAlignedBody(frame)
  // assuming geometricJacobian and CalcTwistInWorldAlignedBody are correct.
  void TestWorldAlignedJacobian(bool use_qdot) {
    TwistVector<double> xdot =
        robot_->CalcTwistInWorldAlignedBody(*cache_, *body_ptr_, offset_);
    MatrixX<double> J = robot_->CalcJacobianForWorldAlignedBody(
        *cache_, *body_ptr_, offset_, use_qdot);
    MatrixX<double> J_as_frame = robot_->CalcJacobianForWorldAlignedBody(
        *cache_, *frame_ptr_, Isometry3<double>::Identity(), use_qdot);

    EXPECT_TRUE(drake::CompareMatrices(J, J_as_frame, 1e-14,
                                       drake::MatrixCompareType::absolute));

    if (!use_qdot) {
      EXPECT_TRUE(drake::CompareMatrices(xdot, J * v_, 1e-14,
                                         drake::MatrixCompareType::absolute));
    } else {
      VectorX<double> qd = robot_->GetVelocityToQDotMapping(*cache_) * v_;
      EXPECT_TRUE(drake::CompareMatrices(xdot, J * qd, 1e-14,
                                         drake::MatrixCompareType::absolute));
    }

    Isometry3<double> pose = robot_->CalcPoseInWorld(*cache_, *frame_ptr_);
    KinematicPath kinematic_path = robot_->findKinematicPath(
        robot_->world().get_body_index(), frame_ptr_->get_frame_index());
    MatrixX<double> Jg = robot_->geometricJacobian(
        *cache_, robot_->world().get_body_index(),
        frame_ptr_->get_frame_index(), frame_ptr_->get_frame_index(), use_qdot);
    Jg = robot_->compactToFull(Jg, kinematic_path.joint_path, use_qdot);
    Jg.topRows<3>() = pose.linear() * Jg.topRows<3>();
    Jg.bottomRows<3>() = pose.linear() * Jg.bottomRows<3>();

    EXPECT_TRUE(drake::CompareMatrices(Jg, J, 1e-14,
                                       drake::MatrixCompareType::absolute));
  }

  // Tests
  // CalcJacobianDotTimesVForWorldAlignedBody(body, offset) ==
  // CalcJacobianDotTimesVForWorldAlignedBody(frame),
  // and
  // CalcJacobianDotTimesVForWorldAlignedBody(frame) ==
  // transformPointsJacobianDotTimesV(Vec3::Zero, frame, world),
  // assuming geometricJacobianDotTimesV and transformPointsJacobianDotTimesV
  // are correct.
  void TestWorldAlignedJacobianDotTimesV() {
    TwistVector<double> Jdv = robot_->CalcJacobianDotTimesVForWorldAlignedBody(
        *cache_, *body_ptr_, offset_);
    TwistVector<double> Jdv_as_frame =
        robot_->CalcJacobianDotTimesVForWorldAlignedBody(*cache_, *frame_ptr_);
    EXPECT_TRUE(drake::CompareMatrices(Jdv, Jdv_as_frame, 1e-14,
                                       drake::MatrixCompareType::absolute));

    Vector3<double> off = offset_.translation();
    VectorX<double> Jdv1 = robot_->transformPointsJacobianDotTimesV(
        *cache_, off, body_ptr_->get_body_index(),
        robot_->world().get_body_index());
    VectorX<double> Jdv2 = robot_->transformPointsJacobianDotTimesV(
        *cache_, Vector3<double>::Zero(), frame_ptr_->get_frame_index(),
        robot_->world().get_body_index());

    EXPECT_TRUE(drake::CompareMatrices(Jdv2, Jdv1, 1e-14,
                                       drake::MatrixCompareType::absolute));

    EXPECT_TRUE(drake::CompareMatrices(Jdv.tail<3>(), Jdv1, 1e-14,
                                       drake::MatrixCompareType::absolute));
  }

  std::unique_ptr<RigidBodyTree<double>> robot_;
  std::unique_ptr<KinematicsCache<double>> cache_;
  const RigidBodyFrame<double>* frame_ptr_;
  const RigidBody<double>* body_ptr_;
  int frame_id_;

  VectorX<double> q_;
  VectorX<double> v_;
  Isometry3<double> offset_;
};

TEST_F(RBTDifferentialKinematicsHelperTest, RPYPoseTest) {
  SetUpWithType(multibody::joints::kRollPitchYaw);
  TestPose();
}

TEST_F(RBTDifferentialKinematicsHelperTest, RPYTwistInWorldTest) {
  SetUpWithType(multibody::joints::kRollPitchYaw);
  TestTwistInWorld();
}

TEST_F(RBTDifferentialKinematicsHelperTest, RPYTwistInWorldAlignedBodyTest) {
  SetUpWithType(multibody::joints::kRollPitchYaw);
  TestTwistInWorldAlignedBody();
}

TEST_F(RBTDifferentialKinematicsHelperTest, RPYJacobianTest) {
  SetUpWithType(multibody::joints::kRollPitchYaw);
  TestWorldAlignedJacobian(true);
  TestWorldAlignedJacobian(false);
}

TEST_F(RBTDifferentialKinematicsHelperTest, RPYJacobianDotTimeVTest) {
  SetUpWithType(multibody::joints::kRollPitchYaw);
  TestWorldAlignedJacobianDotTimesV();
}

TEST_F(RBTDifferentialKinematicsHelperTest, QuatPoseTest) {
  SetUpWithType(multibody::joints::kQuaternion);
  TestPose();
}

TEST_F(RBTDifferentialKinematicsHelperTest, QuatTwistInWorldTest) {
  SetUpWithType(multibody::joints::kQuaternion);
  TestTwistInWorld();
}

TEST_F(RBTDifferentialKinematicsHelperTest, QuatTwistInWorldAlignedBodyTest) {
  SetUpWithType(multibody::joints::kQuaternion);
  TestTwistInWorldAlignedBody();
}

TEST_F(RBTDifferentialKinematicsHelperTest, QuatJacobianTest) {
  SetUpWithType(multibody::joints::kQuaternion);
  TestWorldAlignedJacobian(true);
  TestWorldAlignedJacobian(false);
}

TEST_F(RBTDifferentialKinematicsHelperTest, QuatJacobianDotTimeVTest) {
  SetUpWithType(multibody::joints::kQuaternion);
  TestWorldAlignedJacobianDotTimesV();
}
}  // namespace multibody
}  // namespace drake
