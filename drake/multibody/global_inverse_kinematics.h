#pragma once
#include <vector>

#include "drake/multibody/constraint/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
class GlobalInverseKinematics : public solvers::MathematicalProgram {
 public:
  GlobalInverseKinematics(const RigidBodyTreed& robot, int num_binary_vars_per_half_axis = 2);

  const std::vector<solvers::MatrixDecisionVariable<3, 3>>& body_rotmat() const { return body_rotmat_;}

  const std::vector<solvers::VectorDecisionVariable<3>>& body_pos() const { return body_pos_;}

  /**
   * Adds the constraint that position of a point `body_pt` on a body
   * (whose index is `body_idx`), is within a box in a specified frame.
   * where the inequality is elementwise.
   * @param body_idx The index of the body on which the position of a point is constrained.
   * @param body_pt The position of the point measured in the body `body_idx`.
   * @param box_lb The lower bound of the box.
   * @param box_ub The upper bound of the box.
   * @param measured_frame. The frame in which the box is specified. Namely if
   * the position of `body_pt` in the world frame is x, then the constraint
   * is box_lb <= measured_transform.linear().transpose() * (x - measured_transform.translation()) <= box_ub
   */
  void AddWorldPositionConstraint(int body_idx,
                             const Eigen::Vector3d& body_pt,
                             const Eigen::Vector3d& box_lb,
                             const Eigen::Vector3d& box_ub,
                             const Eigen::Isometry3d& measured_frame = Eigen::Isometry3d::Identity());

  void AddWorldOrientationConstraint(int body_idx,
                                     const Eigen::Quaterniond& desired_orientation,
                                     double angle_tol);

 private:
  const RigidBodyTree<double> *robot_;

  // body_rotmat_[i] is the rotation matrix for body i.
  std::vector<solvers::MatrixDecisionVariable<3, 3>> body_rotmat_;

  // body_pos_[i] is the position for body i.
  std::vector<solvers::VectorDecisionVariable<3>> body_pos_;
};
}  // namespace multibody
}  // namespace drake
