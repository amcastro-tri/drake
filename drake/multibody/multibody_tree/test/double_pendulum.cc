#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/mass_properties.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/unit_inertia.h"

#include <iostream>

#include "drake/common/eigen_types.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace multibody {

using Eigen::Isometry3d;
using Eigen::Vector3d;

using std::cout;
using std::endl;
using std::make_unique;
using std::unique_ptr;

template <typename T>
std::ostream& operator<<(
    std::ostream& o, const Isometry3<T>& X) {
  return o
      << " Translation = " << X.translation().transpose() << std::endl
      << " Rotation = \n" << X.linear() << std::endl;
}

int DoMain() {
  const double length = 1.0;
  UnitInertia<double> G_Bc_B = UnitInertia<double>::SolidCube(length);
  (void) G_Bc_B;

  MultibodyTree<double> model;

  MassProperties<double> mass_properties(1.0, Vector3d::Zero(), G_Bc_B);
  Body<double>* pendulum =
      model.AddBody(make_unique<Body<double>>(mass_properties));
  Body<double> world_body = model.get_world_body();

  RigidBodyFrame<double>& Fframe =
      world_body.RigidlyAttachFrame(Isometry3d::Identity());

  RigidBodyFrame<double>& Mframe =
      pendulum->RigidlyAttachFrame(Isometry3d::Identity());

  RevoluteMobilizer<double>* pin_joint =
      model.AddMobilizer(make_unique<RevoluteMobilizer<double>>(
          Fframe, Mframe, Vector3d::UnitZ()));

  (void) Fframe;
  (void) Mframe;
  (void) pin_joint;

  PRINT_VAR(model.get_num_bodies());
  PRINT_VAR(model.get_num_frames());
  PRINT_VAR(model.get_num_mobilizers());

  model.Compile();
  model.PrintTopology();
  unique_ptr<MultibodyTreeContext<double>> context =
      model.CreateDefaultContext();
  context->get_mutable_positions().setZero();
  context->get_mutable_velocities().setZero();
  model.UpdatePositionKinematics(*context);
  context->Print();

#if 0
  MassProperties<double> mass_properties(1.0, Vector3d::Zero(), I_Bo_B);

  MultibodyTree<double> model;

  const Body<double>& world_body = model.get_world_body();

  // Method 1 of creation.
  // Disadvantage:
  //    it requires to expose Body::set_parent() publicly so that
  //    MultibodyTree::AddBody() can use it. which users might call and cause
  //    trouble.
  // Solution: Make MultibodyTree a friend of Body?
  //
  // Users can do this, and their link1 pointer will be invalid right after
  // AddBody, not-very-nice even when expected:
  //   auto link1 = make_unique<Body<double>>(mass_properties);
  //   model.AddBody(std::move(link1));
  //
  // So you are better off doing this:
  //Body<double>* link1 = model.AddBody(
  //    make_unique<Body<double>>(mass_properties));

  auto link1 = model.AddBody(make_unique<Body<double>>(mass_properties));

  // Another option is to imitate what DiagramBuilder::AddSystem() does so that
  // users do not have to type unique_ptr's, with the API:
  // auto link1 = model.AddBody<Body<double>>(mass_properties);

  // Method 2 of creation.
  // Disadvantage:
  //    It requires to expose MultibodyTree::AddBody() publicly so it can be
  //    called from within Body::CreateBody(). However not much of an issue if
  //    all Body constructors are private.
  // Solution:
  //    Can I make Body::CreateBody() a friend of MultibodyTree? should I?
  auto link2 = model.AddBody(make_unique<Body<double>>(mass_properties));

  PRINT_VAR(link1->get_default_mass_properties());
  PRINT_VAR(link2->get_default_mass_properties());
  PRINT_VAR(world_body.get_default_mass_properties());

  // Connect bodies with joints.
  Isometry3d X_PF, X_BM;
  Vector3d axis_F = Vector3d::UnitZ();
  PRINT_VAR(X_PF);
  X_PF = Isometry3d::Identity();
  X_BM = Isometry3d::Identity();
  X_BM.translation() = Vector3d(0.0, 0.5, 0.0);
  auto pin1 = model.AddJoint(
      make_unique<RevoluteJoint<double>>(
          world_body, *link1, X_PF, X_BM, axis_F));

  X_PF = Isometry3d::Identity();
  X_PF.translation() = Vector3d(0.0, -0.5, 0.0);
  X_BM = Isometry3d::Identity();
  X_BM.translation() = Vector3d(0.0, 0.5, 0.0);
  auto pin2 = model.AddJoint(
      make_unique<RevoluteJoint<double>>(
          *link1, *link2, X_PF, X_BM, axis_F));

  PRINT_VAR(model.get_num_bodies());
  PRINT_VAR(model.get_num_joints());

  model.Compile();
  model.PrintTopology();
  //unique_ptr<Context> context = CreateDefaultContext();
  //pin1->set_angle(context, M_PI / 6.0);
  //pin1->set_angular_velocity(context, M_PI / 6.0);
  //model.PrintTopology(std::cout);

  (void) pin1;
  (void) pin2;
#endif

#if 0
  MultibodyTree<double> tree;
  MassProperties<double> mass_properties(
      1.0, Vector3d::Zero(), RotationalInertia());

  auto link1 = Body<double>::CreateBody(&tree, mass_properties);

  auto link2 = Body<double>::CreateBody(&tree, mass_properties);

  auto joint1 = RevoluteJoint<double>::CreateJoint(&tree, )
#endif
  return 0;
}

}
}

int main() {
  return drake::multibody::DoMain();
}
