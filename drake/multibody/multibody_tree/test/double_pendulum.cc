#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/mass_properties.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/unit_inertia.h"
#include "drake/multibody/multibody_tree/weld_mobilizer.h"

#include <iostream>

#include "drake/common/eigen_types.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace multibody {

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Translation3d;
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
  const double length = 0.3;
  const double radius = 0.005;
  // Unit inertia in the "geometry" frame G computed about the geometric
  // center Gc.
  UnitInertia<double> G_Gc_G = UnitInertia<double>::SolidRod(radius, length);
  // Transformation from the geometry frame G to the body frame B.
  Isometry3d X_BG(AngleAxisd(M_PI_2, Vector3d::UnitX()));
  UnitInertia<double> G_Bc_B = G_Gc_G.ReExpress(X_BG.rotation());
  PRINT_VARn(G_Bc_B);

  auto owned_model = std::make_unique<MultibodyTree<double>>();
  MultibodyTree<double>* model = owned_model.get();

  MassProperties<double> mass_properties(1.0, Vector3d::Zero(), G_Bc_B);
  const RigidBody<double>& upper_body =
      RigidBody<double>::Create(model, mass_properties);
  //const RigidBody<double>& lower_body =
  //    RigidBody<double>::Create(model, mass_properties);
  const Body<double>& world_body = model->get_world_body();

  //(void) lower_body;
  (void) world_body;

  const auto& shoulder_inboard_frame = model->get_world_frame();

  // Pose of the mobilized frame M in the body frame B.
  Isometry3d X_BM(Translation3d(0.0, 0.5, 0.0));
  const auto& shoulder_outboard_frame =
      RigidBodyFrame<double>::Create(model, upper_body, X_BM);

  const RevoluteMobilizer<double>& shoulder_mobilizer =
      RevoluteMobilizer<double>::Create(
          model, shoulder_inboard_frame, shoulder_outboard_frame,
          Vector3d::UnitZ());

  model->Compile();
  model->PrintTopology();
  unique_ptr<MultibodyTreeContext<double>> context =
      model->CreateDefaultContext();

  //model.SetZeroConfiguration(context.get());
  shoulder_mobilizer.set_zero_configuration(context.get());
  shoulder_mobilizer.set_angular_velocity(context.get(), 0.0);
  model->UpdatePositionKinematicsCache(*context);
  context->Print();
  PRINT_VARn(upper_body.get_pose_in_world(*context).matrix());
  PRINT_VARn(upper_body.get_com_W(*context).transpose());
  PRINT_VARn(upper_body.get_M_Bo_W(*context));

  shoulder_mobilizer.set_angle(context.get(), M_PI / 6.0);
  model->UpdatePositionKinematicsCache(*context);
  model->UpdateCompositeBodyInertiasCache(*context);
  context->Print();
  const auto& X_WU = upper_body.get_pose_in_world(*context);
  PRINT_VARn(X_WU.matrix());
  PRINT_VARn(upper_body.get_com_W(*context).transpose());
  PRINT_VARn(upper_body.get_M_Bo_W(*context));

#if 0
  elbow_mobilizer->set_zero_configuration(context.get());
  elbow_mobilizer->set_angular_velocity(context.get(), 0.0);
  context->Print();

  elbow_mobilizer->set_angular_velocity(context.get(), M_PI / 6.0);
  model.UpdatePositionKinematicsCache(*context);

  cout << "Context after UpdatePositionKinematicsCache():" << endl;
  context->Print();
#endif

#if 0
  // Shoulder inboard frame IS the world frame.
  // TODO: Refactor ReferenceFrame to ReferenceFrame.
  const BodyFrame<double>& shoulder_inboard_frame = world_body.get_body_frame();

  // Pose of the shoulder outboard frame So in the frame U of the upper body.
  // TODO: see how to use Eigen::Translation.
  Isometry3d X_USo = Isometry3d::Identity();
  X_USo.translation() = Vector3d(0.0, 0.5, 0.0);
  // Rigidly attach the shoulder outboard frame to the upper body.
  RigidBodyFrame<double>& shoulder_outboard_frame =
      upper_body->RigidlyAttachFrame(X_USo);

  // Create the shoulder mobilizer between the inboard (world) and outboard (So)
  // frames.
  RevoluteMobilizer<double>* shoulder_mobilizer =
      model.AddMobilizer(make_unique<RevoluteMobilizer<double>>(
          shoulder_inboard_frame, shoulder_outboard_frame, Vector3d::UnitZ()));

  // Pose of the elbow inboard frame Ei in the frame U of the upper body.
  Isometry3d X_UEi = Isometry3d::Identity();
  X_UEi.translation() = Vector3d(0.0, -0.5, 0.0);
  // Frame rigidly attached to the end of the upper body. In general for this
  // example we will assume the upper body is rigid and therefore the use of a
  // rigidly attached frame.
  RigidBodyFrame<double>& elbow_inboard_frame =
      upper_body->RigidlyAttachFrame(X_UEi);

  // Pose of the elbow outboard frame Eo in the frame of the lower body L.
  Isometry3d X_LEo = Isometry3d::Identity();
  X_LEo.translation() = Vector3d(0.0, 0.5, 0.0);
  // End frame Le rigidly attaches to the lower body L.
  RigidBodyFrame<double>& elbow_outboard_frame =
      lower_body->RigidlyAttachFrame(X_LEo);

  // Now with all the necessary frames defined, create the elbow mobilizer
  // between the inboard (Ei) and outboard (Eo) elbow frames.
  RevoluteMobilizer<double>* elbow_mobilizer =
      model.AddMobilizer(make_unique<RevoluteMobilizer<double>>(
          elbow_inboard_frame, elbow_outboard_frame, Vector3d::UnitZ()));

  PRINT_VAR(model.get_num_bodies());
  PRINT_VAR(model.get_num_frames());
  PRINT_VAR(model.get_num_mobilizers());

  model.Compile();
  model.PrintTopology();
  unique_ptr<MultibodyTreeContext<double>> context =
      model.CreateDefaultContext();

  //model.SetZeroConfiguration(context.get());
  shoulder_mobilizer->set_zero_configuration(context.get());
  shoulder_mobilizer->set_angular_velocity(context.get(), 0.0);
  elbow_mobilizer->set_zero_configuration(context.get());
  elbow_mobilizer->set_angular_velocity(context.get(), 0.0);
  context->Print();

  elbow_mobilizer->set_angular_velocity(context.get(), M_PI / 6.0);
  model.UpdatePositionKinematicsCache(*context);

  cout << "Context after UpdatePositionKinematicsCache():" << endl;
  context->Print();
#endif

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
