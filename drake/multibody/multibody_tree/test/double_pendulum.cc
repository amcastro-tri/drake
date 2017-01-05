#include "drake/multibody/multibody_tree/multibody_tree.h"

#include <iostream>

#include "drake/common/eigen_types.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace multibody {

using Eigen::Vector3d;

using std::cout;
using std::endl;
using std::make_unique;

int DoMain() {
  const double Ix = 1.0;
  RotationalInertia<double> I_Bo_B(Ix, Ix, Ix);
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
  Body<double>* link1 = model.AddBody(
      make_unique<Body<double>>(mass_properties));

  // Method 2 of creation.
  // Disadvantage:
  //    It requires to expose MultibodyTree::AddBody() publicly so it can be
  //    called from within Body::CreateBody(). However not much of an issue if
  //    all Body constructors are private.
  // Solution:
  //    Can I make Body::CreateBody() a friend of MultibodyTree? should I?
  Body<double>* link2 = Body<double>::CreateBody(&model, mass_properties);

  // Question, how to add visuals? collision geometries?
  // Maybe:
  // link1->add_visual(X_BG, Sphere(some_radius), drake::visuals::Red);
  // link1->add_collision_(X_BC, Sphere(some_radius), drake::materials::Steel);
  // However: Probably even better leave MBT not related at all with visuals,
  // collisions. Place these in MultibodyWorldSystem instead? and figure out
  // connections with GeometrySystem?
  // Then MBT is purely math for dynamics.
  //
  // Where should contact forces computation go then? and contact Jacobians?

  // Connect bodies with joints.
  //RevoluteJoint<double> pin1 =
   //   RevoluteJoint<double>::CreateJoint(&model, )

  PRINT_VAR(model.get_num_bodies());

  PRINT_VAR(link1->get_default_mass_properties());
  PRINT_VAR(link2->get_default_mass_properties());
  PRINT_VAR(world_body.get_default_mass_properties());
  

  (void) link2;

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
