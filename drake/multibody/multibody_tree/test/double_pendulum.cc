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

int DoMain() {
  const double Ix = 1.0;
  RotationalInertia<double> I_Bo_B(Ix, Ix, Ix);
  MassProperties<double> mass_properties(1.0, Vector3d::Zero(), I_Bo_B);

  MultibodyTree<double> model;

  const Body<double>& world_body = model.get_world_body();

  // Create bodies.
  Body<double>* link1 = Body<double>::CreateBody(&model, mass_properties);
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
