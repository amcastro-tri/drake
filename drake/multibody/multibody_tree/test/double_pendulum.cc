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

  cout << mass_properties << endl;

  MultibodyTree<double> world;

  Body<double>* link1 = Body<double>::CreateBody(&world, mass_properties);

  PRINT_VAR(world.get_num_bodies());

  PRINT_VAR(link1->get_default_mass_properties());

  (void) link1;

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
