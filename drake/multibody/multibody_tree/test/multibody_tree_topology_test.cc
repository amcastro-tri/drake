#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

int main() {
  MultibodyTree<double> tree;
  MassProperties<double> mass_properties(
      1.0, Vector3d::Zero(), RotationalInertia());

  auto link1 = Body<double>::CreateBody(&tree, mass_properties);

  auto link2 = Body<double>::CreateBody(&tree, mass_properties);

  auto joint1 = RevoluteJoint<double>::CreateJoint(&tree, )

}

}
}