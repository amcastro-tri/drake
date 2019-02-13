#include "drake/common/find_resource.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"
#include "drake/multibody/plant/volumetric_contact/load_objs/load_objs.h"
#include "drake/multibody/plant/volumetric_contact/tools/wildmagic_tools.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

using drake::math::RigidTransform;
using drake::multibody::LoadConvexPolyhedronFromObj;
using drake::multibody::ReExpressConvexPolyhedron;

namespace drake {  

int do_main() {
  Wm5::ConvexPolyhedrond mWorldPoly1, mIntersection;

  const std::string model_file =
      "drake/examples/wildmagic/IntersectConvexPolyhedra/cube.obj";
  const std::string obj_model = drake::FindResourceOrThrow(model_file);

  std::unique_ptr<Wm5::ConvexPolyhedron<double>> mWorldPoly0 =
      LoadConvexPolyhedronFromObj(obj_model, Vector3<double>(0.5, 0.5, 0.5));

  // The first polyhedron is an ellipsoid.
  //Wm5::ConvexPolyhedrond::CreateEggShape(Wm5::Vector3d::ZERO, 1.0, 1.0, 2.0, 2.0,
  //                                  4.0, 4.0, 3, mWorldPoly0);

  // The second polyhedron is egg shaped.
  Wm5::ConvexPolyhedrond::CreateEggShape(Wm5::Vector3d::ZERO,
                                         1.0, 1.0, // in x
                                         0.5, 0.5, // in y
                                         0.8, 0.8, // in z
                                         4, mWorldPoly1);

  PRINT_VAR(mWorldPoly0->GetNumVertices());
  PRINT_VAR(mWorldPoly0->GetNumTriangles());

  PRINT_VAR(mWorldPoly1.GetNumVertices());
  PRINT_VAR(mWorldPoly1.GetNumTriangles());

  // Change the pose of poly 0.
  const RigidTransform<double> X_WP0(Vector3<double>(2.5, 0.0, 0.0));

  ReExpressConvexPolyhedron(*mWorldPoly0, X_WP0.GetAsIsometry3(),
                            mWorldPoly0.get());

  // Compute the intersection (if any) in world space.
  bool hasIntersection = mWorldPoly0->FindIntersection(
      mWorldPoly1, mIntersection);

  PRINT_VAR(hasIntersection);
  PRINT_VAR(mIntersection.GetNumVertices());
  PRINT_VAR(mIntersection.GetNumTriangles());

  mWorldPoly0->PrintObj("poly0.obj");
  mWorldPoly1.PrintObj("poly1.obj");
  mIntersection.PrintObj("intersection.obj");

  return 0;
}

}  // namespace drake.

int main(int argc, char* argv[]) {
  #if 0
  gflags::SetUsageMessage(
      "A demo for a cylinder falling towards the ground using Drake's"
      "MultibodyPlant, with SceneGraph contact handling and visualization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  #endif
  return drake::do_main();
}
