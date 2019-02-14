#include "drake/common/find_resource.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"
#include "drake/multibody/plant/volumetric_contact/load_objs/load_objs.h"
#include "drake/multibody/plant/volumetric_contact/tools/wildmagic_tools.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::multibody::LoadConvexPolyhedronFromObj;
using drake::multibody::ReExpressConvexPolyhedron;

namespace drake {  

int do_main() {
  std::string model_file, obj_model;  

  model_file = "drake/examples/wildmagic/IntersectConvexPolyhedra/cube.obj";
  obj_model = drake::FindResourceOrThrow(model_file);

  std::unique_ptr<Wm5::ConvexPolyhedron<double>> poly0_P0 =
      LoadConvexPolyhedronFromObj(obj_model, Vector3<double>(0.5, 0.5, 0.5));

  // The first polyhedron is an ellipsoid.
  //Wm5::ConvexPolyhedrond::CreateEggShape(Wm5::Vector3d::ZERO, 1.0, 1.0, 2.0, 2.0,
  //                                  4.0, 4.0, 3, poly0_P0);
  // The second polyhedron is egg shaped.
  //Wm5::ConvexPolyhedrond::CreateEggShape(Wm5::Vector3d::ZERO,
  //                                       1.0, 1.0, // in x
   //                                      0.5, 0.5, // in y
  //                                       0.8, 0.8, // in z
  //                                       4, poly1_P1);

  model_file =
      "drake/examples/wildmagic/IntersectConvexPolyhedra/ground_box.obj";
  obj_model = drake::FindResourceOrThrow(model_file);
  std::unique_ptr<Wm5::ConvexPolyhedron<double>> poly1_P1 =
      LoadConvexPolyhedronFromObj(obj_model, Vector3<double>(1.0, 1.0, 1.0));  

  PRINT_VAR(poly0_P0->GetNumVertices());
  PRINT_VAR(poly0_P0->GetNumTriangles());

  PRINT_VAR(poly1_P1->GetNumVertices());
  PRINT_VAR(poly1_P1->GetNumTriangles());

  // Change the pose of poly 0.
  const RollPitchYaw<double> rpy(M_PI/4, 0, 0);
  const RigidTransform<double> X_WP0(rpy, Vector3<double>(0, 0.0, 0.5));
  Wm5::ConvexPolyhedrond poly0_W;
  ReExpressConvexPolyhedron(*poly0_P0, X_WP0.GetAsIsometry3(),
                            &poly0_W);

  // Change the pose of poly 1.
  const RollPitchYaw<double> rpy1(0, 0, 0);
  const RigidTransform<double> X_WP1(rpy1, Vector3<double>(0, 0.0, 0.0));
  Wm5::ConvexPolyhedrond poly1_W;
  ReExpressConvexPolyhedron(*poly1_P1, X_WP1.GetAsIsometry3(),
                            &poly1_W);                            

  // Compute the intersection (if any) in world space.
  Wm5::ConvexPolyhedrond intersection_W;
  bool hasIntersection = poly0_W.FindIntersection(
      poly1_W, intersection_W);

  PRINT_VAR(hasIntersection);
  PRINT_VAR(intersection_W.GetNumVertices());
  PRINT_VAR(intersection_W.GetNumTriangles());

  poly0_W.PrintObj("poly0_W.obj");
  poly1_W.PrintObj("poly1_W.obj");
  if (hasIntersection) intersection_W.PrintObj("intersection_W.obj");

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
