#include "drake/common/find_resource.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"
#include "drake/multibody/plant/volumetric_contact/load_objs/load_objs.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

using drake::multibody::LoadConvexPolyhedronFromObj;

namespace drake {

int do_main() {
  Wm5::ConvexPolyhedrond mWorldPoly1, mIntersection;

  const std::string model_file =
      "drake/examples/wildmagic/IntersectConvexPolyhedra/cube.obj";
  const std::string obj_model = drake::FindResourceOrThrow(model_file);

  std::unique_ptr<Wm5::ConvexPolyhedron<double>> mWorldPoly0 =
      LoadConvexPolyhedronFromObj(obj_model, Vector3<double>(1.0, 1.0, 1.0));

  // The first polyhedron is an ellipsoid.
  //Wm5::ConvexPolyhedrond::CreateEggShape(Wm5::Vector3d::ZERO, 1.0, 1.0, 2.0, 2.0,
  //                                  4.0, 4.0, 3, mWorldPoly0);

  // The second polyhedron is egg shaped.
  Wm5::ConvexPolyhedrond::CreateEggShape(Wm5::Vector3d::ZERO, 2.0, 2.0, 4.0, 4.0,
                                    5.0, 3.0, 4, mWorldPoly1);


  PRINT_VAR(mWorldPoly0->GetNumVertices());
  PRINT_VAR(mWorldPoly0->GetNumTriangles());

  PRINT_VAR(mWorldPoly1.GetNumVertices());
  PRINT_VAR(mWorldPoly1.GetNumTriangles());

#if 0
  // Transform the model-space vertices to world space.
  VertexBufferAccessor vba(mMeshPoly0);
  int i;
  for (i = 0; i < vba.GetNumVertices(); ++i)
  {
    APoint modPos = vba.Position<Float3>(i);
    APoint locPos = mMeshPoly0->LocalTransform*modPos;
    mWorldPoly0.Point(i) = Vector3f(locPos[0], locPos[1], locPos[2]);
  }
  mWorldPoly0.UpdatePlanes();

  vba.ApplyTo(mMeshPoly1);
  for (i = 0; i < vba.GetNumVertices(); ++i)
  {
    APoint modPos = vba.Position<Float3>(i);
    APoint locPos = mMeshPoly1->LocalTransform*modPos;
    mWorldPoly1.Point(i) = Vector3f(locPos[0], locPos[1], locPos[2]);
  }
  mWorldPoly1.UpdatePlanes();
#endif

  // Compute the intersection (if any) in world space.
  bool hasIntersection = mWorldPoly0->FindIntersection(
      mWorldPoly1, mIntersection);

  PRINT_VAR(hasIntersection);
  PRINT_VAR(mIntersection.GetNumVertices());
  PRINT_VAR(mIntersection.GetNumTriangles());

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
