#include "drake/multibody/plant/volumetric_contact/volumetric_contact_model.h"

#include <utility>
#include <spruce.hh>
#include <tiny_obj_loader.h>

//#include "MTMesh.h"
//#include "Wm5Plane3.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"

//#include <iostream>
//#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

using drake::geometry::GeometryId;
// using Wm5::ConvexPolyhedra;

namespace drake {

namespace multibody {

template <typename T>
Vector3<T> ToEigenVector3(const Wm5::Vector3<T>& p) {
  return Vector3<T>(p.X(), p.Y(), p.Z());
}

template <typename T>
Wm5::Vector3<T> ToWm5Vector3(const Vector3<T>& p) {
  return Wm5::Vector3<T>(p.x(), p.y(), p.z());
}

//
// Convert faces from tinyobj to FCL.
//
//
// A tinyobj mesh has an integer array storing the number of vertices of
// each polygonal face.
//     mesh.num_face_vertices = {n0,n1,n2,...}
//         face0 has n0 vertices.
//         face1 has n1 vertices.
//         face2 has n2 vertices.
//         ...
// A tinyobj mesh has a vector of vertices that belong to the faces.
//     mesh.indices = {v0_0, v0_1,..., v0_n0-1,
//                     v1_0, v1_1,..., v1_n1-1,
//                     v2_0, v2_1,..., v2_n2-1,
//                     ...}
//         face0 has vertices v0_0, v0_1,...,v0_n0-1.
//         face1 has vertices v1_0, v1_1,...,v1_n1-1.
//         face2 has vertices v2_0, v2_1,...,v2_n2-1.
//         ...
// For fcl::Convex, the `faces` is as an array of integers in this format.
//     faces = { n0, v0_0,v0_1,...,v0_n0-1,
//               n1, v1_0,v1_1,...,v1_n1-1,
//               n2, v2_0,v2_1,...,v2_n2-1,
//               ...}
// where n_i is the number of vertices of face_i.
//
int TinyObjToFaces(const tinyobj::mesh_t& mesh, std::vector<int>* faces) {
  auto iter = mesh.indices.begin();
  for (const int& num : mesh.num_face_vertices) {
    std::for_each(iter, iter + num, [faces](const tinyobj::index_t& index) {
      faces->push_back(index.vertex_index);
    });
    iter += num;
  }

  return mesh.num_face_vertices.size();
}

//
// Convert vertices from tinyobj format to FCL format.
//
// Vertices from tinyobj are in a vector of floating-points like this:
//     attrib.vertices = {c0,c1,c2, c3,c4,c5, c6,c7,c8,...}
//                     = {x, y, z,  x, y, z,  x, y, z,...}
// We will convert to a vector of Vector3d for FCL like this:
//     vertices = {{c0,c1,c2}, {c3,c4,c5}, {c6,c7,c8},...}
//              = {    v0,         v1,         v2,    ...}
//
// The size of `attrib.vertices` is three times the number of vertices.
//
std::vector<Wm5::Vector3d> TinyObjToVertices(const tinyobj::attrib_t& attrib,
                                             const Vector3<double>& scales) {
  int num_coords = attrib.vertices.size();
  DRAKE_DEMAND(num_coords % 3 == 0);
  std::vector<Wm5::Vector3d> vertices;
  vertices.reserve(num_coords / 3);

  auto iter = attrib.vertices.begin();
  while (iter != attrib.vertices.end()) {
    // We increment `iter` three times for x, y, and z coordinates.
    double x = *(iter++) * scales[0];
    double y = *(iter++) * scales[1];
    double z = *(iter++) * scales[2];
    vertices.emplace_back(x, y, z);
  }

  return vertices;
}

template <typename T>
void VolumetricContactModel<T>::LoadObj(
    const std::string& file_name, const Vector3<double>& scales,
    std::vector<Wm5::Vector3<double>>* points, std::vector<int>* indexes) {
  PRINT_VAR(file_name);

  // We use tiny_obj_loader to read the .obj file of the convex shape.
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string err;
  // We keep polygonal faces without triangulating them. Some algorithms for
  // convex geometry perform better with fewer faces.
  bool do_tinyobj_triangulation = false;
  // We use default value (NULL) for the base directory of .mtl file (material
  // description), so it will be searched from the working directory.
  const char* mtl_basedir = nullptr;
  bool ret =
      tinyobj::LoadObj(&attrib, &shapes, &materials, &err, file_name.c_str(),
                       mtl_basedir, do_tinyobj_triangulation);
  if (!ret || !err.empty()) {
    throw std::runtime_error("Error parsing file '" + file_name + "' : " + err);
  }

  // TODO(DamrongGuoy) Check that the input is a valid convex polyhedron.
  // 1. Each face is a planar polygon.
  // 2. Each face is a convex polygon.
  // 3. The polyhedron is convex.

  //
  // Now we convert tinyobj data for fcl::Convex.
  //

  if (shapes.size() != 1) {
    throw std::runtime_error(
        "For Convex geometry, the .obj file must have "
        "one and only one object defined in it.");
  }

  // Convert to format needed in ConvexPolyhedron
  *points = TinyObjToVertices(attrib, scales);

  const tinyobj::mesh_t& mesh = shapes[0].mesh;

  // Convert to the format needed by ConvexPolyhedron
  TinyObjToFaces(mesh, indexes);
}

template <typename T>
void VolumetricContactModel<T>::ReExpressMesh(
    const Wm5::ConvexPolyhedron<T>& mesh_B, const Isometry3<T>& X_AB,
    Wm5::ConvexPolyhedron<T>* mesh_A_ptr) {  
  Wm5::ConvexPolyhedron<T>& mesh_A = *mesh_A_ptr;
  mesh_A = mesh_B;  // "allocate" sime size.
  for (int i = 0; i < mesh_A.GetNumVertices(); ++i) {
    const Wm5::Vector3<double> p_AP =
        ToWm5Vector3(X_AB * ToEigenVector3(mesh_B.Point(i)));
    mesh_A.Point(i) = p_AP;
  }
  mesh_A.UpdatePlanes();
}

template <typename T>
bool VolumetricContactModel<T>::CalcIntersection(
    int index_A, const Isometry3<double>& X_WA,
    int index_B, const Isometry3<double>& X_WB,
    VolumetricContactPair<T>* pair) const {
  // "Allocate" by simply copying.
  meshA_W_ = *owned_meshes_[index_A];
  meshB_W_ = *owned_meshes_[index_B];

  // Transform to a common frame, the world.
  ReExpressMesh(meshA_W_, X_WA, &meshA_W_);
  ReExpressMesh(meshB_W_, X_WB, &meshB_W_);

  const bool has_intersection =
      meshA_W_.FindIntersection(meshB_W_, intersection_W_);

  if (has_intersection) {
    pair->index_A = index_A;
    pair->index_B = index_B;
    pair->volume = intersection_W_.GetVolume();

    // For now use this approximation.
    // TODO(amcastro): change to computation in terms of normal integral.
    pair->area = intersection_W_.GetSurfaceArea() / 2.0;

    // Only for contact vs plane approximation.
    // TODO(amcastro): change to computation in terms of normal integral.
    pair->nhat_BA_W = Vector3<double>::UnitZ();

    intersection_W_.ComputeCentroid();
    pair->p_WCo = ToEigenVector3(intersection_W_.GetCentroid());
  }

  return has_intersection;
}

template <typename T>
std::vector<VolumetricContactPair<T>>
VolumetricContactModel<T>::CalcAllIntersectionPairs(
    const std::vector<Isometry3<T>>& X_WM_list) const {
  DRAKE_DEMAND(X_WM_list.size() == owned_meshes_.size());
  std::vector<VolumetricContactPair<T>> pairs;
  VolumetricContactPair<T> a_pair;

  // O(n^2) prototype.
  for (int imesh = 0; imesh < num_meshes(); ++imesh) {
    const auto& X_WA = X_WM_list[imesh];
    for (int jmesh = imesh + 1; jmesh < num_meshes(); ++jmesh) {
      const auto& X_WB = X_WM_list[jmesh];

      const bool has_intersection =
          CalcIntersection(imesh, X_WA, jmesh, X_WB, &a_pair);

      if (has_intersection) {
          PRINT_VAR(a_pair.volume);
          PRINT_VAR(a_pair.area);
          PRINT_VAR(a_pair.p_WCo.transpose());
          pairs.push_back(a_pair);
      }
    }
  }

  PRINT_VAR(pairs.size());

  return pairs;
}

}  // namespace multibody
}  // namespace drake

// Explicitly instantiates on the most common scalar types.
template class ::drake::multibody::VolumetricContactModel<double>;
// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
//    class ::drake::multibody::VolumetricContactModel)
