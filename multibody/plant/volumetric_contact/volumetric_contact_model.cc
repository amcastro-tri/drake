#include "drake/multibody/plant/volumetric_contact/volumetric_contact_model.h"

#include <utility>
#include <spruce.hh>
#include <tiny_obj_loader.h>

//#include "MTMesh.h"
//#include "Wm5Plane3.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"
#include "drake/multibody/plant/volumetric_contact/load_objs/load_objs.h"
#include "drake/multibody/plant/volumetric_contact/tools/wildmagic_tools.h"

//#include <iostream>
//#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

using drake::geometry::GeometryId;
// using Wm5::ConvexPolyhedra;

namespace drake {

namespace multibody {

template <typename T>
int VolumetricContactModel<T>::RegisterGeometry(const std::string& file_name,
                                                geometry::GeometryId id,
                                                const Vector3<double>& scales) {
  std::vector<Wm5::Vector3<double>> points;
  std::vector<int> faces;
  LoadObj(file_name, scales, &points, &faces);

#if 0
  PRINT_VAR(points.size());
  for (const auto& p : points) {
    PRINT_VAR(ToEigenVector3(p).transpose());
  }

  PRINT_VAR(faces.size());
  for (const auto& f : faces) {
    PRINT_VAR(f);
  }
#endif

  auto mesh = std::make_unique<Wm5::ConvexPolyhedron<T>>(points, faces);
  const bool is_convex = mesh->ValidateHalfSpaceProperty(
                                 16 * std::numeric_limits<double>::epsilon());
  DRAKE_THROW_UNLESS(!is_convex);

  owned_meshes_.push_back(std::move(mesh));
  geometry_ids_.push_back(id);

  const int index = geometry_id_to_index_.size();
  geometry_id_to_index_[id] = index;

  return static_cast<int>(geometry_ids_.size());
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

    // Detailed info.
    pair->detail = std::make_unique<typename VolumetricContactPair<T>::Detail>();
    pair->detail->meshA_W = meshA_W_;
    pair->detail->meshB_W = meshB_W_;
    pair->detail->intersection_W = intersection_W_;
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
          pairs.push_back(std::move(a_pair));
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
