#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;

namespace drake {
namespace multibody {

template <typename T>
struct VolumetricContactPair {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumetricContactPair)
  VolumetricContactPair() = default;

  /** The id of the first geometry in the contact. */
  int index_A;
  /** The id of the second geometry in the contact. */
  int index_B;
  // Contact point. The "mean" in the intersected contact volume.
  Vector3<T> p_WCo;
  /** The unit-length normal which defines the penetration direction, pointing
   from geometry B into geometry A, measured and expressed in the world frame.
   It _approximates_ the normal to the plane on which the contact patch lies. */
  Vector3<T> nhat_BA_W;
  // The volume of the intersection.
  T volume;
  // The area of the intersection.
  T area;

  struct Detail {
    // Expensive to save info. Used for debugging.
    Wm5::ConvexPolyhedron<T> meshA_W;
    Wm5::ConvexPolyhedron<T> meshB_W;
    Wm5::ConvexPolyhedron<T> intersection_W;
  };
  std::unique_ptr<Detail> detail;

};

template <typename T>
class VolumetricContactModel {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(VolumetricContactModel)

  VolumetricContactModel() {}
  
  int num_meshes() const { return static_cast<int>(owned_meshes_.size()); }

  int RegisterGeometry(const std::string& file_name, geometry::GeometryId id, const Vector3<double>& scales);

  bool CalcIntersection(int indexA, const Isometry3<double>& X_WA,
                        int indexB, const Isometry3<double>& X_WB,
                        VolumetricContactPair<T>* pair) const;

  std::vector<VolumetricContactPair<T>> CalcAllIntersectionPairs(
      const std::vector<Isometry3<T>>& X_WM_list) const;

 private:
  static void LoadObj(const std::string& file_name,
                      const Vector3<double>& scales,
                      std::vector<Wm5::Vector3<double>>* points,
                      std::vector<int>* indexes);

  static void ReExpressMesh(const Wm5::ConvexPolyhedron<T>& mesh_B,
                            const Isometry3<T>& X_AB,
                            Wm5::ConvexPolyhedron<T>* mesh_A);

  std::vector<geometry::GeometryId> geometry_ids_;
  std::unordered_map<geometry::GeometryId, int> geometry_id_to_index_;
  std::vector<std::unique_ptr<Wm5::ConvexPolyhedron<T>>> owned_meshes_;

  //mutable Wm5::ConvexPolyhedron<T> meshA_W_;
  //mutable Wm5::ConvexPolyhedron<T> meshB_W_;
  //mutable Wm5::ConvexPolyhedron<T> intersection_W_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::VolumetricContactModel)
