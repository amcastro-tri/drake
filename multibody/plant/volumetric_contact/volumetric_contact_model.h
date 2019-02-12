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
class VolumetricContactModel {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(VolumetricContactModel)

  VolumetricContactModel() {}
  
  int RegisterGeometry(const std::string& file_name, geometry::GeometryId id, const Vector3<double>& scales) {      
      std::vector<Wm5::Vector3<double>> points;
      std::vector<int> faces;
      LoadObj(file_name, scales, &points, &faces);

      PRINT_VAR(points.size());
      PRINT_VAR(faces.size());

      auto mesh = std::make_unique<Wm5::ConvexPolyhedron<T>>(points, faces);

      owned_meshes_.push_back(std::move(mesh));
      geometry_ids_.push_back(id);
      return static_cast<int>(geometry_ids_.size());
  }

 private:
    static void LoadObj(const std::string& file_name, const Vector3<double>& scales, std::vector<Wm5::Vector3<double>>* points, std::vector<int>* indexes);

    std::vector<geometry::GeometryId> geometry_ids_;
    std::vector<std::unique_ptr<Wm5::ConvexPolyhedron<T>>> owned_meshes_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::VolumetricContactModel)
