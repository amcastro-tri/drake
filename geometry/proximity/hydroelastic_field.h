#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_field.h"

namespace drake {
namespace geometry {
namespace internal {

template <typename T>
class HydroelasticField {
 public:
  HydroelasticField(std::unique_ptr<VolumeMesh<T>> mesh,
                    std::unique_ptr<VolumeMeshFieldLinear<T, T>> e_mn)
      : mesh_(std::move(mesh)), e_mn_(std::move(e_mn)), grad_h_MN_M_(nullptr) {
    // std::unique_ptr<VolumeMeshFieldLinear<Vector3<T>, T>> grad_h_MN_M)
  }

  HydroelasticField(HydroelasticField&&) = default;
  HydroelasticField& operator=(HydroelasticField&&) = default;

  const VolumeMesh<T>& volume_mesh() const { return *mesh_; }

  const VolumeMeshFieldLinear<T, T>& scalar_field() const { return *e_mn_; }

 private:
  /** The surface mesh of the contact surface 𝕊ₘₙ between M and N. */
  std::unique_ptr<VolumeMesh<T>> mesh_;
  /** Represents the scalar field eₘₙ on the surface mesh. */
  std::unique_ptr<VolumeMeshFieldLinear<T, T>> e_mn_;
  /** Represents the vector field ∇hₘₙ on the surface mesh, expressed in M's
    frame */
  std::unique_ptr<VolumeMeshFieldLinear<Vector3<T>, T>> grad_h_MN_M_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
