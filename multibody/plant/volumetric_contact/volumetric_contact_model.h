#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"

#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"

namespace drake {
namespace multibody {

template <typename T>
class VolumetricContactModel {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(VolumetricContactModel)

  VolumetricContactModel() {}


 private:
    std::vector<std::unique_ptr<ConvexPolyhedron<T>>> meshes_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::VolumetricContactModel)
