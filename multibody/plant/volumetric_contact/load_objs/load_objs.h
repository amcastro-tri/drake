#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

#include "drake/multibody/plant/volumetric_contact/wildmagic/ConvexPolyhedron.h"

namespace drake {
namespace multibody {

std::unique_ptr<Wm5::ConvexPolyhedron<double>> LoadConvexPolyhedronFromObj(
    const std::string& file_name, const Vector3<double>& scales);

}  // namespace multibody
}  // namespace drake
