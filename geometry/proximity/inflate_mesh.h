#pragma once

#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

VolumeMesh<double> MakeInflatedMesh(const VolumeMesh<double>& mesh,
                                    double margin);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
