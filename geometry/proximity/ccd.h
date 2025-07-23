#pragma once

#include <array>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

bool point_triangle_ccd(const Vector3d& p0,
                        const Vector3d& v00,
                        const Vector3d& v10,
                        const Vector3d& v20,
                        const Vector3d& p1,
                        const Vector3d& v01,
                        const Vector3d& v11,
                        const Vector3d& v21, double* toi);

bool edge_edge_ccd(const Vector3d& p00,
                   const Vector3d& p10,
                   const Vector3d& q00,
                   const Vector3d& q10,
                   const Vector3d& p01,
                   const Vector3d& p11,
                   const Vector3d& q01,
                   const Vector3d& q11, double* toi);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
