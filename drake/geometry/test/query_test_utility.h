#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_engine_stub.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shapes.h"

namespace drake {
namespace geometry {
namespace test {

// Convenience struct for associating data with pairs of indices. It can serve
// as a key in a set or map as long as the set/map is declared with this as
// the class as its own Hash class. E.g.,
//  unordered_map<IndexPair, float, IndexPair> pair_to_scalar_;
struct IdPair {
  GeometryId id1;
  GeometryId id2;
  IdPair() {}
  IdPair(GeometryId i1, GeometryId i2) : id1(i1), id2(i2) {}
  bool operator==(const IdPair& other) const {
    return id1 == other.id1 && id2 == other.id2;
  }
  // The pair serves is its own hash function.
  size_t operator()(const IdPair& p) const noexcept {
    size_t key = static_cast<size_t>(p.id1.get_value());
    return key << 32 | static_cast<size_t>(p.id2.get_value());
  }
};

class GeometryQueryTest : public ::testing::Test {
 protected:
  // Utility methods to create a sphere of the given radius.
  static std::unique_ptr <Shape> make_sphere(double radius) {
    return std::unique_ptr<Shape>(new Sphere(radius));
  }

  // Initialize the engine with four spheres. One at the origin and one each
  // at <1, 0, 0>, <0, 1, 0>, and <0, 0, 1>. The sphere at <1, 0, 0> is rotated
  // 90 degrees around the x-axis.
  void SetUpAxisSpheres();

  // A collection of operations to assert correctness of results. Confirms that
  // the nearest pair data is consistent when considering the first *n* spheres.
  void ExpectCorrectProximity(
      const std::vector<NearestPair<double>>& results,
      const std::vector<IdPair>& pairs);

  GeometryEngineStub<double> engine_;
  // Geometry indices for the axis-sphere configuration.
  static const double kRadius;
  static const int kSphereCount;
  // The indices of the spheres added to the engine.
  std::vector<GeometryIndex> axis_indices_;
  // The ith entry contains the id that corresponds to the ith geometry in
  // axis_indices_;
  std::vector<GeometryId> sphere_ids_;
  // The position of the near point on sphere i, for the pair (i, j), using
  // GetKeyFromIndices to map (i, j) into the key. The point on j, nearest i,
  // is just the negative value.
  std::unordered_map<IdPair, std::pair<Vector3<double>, Vector3<double>>,
                     IdPair>
      near_points_;
  // The expected distance between the given pair of identified geometry.
  std::unordered_map<IdPair, double, IdPair> distances_;
};

}  // namespace test
}  // namespace geometry
}  // namespace drake
