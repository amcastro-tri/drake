#include "drake/geometry/proximity/contact_surface_calculator.h"

#include <fstream>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_unit_sphere_mesh.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"

using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using Eigen::Vector3d;

namespace drake {
namespace geometry {
namespace internal {
namespace {

// This method computes the right handed normal defined by the three vertices
// of a triangle in the input argument v.
Vector3<double> CalcTriangleNormal(
    const std::vector<SurfaceVertex<double>>& v) {
  return ((v[1].r_MV() - v[0].r_MV()).cross(v[2].r_MV() - v[0].r_MV()))
      .normalized();
}

// Fixture to test the internals of ContactSurfaceCalculator.
// We will test the intersection of a level set with a single regular
// tetrahedron. All cases in the marching tetrahedra algorithm are tested.
class TetrahedronIntersectionTest : public ::testing::Test {
 protected:
  void SetUp() { unit_tet_ = MakeRegularTetrahedron(); }

  // Helper to make a tetrahedron with unit length edges.
  static std::array<Vector3<double>, 4> MakeRegularTetrahedron() {
    const double face_height = sqrt(3.0) / 2.0;
    const double tet_height = sqrt(6.0) / 3.0;
    std::array<Vector3<double>, 4> vertices;
    vertices[0] = {2. / 3. * face_height, 0.0, 0.0};
    vertices[1] = {-1. / 3. * face_height, -0.5, 0.0};
    vertices[2] = {-1. / 3. * face_height, 0.5, 0.0};
    vertices[3] = {0.0, 0.0, tet_height};
    return vertices;
  }

  std::array<Vector3<double>, 4> unit_tet_;
};

// Verifies we get an empty intersection when all vertices have the same sign.
TEST_F(TetrahedronIntersectionTest, EmptyIntersection) {
  std::vector<SurfaceVertex<double>> vertices;
  std::vector<SurfaceFace> faces;
  std::vector<double> e_M_surface;

  // All positive vertices.
  Vector4<double> phi_N = Vector4<double>::Ones();
  EXPECT_EQ(IntersectTetWithLevelSet(unit_tet_, phi_N, phi_N, &vertices, &faces,
                                     &e_M_surface),
            0);

  // All negative vertices.
  phi_N = -Vector4<double>::Ones();
  EXPECT_EQ(IntersectTetWithLevelSet(unit_tet_, phi_N, phi_N, &vertices, &faces,
                                     &e_M_surface),
            0);
}

// Case I of marching tetrahedra: only one of the vertices has a sign different
// from all the other three.
TEST_F(TetrahedronIntersectionTest, CaseI) {
  std::vector<SurfaceVertex<double>> vertices;
  std::vector<SurfaceFace> faces;
  std::vector<double> e_M_surface;
  const double kTolerance = 5.0 * std::numeric_limits<double>::epsilon();

  const int expected_num_intersections = 3;

  // Computes a unit vector in the direction of vertex "top" from the centroid
  // of the face opposite to "top".
  auto calc_unit_vector_from_base_to_top = [&unit_tet =
                                            this->unit_tet_](int top) {
    const Vector3<double>& to = unit_tet[top];
    // Index to the other three vertices.
    const Vector3<double>& v1 = unit_tet[++top % 4];
    const Vector3<double>& v2 = unit_tet[++top % 4];
    const Vector3<double>& v3 = unit_tet[++top % 4];
    const Vector3<double> from = (v1 + v2 + v3) / 3.0;
    return (to - from).normalized();
  };

  // All vertices are positive but the i-th vertex.
  for (int i = 0; i < 4; ++i) {
    vertices.clear();
    faces.clear();
    Vector4<double> phi_N = Vector4<double>::Ones();
    phi_N[i] = -1.0;
    ASSERT_EQ(IntersectTetWithLevelSet(unit_tet_, phi_N, phi_N, &vertices,
                                       &faces, &e_M_surface),
              expected_num_intersections);

    ASSERT_EQ(faces.size(), 1);
    const Vector3<int> expected_face(0, 1, 2);
    const Vector3<int> face(faces[0].vertex(0), faces[0].vertex(1),
                            faces[0].vertex(2));
    EXPECT_EQ(face, expected_face);

    ASSERT_EQ(vertices.size(), 3);

    // The negative sign is due to the fact that the i-th vertex is negative.
    const Vector3<double> expected_normal =
        -calc_unit_vector_from_base_to_top(i);
    const Vector3<double> normal = CalcTriangleNormal(vertices);
    EXPECT_TRUE(CompareMatrices(normal, expected_normal, kTolerance));
  }

  // All vertices are negative but the i-th vertex.
  for (int i = 0; i < 4; ++i) {
    vertices.clear();
    faces.clear();
    Vector4<double> phi_N = -Vector4<double>::Ones();
    phi_N[i] = 1.0;
    ASSERT_EQ(IntersectTetWithLevelSet(unit_tet_, phi_N, phi_N, &vertices,
                                       &faces, &e_M_surface),
              expected_num_intersections);

    ASSERT_EQ(faces.size(), 1);
    const Vector3<int> expected_face(0, 1, 2);
    const Vector3<int> face(faces[0].vertex(0), faces[0].vertex(1),
                            faces[0].vertex(2));
    EXPECT_EQ(face, expected_face);

    ASSERT_EQ(vertices.size(), 3);

    // The expected normal points in the direction of the i-th vertex.
    const Vector3<double> expected_normal =
        calc_unit_vector_from_base_to_top(i);
    const Vector3<double> normal = CalcTriangleNormal(vertices);
    EXPECT_TRUE(CompareMatrices(normal, expected_normal, kTolerance));
  }
}

// Case II of marching tetrahedra: two pairs of vertices with the same sign.
TEST_F(TetrahedronIntersectionTest, CaseII) {
  // This method assumes that two of vertices are positive (thus defining a
  // positive edge) and two vertices are negative (thus defining a negative
  // edge).
  auto calc_unit_vector_from_negative_edge_to_positive_edge =
      [& unit_tet = this->unit_tet_](const Vector4<double>& phi) {
        Vector3<double> from = Vector3<double>::Zero();
        Vector3<double> to = Vector3<double>::Zero();
        for (int i = 0; i < 4; ++i) {
          (phi[i] > 0.0 ? to : from) += unit_tet[i];
        }
        to /= 2.0;
        from /= 2.0;
        return (to - from).normalized();
      };

  auto verify_case2 = [&](const Vector4<double>& phi_N) {
    const double kTolerance = 5.0 * std::numeric_limits<double>::epsilon();
    const int expected_num_vertices = 5;

    std::vector<SurfaceVertex<double>> vertices;
    std::vector<SurfaceFace> faces;
    std::vector<double> e_M_surface;
    ASSERT_EQ(IntersectTetWithLevelSet(unit_tet_, phi_N, phi_N, &vertices,
                                       &faces, &e_M_surface),
              expected_num_vertices);

    ASSERT_EQ(faces.size(), 4);
    ASSERT_EQ(vertices.size(), 5);

    // To verify the computation of the normals we make use of the symmetry of
    // the problem, for which we have a regular tetrahedron and we set all
    // vertices to have the same absolute value of phi_N. With this setup, all
    // triangles lie on the same plane, thus with the same normal.
    const Vector3<double> expected_normal =
        calc_unit_vector_from_negative_edge_to_positive_edge(phi_N);

    for (int i = 0; i < 4; ++i) {
      const SurfaceFace& t = faces[i];
      std::vector<SurfaceVertex<double>> triangle_vertices = {
          vertices[t.vertex(0)], vertices[t.vertex(1)], vertices[t.vertex(2)]};
      const Vector3<double> normal = CalcTriangleNormal(triangle_vertices);
      // This check has the nice side effect of checking triangles are properly
      // oriented with the right-handed normal pointing towards the positive
      // half of the tetrahedron.
      EXPECT_TRUE(CompareMatrices(normal, expected_normal, kTolerance));
    }
  };

  // Verify case II for the six different cases.
  verify_case2(Vector4<double>(1.0, 1.0, -1.0, -1.0));
  verify_case2(Vector4<double>(-1.0, -1.0, 1.0, 1.0));
  verify_case2(Vector4<double>(-1.0, 1.0, -1.0, 1.0));
  verify_case2(Vector4<double>(1.0, -1.0, 1.0, -1.0));
  verify_case2(Vector4<double>(1.0, -1.0, -1.0, 1.0));
  verify_case2(Vector4<double>(-1.0, 1.0, 1.0, -1.0));
}

// Fixture to test the intersection of a level set with a unit box.
class BoxPlaneIntersectionTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Generate a partition of the unit cube into 6 tetrahedra.
    std::vector<VolumeVertex<double>> vertices = {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {1.0, 1.0, 1.0}, {0.0, 1.0, 1.0}};
    // translate origin to the geometric center of the box.
    std::transform(
        vertices.begin(), vertices.end(), vertices.begin(),
        [](const VolumeVertex<double>& v) {
          return VolumeVertex<double>(v.r_MV() - 0.5 * Vector3<double>::Ones());
        });

    const int e0[] = {0, 1, 3, 7};
    const int e1[] = {1, 0, 4, 7};
    const int e2[] = {1, 2, 3, 7};
    const int e3[] = {2, 1, 5, 7};
    const int e4[] = {1, 4, 5, 7};
    const int e5[] = {2, 5, 6, 7};
    using VE = VolumeElement;
    std::vector<VolumeElement> elements = {VE(e0), VE(e1), VE(e2),
                                           VE(e3), VE(e4), VE(e5)};
    box_B_ = std::make_unique<VolumeMesh<double>>(std::move(elements),
                                                  std::move(vertices));
    half_space_H_ = [](const Vector3<double>& p_HQ) { return p_HQ[2]; };

    // Not relevant for this test since all vertices are on the surface.
    e_B_ = std::vector<double>(8, 0.0);
  }

  double CalcSurfaceArea(const SurfaceMesh<double>& mesh) {
    double area = 0;
    for (SurfaceFaceIndex f(0); f < mesh.num_faces(); ++f) {
      area += mesh.area(f);
    }
    return area;
  }

  // Mesh modeling a box, with its vertex positions expressed in the frame of
  // the box B.
  std::unique_ptr<VolumeMesh<double>> box_B_;

  // Value of a scalar field e_M defined on the volume mesh box_B_.
  std::vector<double> e_B_;

  // A level set function, chosen to be the distance function, for a half space.
  // It is defined as a function φ: ℝ³ → ℝ with the input position vector
  // expressed in the frame H of the half space. Frame H is defined such that Hz
  // normal to the half space points into the positive direction (outwards).
  std::function<double(const Vector3<double>&)> half_space_H_;
};

// The bottom face of the box is machine epsilon from being flat on the
// half-space, making contact. Currently, this might lead to double counting the
// contact interface between neighboring tetrahedra. However, we can still
// detect this situation and throw an exception with an appropriate message. We
// verify this.
TEST_F(BoxPlaneIntersectionTest, ImminentContact) {
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  std::vector<double> e_mn_surface;

  // The box overlaps the plane by kEpsilon. Expect intersection.
  {
    const math::RigidTransformd X_HB =
        Translation3<double>(0.0, 0.0, 0.5 - 5 * kEpsilon);
    DRAKE_EXPECT_THROWS_MESSAGE(CalcZeroLevelSetInMeshDomain(
            *box_B_, half_space_H_, X_HB, e_B_, &e_mn_surface),
        std::logic_error,
        "One or more faces of this tetrahedron are close to being a zero "
        "crossing.*");
  }

  // The box is on top of the plane by kEpsilon. Expect no intersection.
  {
    const math::RigidTransformd X_HB =
        Translation3<double>(0.0, 0.0, 0.5 + 5 * kEpsilon);
    DRAKE_EXPECT_THROWS_MESSAGE(CalcZeroLevelSetInMeshDomain(
            *box_B_, half_space_H_, X_HB, e_B_, &e_mn_surface),
        std::logic_error,
        "One or more faces of this tetrahedron are close to being a zero "
        "crossing.*");
  }
}

// When one of the axes of the box is aligned with the plane normal the contact
// surface simply is a unit square. The contact area however gets discretized
// by a complex mesh of triangles given the non-homogenous tessellation of the
// original box. Therefore verifying that the contact surface has unit area is a
// very good measure of the success of the algorithm.
// We do this for several orientations.
TEST_F(BoxPlaneIntersectionTest, VerifyContactArea) {
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  const double kTolerance = 5.0 * kEpsilon;

  const auto Rx_pi_2 = RollPitchYawd(M_PI_2, 0.0, 0.0);
  const auto Ry_pi_2 = RollPitchYawd(0.0, M_PI_2, 0.0);
  const auto Rx_pi = RollPitchYawd(M_PI, 0.0, 0.0);
  const auto Ry_pi = RollPitchYawd(0.0, M_PI, 0.0);
  const Vector3<double> lowest(0.0, 0.0, -0.4);
  const Vector3<double> middle(0.0, 0.0, 0.0);
  const Vector3<double> highest(0.0, 0.0, 0.4);
  // We choose a number of arbitrary poses but such that the plane is
  // perpendicular to one of the main axes of the box. Therefore we know that
  // the intersected surface is a 1x1 square.
  std::vector<math::RigidTransformd> poses = {
      Translation3<double>(lowest),
      Translation3<double>(middle),
      Translation3<double>(highest),
      RigidTransformd(Rx_pi_2, lowest),
      RigidTransformd(Rx_pi_2, middle),
      RigidTransformd(Rx_pi_2, highest),
      RigidTransformd(Ry_pi_2, lowest),
      RigidTransformd(Ry_pi_2, middle),
      RigidTransformd(Ry_pi_2, highest),
      RigidTransformd(Rx_pi, lowest),
      RigidTransformd(Rx_pi, middle),
      RigidTransformd(Rx_pi, highest),
      RigidTransformd(Ry_pi, lowest),
      RigidTransformd(Ry_pi, middle),
      RigidTransformd(Ry_pi, highest)};

  std::vector<double> e_mn_surface;
  for (const auto& X_HB : poses) {
    const SurfaceMesh<double> contact_surface = CalcZeroLevelSetInMeshDomain(
        *box_B_, half_space_H_, X_HB, e_B_, &e_mn_surface);
    EXPECT_NEAR(CalcSurfaceArea(contact_surface), 1.0, kTolerance);
  }
}

// Verify the algorithm returns no intersections when the box is over the plane.
TEST_F(BoxPlaneIntersectionTest, NoIntersection) {
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  const double kTolerance = 5.0 * kEpsilon;

  const auto Rx_pi_2 = RollPitchYawd(M_PI_2, 0.0, 0.0);
  const auto Ry_pi_2 = RollPitchYawd(0.0, M_PI_2, 0.0);
  const auto Rx_pi = RollPitchYawd(M_PI, 0.0, 0.0);
  const auto Ry_pi = RollPitchYawd(0.0, M_PI, 0.0);
  // We choose a an arbitrary number of poses but such that the box is entirely
  // above the plane.
  const Vector3<double> lowest(0.0, 0.0, 0.6);
  const Vector3<double> middle(0.0, 0.0, 0.8);
  const Vector3<double> highest(0.0, 0.0, 1.0);
  std::vector<math::RigidTransformd> poses = {
      Translation3<double>(lowest),
      Translation3<double>(middle),
      Translation3<double>(highest),
      RigidTransformd(Rx_pi_2, lowest),
      RigidTransformd(Rx_pi_2, middle),
      RigidTransformd(Rx_pi_2, highest),
      RigidTransformd(Ry_pi_2, lowest),
      RigidTransformd(Ry_pi_2, middle),
      RigidTransformd(Ry_pi_2, highest),
      RigidTransformd(Rx_pi, lowest),
      RigidTransformd(Rx_pi, middle),
      RigidTransformd(Rx_pi, highest),
      RigidTransformd(Ry_pi, lowest),
      RigidTransformd(Ry_pi, middle),
      RigidTransformd(Ry_pi, highest)};

  for (const auto& X_HB : poses) {
    std::vector<double> e_mn_surface;
    const SurfaceMesh<double> contact_surface = CalcZeroLevelSetInMeshDomain(
        *box_B_, half_space_H_, X_HB, e_B_, &e_mn_surface);
    EXPECT_NEAR(CalcSurfaceArea(contact_surface), 0.0, kTolerance);
  }
}

// This test verifies the computation of the contact surface between a
// tessellated sphere and a half-space represented by a level set function.
// In particular, we verify that the vertices on the contact surface are
// properly interpolated to lie on the plane within a circle of the expected
// radius, and normals point towards the positive side of the plane.
GTEST_TEST(SpherePlaneIntersectionTest, VerifyInterpolations) {
  const double kTolerance = 5.0 * std::numeric_limits<double>::epsilon();

  // A tessellation of a unit sphere. Vertices are in the mesh frame M.
  // This creates a volume mesh with over 32K tetrahedra.
  //const VolumeMesh<double> sphere_M = MakeUnitSphereMesh<double>(4);
  std::unique_ptr<HydroelasticField<double>> sphere_field =
      MakeUnitSphereHydroelasticField<double>(4);
  const VolumeMesh<double>& sphere_M = sphere_field->volume_mesh();
  const std::vector<double>& e_mn = sphere_field->scalar_field().values();

  // A level set for a half-space, as a function of the position vector p_WQ for
  // points Q in the world frame W.
  std::function<double(const Vector3<double>&)> half_space_W =
      [](const Vector3<double>& p_WQ) { return p_WQ[2]; };

  // We place the sphere above the half-space but overlapping, such that the
  // deepest penetration point is at a distance equal to 0.3 (the sphere radius
  // is 1.0, dimensionless for the purposes of this test).
  const double height = 0.7;
  const RigidTransformd X_WM = Translation3<double>(0.0, 0.0, height);

  // The contact surface is expressed in the frame of the level set, in this
  // case the world frame W.
  std::vector<double> e_mn_surface;
  const SurfaceMesh<double> contact_surface_W = CalcZeroLevelSetInMeshDomain(
      sphere_M, half_space_W, X_WM, e_mn, &e_mn_surface);
  // Assert non-empty intersection.
  ASSERT_GT(contact_surface_W.num_faces(), 0);

  for (SurfaceVertexIndex v(0); v < contact_surface_W.num_vertices(); ++v) {
    const Vector3<double>& p_WV = contact_surface_W.vertex(v).r_MV();
    // We verify that the postions were correctly interpolated to lie on the
    // plane.
    EXPECT_NEAR(p_WV[2], 0.0, kTolerance);

    // Verify surface vertices lie within a circle of the expected radius.
    const double surface_radius = std::sqrt(1.0 - height * height);
    const double radius = p_WV.norm();  // since z component is zero.
    EXPECT_LE(radius, surface_radius);
  }

  // Verify all normals point towards the positive side of the plane.
  for (SurfaceFaceIndex f(0); f < contact_surface_W.num_faces(); ++f) {
    const SurfaceFace& face = contact_surface_W.element(f);
    std::vector<SurfaceVertex<double>> vertices_W = {
        contact_surface_W.vertex(face.vertex(0)),
        contact_surface_W.vertex(face.vertex(1)),
        contact_surface_W.vertex(face.vertex(2))};
    const Vector3<double> normal_W = CalcTriangleNormal(vertices_W);
    // We expect the normals to point in the same direction as the halfsapce's
    // outward facing normal, according to the convention used by
    // CalcZeroLevelSetInMeshDomain().
    // Additional precision is lost during the computation of the surface. Most
    // likely due to:
    //   1) precision loss in the frame transformation between the sphere and
    //      the plane.
    //   2) Interpolation within the tetrahedra.
    EXPECT_TRUE(CompareMatrices(normal_W, Vector3d::UnitZ(), 40 * kTolerance));
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
