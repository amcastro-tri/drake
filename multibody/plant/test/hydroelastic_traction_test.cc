#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/hydroelastic_traction_calculator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

#include <fstream>

namespace drake {

using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::MeshFieldLinear;
using geometry::SceneGraph;
using geometry::SurfaceFaceIndex;
using geometry::SurfaceFace;
using geometry::SurfaceMesh;
using geometry::SurfaceVertex;
using geometry::SurfaceVertexIndex;
using math::RigidTransform;
using math::RigidTransformd;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

namespace multibody {
namespace internal {

// Creates a surface mesh that covers the bottom of the "wetted surface",
// where the wetted surface is the part of the box that would be wet if the
// halfspace were a fluid. The entire wetted surface *would* yield
// an open box with five faces but, for simplicity, we'll only
// use the bottom face (two triangles).
std::unique_ptr<SurfaceMesh<double>> CreateSurfaceMesh() {
  std::vector<SurfaceVertex<double>> vertices;
  std::vector<SurfaceFace> faces;

  // Create the vertices, all of which are offset vectors defined in the
  // halfspace body frame.
  vertices.emplace_back(Vector3<double>(0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<double>(-0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<double>(-0.5, -0.5, -0.5));
  vertices.emplace_back(Vector3<double>(0.5, -0.5, -0.5));

  // Create the face comprising two triangles. The box penetrates into the
  // z = 0 plane from above. The contact surface should be constructed such that
  // the normals point out of geometry M and into geometry N. We assume that the
  // half space and box are geometries M and N, respectively. So, that means
  // the contact normals point downwards. We select windings for the triangle
  // vertices so that the normal will point in the [0, 0, -1] direction.
  //
  //             +z  +y
  //        v1 ___|__/____ v0
  //          /   | /    /
  //         /    |/    /
  //      --/----------/-- +x
  //       /     /|   /
  //   v2 /_____/_|__/ v3
  //           /  |
  faces.emplace_back(
      SurfaceVertexIndex(0), SurfaceVertexIndex(2), SurfaceVertexIndex(1));
  faces.emplace_back(
      SurfaceVertexIndex(2), SurfaceVertexIndex(0), SurfaceVertexIndex(3));

  auto mesh = std::make_unique<SurfaceMesh<double>>(
      std::move(faces), std::move(vertices));

  for (SurfaceFaceIndex f(0); f < mesh->num_faces(); ++f) {
    // Can't use an ASSERT_TRUE here because it interferes with the return
    // value.
    if (!CompareMatrices(mesh->face_normal(f), -Vector3<double>::UnitZ(),
        std::numeric_limits<double>::epsilon())) {
      throw std::logic_error("Malformed mesh; normals don't point downwards");
    }
  }

  return mesh;
}

GeometryId FindGeometry(
    const MultibodyPlant<double>& plant, const std::string body_name) {
  const auto& geometries = plant.GetCollisionGeometriesForBody(
      plant.GetBodyByName(body_name));
  DRAKE_DEMAND(geometries.size() == 1);
  return geometries[0];
}

// Creates a contact surface between the two given geometries.
std::unique_ptr<ContactSurface<double>> CreateContactSurface(
    GeometryId halfspace_id, GeometryId block_id,
    const math::RigidTransform<double>& X_WH) {
  // Create the surface mesh first (in the halfspace frame); we'll transform
  // it to the world frame *after* we use the vertices in the halfspace frame
  // to determine the hydroelastic pressure.
  auto mesh = CreateSurfaceMesh();

  // Create the "e" field values (i.e., "hydroelastic pressure") using
  // negated "z" values.
  std::vector<double> e_MN(mesh->num_vertices());
  for (SurfaceVertexIndex i(0); i < mesh->num_vertices(); ++i)
    e_MN[i] = -mesh->vertex(i).r_MV()[2];

  // Now transform the mesh to the world frame, as ContactSurface specifies.
  mesh->TransformVertices(X_WH);

  SurfaceMesh<double>* mesh_pointer = mesh.get();
  return std::make_unique<ContactSurface<double>>(
      halfspace_id, block_id, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e_MN", std::move(e_MN), mesh_pointer));
}

// This fixture defines a contacting configuration between a box and a
// half-space in a local frame, Y. In Frame Y, half-space Frame H is
// positioned such that it passes through the origin and its normal Hz is
// aligned with Yz. In other words, the pose X_YH is the identity pose.
// The box pose X_YB is also set to be the identity pose and therefore the
// geometric center of the box is at the origin Yo, with the bottom (in frame Y)
// half of the box overlapping the half-space. The fixture unit-tests
// an arbitrary set of poses X_WY of frame Y in the world frame W to assess the
// frame invariance of the computed results, which only depend (modulo the
// "expressed-in" frame) on the relative pose of the bodies.
class MultibodyPlantHydroelasticTractionTests :
public ::testing::TestWithParam<RigidTransform<double>> {
 public:
  const HydroelasticTractionCalculator<double>& traction_calculator() const {
    return traction_calculator_;
  }

  const HydroelasticTractionCalculator<double>::Data& calculator_data() {
    return *calculator_data_;
  }

  // Sets all kinematic quantities (contact surface will be left untouched).
  // Note: see Data constructor for description of these parameters.
  void set_calculator_data(
      const RigidTransform<double>& X_WA, const RigidTransform<double>& X_WB,
      const SpatialVelocity<double>& V_WA,
      const SpatialVelocity<double>& V_WB) {
    calculator_data_ = std::make_unique<HydroelasticTractionCalculator<double>::
        Data>(
            X_WA, X_WB, V_WA, V_WB, &contact_surface());
  }

  const ContactSurface<double>& contact_surface() const {
      return *contact_surface_;
  }

  // Returns the default numerical tolerance.
  double tol() const { return tol_; }

  // Computes the spatial tractions due to the hydroelastic model. This computes
  // the spatial traction (in units of N/m²) at Vertex 0 of Triangle 0. We're
  // using the SpatialForce data type to hold these tractions, but they are
  // not forces. We'll use "Ft" rather than "F" here so we remember these are
  // tractions.
  void ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      double dissipation, double mu_coulomb, SpatialForce<double>* Ft_Ao_W,
      SpatialForce<double>* Ft_Bo_W) {
    UpdateCalculatorData();

    // First compute the traction applied to Body A at point Q, expressed in the
    // world frame.
    HydroelasticQuadraturePointData<double> output =
        traction_calculator().CalcTractionAtPoint(
            calculator_data(), SurfaceFaceIndex(0),
            SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0), dissipation,
            mu_coulomb);

    // Compute the expected point of contact in the world frame. The class
    // definition and SetUp() note that the parameter to this test transforms
    // both bodies from their definition in Frame Y to the world frame.
    const RigidTransform<double>& X_WY = GetParam();
    const Vector3<double> p_YQ(0.5, 0.5, -0.5);
    const Vector3<double> p_WQ_expected = X_WY * p_YQ;

    // Verify the point of contact.
    for (int i = 0; i < 3; ++i)
      ASSERT_NEAR(output.p_WQ[i], p_WQ_expected[i], tol()) << i;

    const SpatialForce<double> Ft_Ac_W =
        traction_calculator().ComputeSpatialTractionAtAcFromTractionAtAq(
            calculator_data(), output.p_WQ, output.traction_Aq_W);

    // Shift to body origins. Traction on body B is equal and opposite.
    const Vector3<double>& p_WC = calculator_data().p_WC;
    const Vector3<double>& p_WAo = calculator_data().X_WA.translation();
    const Vector3<double>& p_WBo = calculator_data().X_WB.translation();
    const Vector3<double> p_CAo_W = p_WAo - p_WC;
    const Vector3<double> p_CBo_W = p_WBo - p_WC;
    *Ft_Ao_W = Ft_Ac_W.Shift(p_CAo_W);
    *Ft_Bo_W = -(Ft_Ac_W.Shift(p_CBo_W));
  }

  void SetBoxTranslationalVelocity(const Vector3<double>& v) {
    plant_->SetFreeBodySpatialVelocity(plant_context_,
        plant_->GetBodyByName("box"),
        SpatialVelocity<double>(Vector3<double>::Zero(), v));
  }

  void UpdateCalculatorData() {
    // Get the bodies that the two geometries are affixed to. We'll call these
    // A and B.
    const auto& query_object = plant_->get_geometry_query_input_port().
        template Eval<geometry::QueryObject<double>>(*plant_context_);
    const geometry::FrameId frameM_id = query_object.inspector().GetFrameId(
        contact_surface_->id_M());
    const geometry::FrameId frameN_id = query_object.inspector().GetFrameId(
        contact_surface_->id_N());
    const Body<double>& bodyA = *plant_->GetBodyFromFrameId(frameM_id);
    const Body<double>& bodyB = *plant_->GetBodyFromFrameId(frameN_id);

    // Get the poses of the two bodies in the world frame.
    const math::RigidTransform<double> X_WA =
        plant_->EvalBodyPoseInWorld(*plant_context_, bodyA);
    const math::RigidTransform<double> X_WB =
        plant_->EvalBodyPoseInWorld(*plant_context_, bodyB);

    // Get the spatial velocities for the two bodies (at the body frames).
    const SpatialVelocity<double> V_WA = plant_->EvalBodySpatialVelocityInWorld(
        *plant_context_, bodyA);
    const SpatialVelocity<double> V_WB = plant_->EvalBodySpatialVelocityInWorld(
        *plant_context_, bodyB);

    // (Re)-initialize the traction calculator data.
    set_calculator_data(X_WA, X_WB, V_WA, V_WB);
  }

  void ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      double dissipation, double mu_coulomb,
      SpatialForce<double>* F_Ao_W, SpatialForce<double>* F_Bo_W) {
    UpdateCalculatorData();

    SpatialForce<double> F_Ac_W;
    std::vector<HydroelasticQuadraturePointData<double>> quadrature_point_data;
    traction_calculator().ComputeSpatialForcesAtCentroidFromHydroelasticModel(
        calculator_data(), dissipation, mu_coulomb, &quadrature_point_data,
        &F_Ac_W);

    traction_calculator().ShiftSpatialForcesAtCentroidToBodyOrigins(
        calculator_data(), F_Ac_W, F_Ao_W, F_Bo_W);
  }

 private:
  void SetUp() override {
    // Read the two bodies into the plant.
    DiagramBuilder<double> builder;
    SceneGraph<double>* scene_graph;
    std::tie(plant_, scene_graph) = AddMultibodyPlantSceneGraph(&builder, 0.0);
    MultibodyPlant<double>& plant = *plant_;
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/plant/test/block_on_halfspace.sdf");
    Parser(&plant, scene_graph).AddModelFromFile(full_name);

    plant.Finalize();
    diagram_ = builder.Build();

    ASSERT_EQ(plant.num_velocities(), 12);
    ASSERT_EQ(plant.num_positions(), 14);

    // Create a context for this system.
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(plant, context_.get());

    // See class documentation for description of Frames Y, B, and H.
    const RigidTransform<double>& X_WY = GetParam();
    const auto& X_YH = RigidTransform<double>::Identity();
    const auto& X_YB = RigidTransform<double>::Identity();
    const RigidTransform<double> X_WH = X_WY * X_YH;
    const RigidTransform<double> X_WB = X_WY * X_YB;
    plant.SetFreeBodyPose(plant_context_, plant.GetBodyByName("ground"), X_WH);
    plant.SetFreeBodyPose(plant_context_, plant.GetBodyByName("box"), X_WB);

    GeometryId halfspace_id = internal::FindGeometry(plant, "ground");
    GeometryId block_id = internal::FindGeometry(plant, "box");
    contact_surface_ = CreateContactSurface(halfspace_id, block_id, X_WH);
  }

  const double tol_{10 * std::numeric_limits<double>::epsilon()};
  MultibodyPlant<double>* plant_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  systems::Context<double>* plant_context_;
  std::unique_ptr<ContactSurface<double>> contact_surface_;
  HydroelasticTractionCalculator<double> traction_calculator_;
  std::unique_ptr<
      HydroelasticTractionCalculator<double>::Data> calculator_data_;
};

// Tests the traction calculation without any frictional or dissipation
// tractions.
TEST_P(MultibodyPlantHydroelasticTractionTests, VanillaTraction) {
  const double dissipation = 0.0;  // Units: s/m.
  const double mu_coulomb = 0.0;

  // Compute the spatial tractions at the origins of the body frames.
  multibody::SpatialForce<double> Ft_Ao_W, Ft_Bo_W;
  ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &Ft_Ao_W, &Ft_Bo_W);

  // Re-express the spatial tractions in Y's frame for easy interpretability.
  // Note that f and tau here are still tractions, so our notation is being
  // misused a little here.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const Vector3<double> f_Bo_Y = R_WY.transpose() * Ft_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * Ft_Bo_W.rotational();

  // Check the spatial traction at p. We know that geometry M is the halfspace,
  // so we'll check the spatial traction for geometry N instead. Note that the
  // tangential components are zero.
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], 0.5, tol());

  // A moment on the box will be generated due to the normal traction. The
  // origin of the box frame is located at (0,0,0) in the Y frame.
  // The moment arm at the point will be (.5, .5, -.5), again in the Y frame.
  // Crossing this vector with the traction at that point (0, 0, 0.5) yields the
  // following.
  EXPECT_NEAR(tau_Bo_Y[0], 0.25, tol());
  EXPECT_NEAR(tau_Bo_Y[1], -0.25, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0, tol());

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((Ft_Bo_W.translational() + Ft_Ao_W.translational()).norm(),
      0, tol());
}

// Tests the traction calculation with friction but without dissipation
// tractions.
TEST_P(MultibodyPlantHydroelasticTractionTests, TractionWithFriction) {
  const double dissipation = 0.0;  // Units: s/m.
  const double mu_coulomb = 1.0;

  // Give the box an initial (vertical) velocity along the +x axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  SetBoxTranslationalVelocity(R_WY * Vector3<double>(1, 0, 0));

  // Compute the spatial tractions at the origins of the body frames.
  multibody::SpatialForce<double> Ft_Ao_W, Ft_Bo_W;
  ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &Ft_Ao_W, &Ft_Bo_W);

  // Re-express the spatial tractions in Y's frame for easy interpretability.
  // Note that f and tau here are still tractions, so our notation is being
  // misused a little here.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * Ft_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * Ft_Bo_W.rotational();

  // Check the spatial traction at p. We know that geometry M is the halfspace,
  // so we'll check the spatial traction for geometry N instead. The coefficient
  // of friction is unity, so the total frictional traction will have
  // approximately the same magnitude as the normal traction. Note that the
  // regularized Coulomb friction model requires backing off of the tolerance
  // for comparing frictional traction components (against the Coulomb model);
  // the units are dissimilar (m/s vs. N and Nm) but using this as the tolerance
  // has proven useful.
  const double field_value = 0.5;  // in N/m².
  const double regularization_scalar =
      traction_calculator().regularization_scalar();
  EXPECT_NEAR(f_Bo_Y[0], -mu_coulomb * field_value, regularization_scalar);
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], field_value, tol());

  // A moment on the box will be generated due to the traction. The
  // origin of the box frame is located at (0,0,0) in the Y frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (-.5, 0, 0.5) yields the following.
  EXPECT_NEAR(tau_Bo_Y[0], 0.25, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.0, regularization_scalar);
  EXPECT_NEAR(tau_Bo_Y[2], 0.25, regularization_scalar);

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((Ft_Bo_W.translational() + Ft_Ao_W.translational()).norm(),
      0, tol());
}

// Tests the traction calculation with dissipation tractions but without
// friction.
TEST_P(MultibodyPlantHydroelasticTractionTests, TractionWithDissipation) {
  const double dissipation = 1.0;  // Units: s/m.
  const double mu_coulomb = 0.0;

  // Give the box an initial (vertical) velocity along the -z axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const double separating_velocity = -1.0;
  SetBoxTranslationalVelocity(R_WY *
      Vector3<double>(0, 0, separating_velocity));

  // Compute the magnitude of the normal traction. Note that the damping
  // constant at each point will be field value * dissipation coefficient.
  const double field_value = 0.5;  // in N/m².
  const double c = field_value * dissipation;  // N/m² * s/m = Ns/m³.
  const double damping_traction_magnitude = c * -separating_velocity;  // N/m².
  const double normal_traction_magnitude = field_value +
      damping_traction_magnitude;

  // Compute the spatial tractions at the origins of the body frames.
  multibody::SpatialForce<double> Ft_Ao_W, Ft_Bo_W;
  ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &Ft_Ao_W, &Ft_Bo_W);

  // Re-express the spatial tractions in Y's frame for easy interpretability.
  // Note that f and tau here are still tractions, so our notation is being
  // misused a little here.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * Ft_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * Ft_Bo_W.rotational();

  // Check the spatial traction at p. We know that geometry M is the halfspace,
  // so we'll check the spatial traction for geometry N instead. The coefficient
  // of friction is unity, so the total frictional traction will have the same
  // magnitude as the normal traction.
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], normal_traction_magnitude, tol());

  // A moment on the box will be generated due to the traction. The
  // origin of the box frame is located at (0,0,0) in the world frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (0, 0, normal_traction_magnitude) yields
  // the following.
  EXPECT_NEAR(tau_Bo_Y[0], 0.5 * normal_traction_magnitude, tol());
  EXPECT_NEAR(tau_Bo_Y[1], -0.5 * normal_traction_magnitude, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((Ft_Bo_W.translational() + Ft_Ao_W.translational()).norm(),
      0, tol());
}

// Tests the traction calculation over an entire contact patch without
// friction or dissipation effecting the traction.
TEST_P(MultibodyPlantHydroelasticTractionTests, VanillaTractionOverPatch) {
  const double dissipation = 0.0;
  const double mu_coulomb = 0.0;

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Ao_W, F_Bo_W;
  ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &F_Ao_W, &F_Bo_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const Vector3<double> f_Bo_Y = R_WY.transpose() * F_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * F_Bo_W.rotational();

  // Check the spatial force. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead.
  const double field_value = 0.5;  // in N/m².
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], field_value, tol());

  // We expect no moment on the box.
  EXPECT_NEAR(tau_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());
}

// Tests the traction calculation over an entire contact patch with
// friction but without dissipation effecting the traction.
TEST_P(MultibodyPlantHydroelasticTractionTests, FrictionalTractionOverPatch) {
  const double dissipation = 0.0;
  const double mu_coulomb = 1.0;

  // Give the box an initial (vertical) velocity along the +x axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  SetBoxTranslationalVelocity(R_WY * Vector3<double>(1, 0, 0));

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Ao_W, F_Bo_W;
  ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &F_Ao_W, &F_Bo_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * F_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * F_Bo_W.rotational();

  // Check the spatial force. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead.  Note that the
  // regularized Coulomb friction model requires backing off of the tolerance
  // for comparing frictional traction components (against the Coulomb model);
  // the units are dissimilar (m/s vs. N and Nm) but using this as the tolerance
  // has proven useful.
  const double field_value = 0.5;  // in N/m².
  const double regularization_scalar =
      traction_calculator().regularization_scalar();
  EXPECT_NEAR(f_Bo_Y[0], -mu_coulomb * field_value, regularization_scalar);
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], field_value, tol());

  // A moment on the box will be generated due to the integral of the tractions.
  // The origin of the box frame is located at (0,0,0) in the Y frame.
  // The mean of all of the moment arms (we expect the tractions at each point
  // on the contact surface to be identical) will be (0, 0, -.5). Crossing this
  // vector with the traction at each point (-.5, 0, 0.5) yields (0, 0.25, 0).
  // The area of the contact surface is unity, so scaling this vector by the
  // area changes only the units, not the values.
  EXPECT_NEAR(tau_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.25, regularization_scalar);
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());
}

// Tests the traction calculation over an entire contact patch without
// friction butwith dissipation effecting the traction.
TEST_P(MultibodyPlantHydroelasticTractionTests,
    TractionOverPatchWithDissipation) {
  const double dissipation = 1.0;  // Units: s/m.
  const double mu_coulomb = 0.0;

  // Give the box an initial (vertical) velocity along the -z axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const double separating_velocity = -1.0;
  SetBoxTranslationalVelocity(R_WY *
      Vector3<double>(0, 0, separating_velocity));

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Ao_W, F_Bo_W;
  ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &F_Ao_W, &F_Bo_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * F_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * F_Bo_W.rotational();

  // Compute the magnitude of the normal traction. Note that the damping
  // constant at each point will be field value * dissipation coefficient.
  const double field_value = 0.5;  // in N/m².
  const double c = field_value * dissipation;  // N/m² * s/m = Ns/m³.
  const double damping_traction_magnitude = c * -separating_velocity;  // N/m².
  const double normal_traction_magnitude = field_value +
      damping_traction_magnitude;

  // Check the spatial force. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead.
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], normal_traction_magnitude, tol());

  // We expect no moment on the box.
  EXPECT_NEAR(tau_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());
}

// Friend class which provides access to HydroelasticTractionCalculator's
// private method.
class HydroelasticTractionCalculatorTester {
 public:
  template <typename T>
  static HydroelasticQuadraturePointData<T> CalcTractionAtQHelper(
      const HydroelasticTractionCalculator<T>& calculator,
      const typename HydroelasticTractionCalculator<T>::Data& data,
      geometry::SurfaceFaceIndex face_index, const T& e,
      const Vector3<T>& nhat_W, double dissipation, double mu_coulomb,
      const Vector3<T>& p_WQ) {
    return calculator.CalcTractionAtQHelper(data, face_index, e, nhat_W,
                                            dissipation, mu_coulomb, p_WQ);
  }

  template <typename T>
  static T CalcAtanXOverXFromXSquared(const T& x_squared) {
    return HydroelasticTractionCalculator<T>::CalcAtanXOverXFromXSquared(
        x_squared);
  }
};

GTEST_TEST(HydroelasticTractionCalculatorTest,
           GenerateValues) {  
  std::ofstream file("atan.dat");
  file << fmt::format("x f_std f_custom df_std df_custom\n");

  auto calc_and_print = [&file](double x_value) {
    const AutoDiffXd x(x_value, 1.0, 0);
    const AutoDiffXd x2 = x * x;
    // Custom f(x).
    const AutoDiffXd fx =
        HydroelasticTractionCalculatorTester::CalcAtanXOverXFromXSquared<
            AutoDiffXd>(x2);
    const double f_custom = fx.value();
    const double df_custom = fx.derivatives()[0];

    // "Standard" (using standard atan(x) function) f(x).
    const AutoDiffXd fx_ref = atan(x) / x;
    const double f_std = fx_ref.value();
    const double df_std = fx_ref.derivatives()[0];

    file << fmt::format("{} {} {} {} {}\n", x_value, f_std, f_custom, df_std,
                        df_custom);
  };

  // Generate vales from 1e-20 to 1e-1.  
  double x_value = 1e-20;  
  for (int e = -20; e <= -1; ++e) {
    std::cout << x_value << std::endl;
    calc_and_print(x_value);
    x_value *= 10.0;
  }

    

  // Additional values in [0.1, 1.0] and 100.0
  for (double x :
       {129.154966501488e-003, 166.810053720006e-003, 215.443469003188e-003,
        278.255940220712e-003, 359.381366380463e-003, 464.158883361278e-003,
        599.484250318941e-003, 774.263682681127e-003, 1.00000000000000e+000,
        1.0e2}) {
    std::cout << x << std::endl;
    calc_and_print(x);
  }

  file.close();
}

GTEST_TEST(HydroelasticTractionCalculatorTest,
           CalcAtanXOverXFromXSquared_AutoDiffXd) {
  using std::atan;

  // Arbitrary values where to sample atan(x)/x. We intentionally avoid zero and
  // values close to zero since our testing reference, atan(x)/x, is ill defined
  // there with large round-off errors. Round-off errors amplify further for
  // derivatives computed with AutoDiffXd since AutoDiffXd does not simplify
  // expressions to minimizer round-off errors but simply uses the chain rule,
  // leading to expressions that are suboptimal.
  //
  // Our implementation of CalcAtanXOverXFromXSquared() uses a Taylor expansion
  // for values of x < 1.0e-4, so that the function is valid even at x = 0. For
  // x >= 1.0e-4 CalcAtanXOverXFromXSquared() simply returns atan(x)/x and we
  // expect a perfect match. Therefore we choose these sample values so that
  // first three show the progression of error after we've crossed the line x =
  // 1.0e-4 and move closer to zero. The last two show where the line is and
  // that the answers exactly match thereafter.
  std::vector<double> x_samples = {
      1e-20, 1e-19, 1e-18, 1e-17, 1e-16, 1e-15, 1e-14, 1e-13, 1e-12, 
      1e-11, 1e-10, 1e-09, 1e-08, 1e-07, 1e-06, 1e-05, 1e-04, 1e-03, 1e-02, 1e-01, 1e-00, 1e+01, 1e+02};

  std::vector<double> df_ref = {
      -6.66707573717740e-021, -66.6666214687113e-021, -666.666679976207e-021,
      -6.66666666566834e-018, -66.6666666667808e-018, -666.666666666672e-018,
      -6.66666666666667e-015, -66.6666666666667e-015, -666.666666666667e-015,
      -6.66666666666667e-012, -66.6666666666667e-012, -666.666666666667e-012,
      -6.66666666666667e-009, -66.6666666666659e-009, -666.666666665867e-009,
      -6.66666666586667e-006, -66.6666658666667e-006, -666.665866667524e-006,
      -6.66586675237206e-003, 
      
      //-65.8751501063017e-003,
      -0.0658751501063017477456020868119600144,
     //-285.398163397448e-003,
      -0.285398163397448309615660845819875721,

      -13.7211777331364e-003, -155.079766000824e-006};

  (void)df_ref;

  // We notice that the expected precision in dfdx degrades for small values of
  // x, in this test for 1e-6 and 1e-5. We verified using variable precision
  // arithmetic (VPA) in Matlab that the reason is the fact that AutoDiffXd
  // generates suboptimal expressions of the derivatives. In other words, the
  // implementation in CalcAtanXOverXFromXSquared() is expected to lead to
  // values and derivatives that deviate from the "exact" values (no round-off)
  // in less than machine epsilon. However, the expressions we use here are
  // generated with AutoDiffXd, and those are NOT exact. Even more, AutoDiffXd
  // applies brute force chain rule, with simply leads to suboptimal expressions
  // that introduce round-off errors.
  //
  // As a simple example, our implementaiton of CalcAtanXOverXFromXSquared() is
  // simply a polynomial of degree four. Although our implementation uses
  // Horner's method to minimize round-off errors, AutoDiffXd applied to that
  // expression still leads to a suboptimal computation that loses precision. We
  // obtained the expected precision below using Matlab variable precision
  // arithmetic.
  std::vector<double> dfdx_expected_precision(
      x_samples.size(), std::numeric_limits<double>::epsilon());
  // We verified this is tha maximum round-off error when using AutoDiffXd to
  // compute the derivatives of CalcAtanXOverXFromXSquared(), @ x = 0.1.        
  dfdx_expected_precision[19] = 6.0e-16;  // x = 0.1

  for (size_t i = 0; i < x_samples.size(); ++i) {
    const double sample = x_samples[i];
    const AutoDiffXd x(sample, 1.0, 0);
    const AutoDiffXd x2 = x * x;
    const AutoDiffXd fx =
        HydroelasticTractionCalculatorTester::CalcAtanXOverXFromXSquared<
            AutoDiffXd>(x2);

    const AutoDiffXd fx_ref = atan(x) / x;

    // Function values should be exactly equal in double precision.
    EXPECT_NEAR(fx.value(), fx_ref.value(),
                std::numeric_limits<double>::epsilon());

    // Compare against analytical derivative.
    EXPECT_NEAR(fx.derivatives()[0], df_ref[i], dfdx_expected_precision[i]);
  }

  // We separately verify CalcAtanXOverXFromXSquared() at x = 0 since AutoDiffXd
  // of atan(x)/x is ill formed.
  const AutoDiffXd x0(0.0, 1.0, 0);
  const AutoDiffXd x0_squared = x0 * x0;
  const AutoDiffXd fx0 =
      HydroelasticTractionCalculatorTester::CalcAtanXOverXFromXSquared<
          AutoDiffXd>(x0_squared);
  EXPECT_NEAR(fx0.value(), 1.0, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(fx0.derivatives()[0], 0.0,
              std::numeric_limits<double>::epsilon());
}

GTEST_TEST(HydroelasticTractionCalculatorTest,
           CalcAtanXOverXFromXSquared_Symbolic) {
  symbolic::Variable x("x");
  const symbolic::Expression x2 = x * x;
  const symbolic::Expression fx =
      HydroelasticTractionCalculatorTester::CalcAtanXOverXFromXSquared<
          symbolic::Expression>(x2);
  const symbolic::Expression fx_expected = atan(abs(x)) / abs(x);
  EXPECT_EQ(fx, fx_expected);
}

// This is a direct test of the underlying helper function CalcTractionAtQHelper
// and how it responds to interesting cases with derivatives. Specifically,
// we need to make sure that zero relative velocity does *not* lead to
// NaN-valued derivatives.
GTEST_TEST(HydroelasticTractionCalculatorTest,
           CalcTractionAtQHelperDerivatives) {
  HydroelasticTractionCalculator<AutoDiffXd> calculator;

  RigidTransform<AutoDiffXd> X_WA(
      math::initializeAutoDiff(Vector3<double>(0, 0, 1)));
  RigidTransform<AutoDiffXd> X_WB;

  // For this test, we need just enough contact surface so that the mesh can
  // report a centroid point (part of Data constructor).
  const Vector3<AutoDiffXd> p_WC =
      (X_WA.translation() + X_WB.translation()) / 2;
  std::vector<SurfaceVertex<AutoDiffXd>> vertices;
  vertices.emplace_back(p_WC + Vector3<AutoDiffXd>(0.5, 0.5, 0));
  vertices.emplace_back(p_WC + Vector3<AutoDiffXd>(-0.5, 0, 0.5));
  vertices.emplace_back(p_WC + Vector3<AutoDiffXd>(0, -0.5, -0.5));

  std::vector<SurfaceFace> faces({SurfaceFace{
      SurfaceVertexIndex(0), SurfaceVertexIndex(1), SurfaceVertexIndex(2)}});
  auto mesh_W = std::make_unique<geometry::SurfaceMesh<AutoDiffXd>>(
      std::move(faces), std::move(vertices));
  // Note: these values are garbage. They merely allow us to instantiate the
  // field so the Data can be instantiated. The *actual* field value passed
  // to CalcTractionAtQHelper is found below.
  std::vector<AutoDiffXd> values{0, 0, 0};
  auto field = std::make_unique<
      geometry::SurfaceMeshFieldLinear<AutoDiffXd, AutoDiffXd>>(
      "junk", std::move(values), mesh_W.get(), false);
  geometry::ContactSurface<AutoDiffXd> surface(
      geometry::GeometryId::get_new_id(), geometry::GeometryId::get_new_id(),
      std::move(mesh_W), std::move(field));

  // Parameters for CalcTractionAtQHelper().
  const geometry::SurfaceFaceIndex f0(0);
  const Vector3<AutoDiffXd>& nhat_W = surface.mesh_W().face_normal(f0);
  const AutoDiffXd e = 1e5;
  const double dissipation = 0.1;
  const double mu_coulomb = 1.0;

  // Case: nothing is moving, so the relative velocity is zero.
  SpatialVelocity<AutoDiffXd> V_WA{Vector3<AutoDiffXd>::Zero(),
                                   Vector3<AutoDiffXd>::Zero()};
  SpatialVelocity<AutoDiffXd> V_WB(V_WA);
  HydroelasticTractionCalculator<AutoDiffXd>::Data data(X_WA, X_WB, V_WA, V_WB,
                                                        &surface);

  auto point_data =
      HydroelasticTractionCalculatorTester::CalcTractionAtQHelper<AutoDiffXd>(
          calculator, data, f0, e, nhat_W, dissipation, mu_coulomb, p_WC);

  // Note: we're differentiating w.r.t. p_WAo, the normal direction does *not*
  // change, so the Jacobean of nhat_W is all zeros. We'll do a quick reality
  // check to confirm this.
  //
  // This is relevant, because the traction_Aq_W is computed *using* the normal.
  // So, derivatives on nhat_W are what propagate to become derivatives on
  // traction_Aq_W. In this case, the zeros should win out. This confirms that
  // we get well-defined (non-NaN) derivatives even when relative velocity is
  // zero.
  //
  // Note: if derivatives didn't propagate down to traction_Aq_W, then the
  // call to math::autoDiffToGradientMatrix() would produce a 3x0 matrix which
  // would fail the call to CompareMatrices(). So, we know that the zero matrix
  // is the result of the chain rule, properly applied. However, to be doubly
  // sure, we'll also test the size of the derivatives() vectors.
  const Matrix3<double> zeros = Matrix3<double>::Zero();
  EXPECT_TRUE(
      CompareMatrices(math::autoDiffToGradientMatrix(nhat_W), zeros, 1e-15));
  EXPECT_EQ(point_data.traction_Aq_W.x().derivatives().size(), 3);
  EXPECT_EQ(point_data.traction_Aq_W.y().derivatives().size(), 3);
  EXPECT_EQ(point_data.traction_Aq_W.z().derivatives().size(), 3);
  const Matrix3<double> grad_traction_Aq_W =
      math::autoDiffToGradientMatrix(point_data.traction_Aq_W);
  EXPECT_TRUE(CompareMatrices(grad_traction_Aq_W, zeros));
}

// This fixture defines a contacting configuration between a box and a
// half-space in a local frame, Y. See MultibodyPlantHydroelasticTractionTests
// class documentation for a description of Frame Y.
class HydroelasticReportingTests
    : public ::testing::TestWithParam<RigidTransform<double>> {
 public:
  const ContactSurface<double>& contact_surface() const {
      return *contact_surface_;
  }

  // Returns the default numerical tolerance.
  double tol() const { return tol_; }

  // Gets the expected pressure (in Pa) at Point Q in Frame Y. To get some
  // interesting values for testing, we define the pressure at Point Q using
  // a plane with normal in the direction [-1, -1, -1] that passes through the
  // origin.
  double pressure(const Vector3<double>& P_YQ) const {
    return pressure_field_normal().dot(P_YQ);
  }

  // Gets the normal to the pressure field in Frame Y.
  Vector3<double> pressure_field_normal() const {
    return Vector3<double>(-1, -1, -1).normalized();
  }

  const HydroelasticTractionCalculator<double>::Data& calculator_data() {
    return *calculator_data_;
  }

 private:
  void SetUp() override {
    // Set the poses in the Y frame. Identity poses should be fine for our
    // purposes because none of the reporting code itself relies upon these (the
    // code that does rely upon these, e.g., traction calculation, slip velocity
    // computation) is tested elsewhere in this file).
    const RigidTransform<double>& X_WY = GetParam();
    const RigidTransform<double> X_YA = RigidTransform<double>::Identity();
    const RigidTransform<double> X_YB = RigidTransform<double>::Identity();

    // Note: this test does not require valid GeometryIds.
    const GeometryId null_id;

    // Create the surface mesh first.
    auto mesh = CreateSurfaceMesh();

    // Create the e field values (i.e., "hydroelastic pressure").
    std::vector<double> e_MN(mesh->num_vertices());
    for (SurfaceVertexIndex i(0); i < mesh->num_vertices(); ++i)
      e_MN[i] = pressure(mesh->vertex(i).r_MV());

    SurfaceMesh<double>* mesh_pointer = mesh.get();
    contact_surface_ = std::make_unique<ContactSurface<double>>(
      null_id, null_id, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e_MN", std::move(e_MN), mesh_pointer));

    // Set the velocities to correspond to one body fixed and one body
    // free so that we can test the slip velocity. Additionally, we'll
    // set the angular velocity to correspond to one of the bodies (A) spinning
    // along the normal to the contact surface. That will let us verify that the
    // velocity at the centroid of the contact surface is zero.
    const double spin_rate = 1.0;  // m/s.
    const SpatialVelocity<double> V_YA(
        Vector3<double>(0, 0, spin_rate), Vector3<double>::Zero());
    const SpatialVelocity<double> V_YB(
        Vector3<double>::Zero(), Vector3<double>::Zero());

    // Convert all quantities to the world frame.
    const RigidTransform<double> X_WA = X_WY * X_YA;
    const RigidTransform<double> X_WB = X_WY * X_YB;
    const SpatialVelocity<double> V_WA = X_WY.rotation() * V_YA;
    const SpatialVelocity<double> V_WB = X_WY.rotation() * V_YB;

    // Set the calculator data.
    calculator_data_ = std::make_unique<
        HydroelasticTractionCalculator<double>::Data>(
            X_WA, X_WB, V_WA, V_WB, contact_surface_.get());
  }

  const double tol_{10 * std::numeric_limits<double>::epsilon()};
  std::unique_ptr<ContactSurface<double>> contact_surface_;
  std::unique_ptr<HydroelasticTractionCalculator<double>::Data>
      calculator_data_;
};

// These transformations, denoted X_WY, are passed as parameters to the tests
// to allow changing the absolute (but not relative) poses of the two bodies.
const RigidTransform<double> poses[] = {
    RigidTransform<double>::Identity(),
    RigidTransform<double>(
        math::RotationMatrix<double>::MakeYRotation(M_PI_4),
        drake::Vector3<double>(1, 2, 3))
};

INSTANTIATE_TEST_SUITE_P(PoseInstantiations,
                        MultibodyPlantHydroelasticTractionTests,
                        ::testing::ValuesIn(poses));

// TODO(edrumwri) Break the tests below out into a separate file.

// Returns a distinct spatial force.
SpatialForce<double> MakeSpatialForce() {
  return SpatialForce<double>(Vector3<double>(1, 2, 3),
                              Vector3<double>(4, 5, 6));
}

// Returns a distinct vector (containing a single element) of quadrature point
// data.
std::vector<HydroelasticQuadraturePointData<double>> GetQuadraturePointData() {
  HydroelasticQuadraturePointData<double> data;
  data.p_WQ = Vector3<double>(3.0, 5.0, 7.0);
  data.face_index = SurfaceFaceIndex(1);
  data.vt_BqAq_W = Vector3<double>(11.0, 13.0, 17.0);
  data.traction_Aq_W = Vector3<double>(19.0, 23.0, 29.0);
  return { data };
}

HydroelasticContactInfo<double> CreateContactInfo(
    std::unique_ptr<ContactSurface<double>>* contact_surface,
    std::unique_ptr<HydroelasticContactInfo<double>>* contact_info) {
  // Create the contact surface using a duplicated arbitrary ID and identity
  // pose; pose and geometry IDs are irrelevant for this test.
  GeometryId arbitrary_id = GeometryId::get_new_id();
  *contact_surface = CreateContactSurface(arbitrary_id, arbitrary_id,
          RigidTransform<double>::Identity());

  // Create the HydroelasticContactInfo using particular spatial force and
  // quadrature point data.
  std::vector<HydroelasticQuadraturePointData<double>>
      quadrature_point_data = GetQuadraturePointData();
  return HydroelasticContactInfo<double>(contact_surface->get(),
                                         MakeSpatialForce(),
                                         std::move(quadrature_point_data));
}

// Verifies that the HydroelasticContactInfo structure uses the raw pointer
// and the unique pointer, as appropriate, on copy construction.
GTEST_TEST(HydroelasticContactInfo, CopyConstruction) {
  std::unique_ptr<ContactSurface<double>> contact_surface;
  std::unique_ptr<HydroelasticContactInfo<double>> contact_info;
  HydroelasticContactInfo<double> copy =
      CreateContactInfo(&contact_surface, &contact_info);

  // Verify that copy construction used the raw pointer.
  EXPECT_EQ(contact_surface.get(), &copy.contact_surface());

  // Copy it again and make sure that the surface is new.
  HydroelasticContactInfo<double> copy2 = copy;
  EXPECT_NE(contact_surface.get(), &copy2.contact_surface());

  // Verify that the spatial force was copied.
  EXPECT_EQ(copy.F_Ac_W().translational(), MakeSpatialForce().translational());
  EXPECT_EQ(copy.F_Ac_W().rotational(), MakeSpatialForce().rotational());

  // Verify that the quadrature point data was copied.
  EXPECT_EQ(copy.quadrature_point_data(), GetQuadraturePointData());
}

// Verifies that the HydroelasticContactInfo structure transfers ownership of
// the ContactSurface.
GTEST_TEST(HydroelasticContactInfo, MoveConstruction) {
  std::unique_ptr<ContactSurface<double>> contact_surface;
  std::unique_ptr<HydroelasticContactInfo<double>> contact_info;
  HydroelasticContactInfo<double> copy =
      CreateContactInfo(&contact_surface, &contact_info);
  HydroelasticContactInfo<double> moved_copy = std::move(copy);

  // Verify that the move construction retained the raw pointer.
  EXPECT_EQ(contact_surface.get(), &moved_copy.contact_surface());

  // Verify that the spatial force was copied.
  EXPECT_EQ(moved_copy.F_Ac_W().translational(),
            MakeSpatialForce().translational());
  EXPECT_EQ(moved_copy.F_Ac_W().rotational(), MakeSpatialForce().rotational());

  // Verify that the quadrature point data was copied.
  EXPECT_EQ(moved_copy.quadrature_point_data(), GetQuadraturePointData());
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

