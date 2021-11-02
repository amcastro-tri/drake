#include "drake/multibody/plant/compliant_contact_manager.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/pgs_solver.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

using drake::geometry::GeometryId;
using drake::geometry::PenetrationAsPointPair;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::SurfaceMesh;
using drake::geometry::VolumeMesh;
using drake::geometry::VolumeMeshFieldLinear;
using drake::math::RigidTransformd;
using drake::multibody::contact_solvers::internal::BlockSparseMatrix;
using drake::multibody::contact_solvers::internal::PgsSolver;
using drake::multibody::internal::DiscreteContactPair;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace internal {

class CompliantContactManagerTest : public ::testing::Test {
 protected:
  // Model parameters.
  struct ContactParameters {
    std::optional<double> point_stiffness;
    std::optional<double> hydro_modulus;
    double dissipation_time_constant;
    double friction_coefficient;
  };

  struct SphereParameters {
    const std::string name;
    const Vector4<double> color;
    double mass;
    double radius;
    ContactParameters contact_parameters;
  };

  void MakeDefaultSetup() {
    const ContactParameters soft_contact{1.0e5, 1.0e5, 0.01, 1.0};
    const ContactParameters hard_hydro_contact{
        std::nullopt, std::numeric_limits<double>::infinity(), 0.0, 1.0};
    const SphereParameters sphere1_params{"Sphere1",
                                          {0.0, 0.0, 1.0, 1.0} /* blue */,
                                          10.0 /* mass */,
                                          0.2 /* size */,
                                          soft_contact};
    const SphereParameters sphere2_params{"Sphere2",
                                          {1.0, 0.0, 0.0, 1.0} /* red */,
                                          10.0 /* mass */,
                                          0.2 /* size */,
                                          soft_contact};

    // Soft sphere/hard ground.
    MakeModel(hard_hydro_contact, sphere1_params, sphere2_params);
  }

  void MakeModel(const ContactParameters& ground_params,
                 const SphereParameters& sphere1_params,
                 const SphereParameters& sphere2_params) {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder, time_step_);

    // Add model of the ground.
    const ProximityProperties ground_properties =
        MakeProximityProperties(ground_params);
    plant_->RegisterCollisionGeometry(plant_->world_body(), RigidTransformd(),
                                      geometry::HalfSpace(), "ground_collision",
                                      ground_properties);
    const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
    plant_->RegisterVisualGeometry(plant_->world_body(), RigidTransformd(),
                                   geometry::HalfSpace(), "ground_visual",
                                   green);

    // Add models of the spheres.
    sphere1_ = &AddSphere(sphere1_params);
    sphere2_ = &AddSphere(sphere2_params);

    plant_->set_contact_model(
        drake::multibody::ContactModel::kHydroelasticWithFallback);

    plant_->Finalize();
    auto owned_contact_manager =
        std::make_unique<CompliantContactManager<double>>(
            std::make_unique<PgsSolver<double>>());
    contact_manager_ = owned_contact_manager.get();
    plant_->SetDiscreteUpdateManager(std::move(owned_contact_manager));

    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());

    SetContactState(sphere1_params, sphere2_params);
  }

  void SetContactState(
      const SphereParameters& sphere1_params,
      const std::optional<SphereParameters>& sphere2_params) const {
    const double sphere1_com_z = sphere1_params.radius - penetration_distance_;
    const RigidTransformd X_WB1(Vector3d(0, 0, sphere1_com_z));
    plant_->SetFreeBodyPose(plant_context_, *sphere1_, X_WB1);
    if (sphere2_params) {
      const double sphere2_com_z = 2.0 * sphere1_params.radius +
                                   sphere2_params->radius -
                                   2.0 * penetration_distance_;
      const RigidTransformd X_WB2(Vector3d(0, 0, sphere2_com_z));
      plant_->SetFreeBodyPose(plant_context_, *sphere2_, X_WB2);
    }
  }

  const RigidBody<double>& AddSphere(const SphereParameters& params) {
    // Add rigid body.
    const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
    const UnitInertia<double> G_BBcm_B =
        UnitInertia<double>::SolidSphere(params.radius);
    const SpatialInertia<double> M_BBcm_B(params.mass, p_BoBcm_B, G_BBcm_B);
    const RigidBody<double>& body = plant_->AddRigidBody(params.name, M_BBcm_B);

    // Add collision geometry.
    const geometry::Sphere shape(params.radius);
    const ProximityProperties properties =
        MakeProximityProperties(params.contact_parameters);
    plant_->RegisterCollisionGeometry(body, RigidTransformd(), shape,
                                      params.name + "_collision", properties);

    // Add visual geometry.
    plant_->RegisterVisualGeometry(body, RigidTransformd(), shape,
                                   params.name + "_visual", params.color);

    return body;
  }

  // N.B. This method assumes a single geometry per body.
  double GetPointContactStiffness(const RigidBody<double>& body) {
    const geometry::QueryObject<double>& query_object =
        plant_->get_geometry_query_input_port()
            .Eval<geometry::QueryObject<double>>(*plant_context_);
    const geometry::SceneGraphInspector<double>& inspector =
        query_object.inspector();
    const std::vector<geometry::GeometryId>& geometries =
        plant_->GetCollisionGeometriesForBody(body);
    DRAKE_DEMAND(geometries.size() == 1u);
    return contact_manager_->GetPointContactStiffness(geometries[0], inspector);
  }

  // N.B. This method assumes a single geometry per body.
  double GetDissipationTimeConstant(const RigidBody<double>& body) {
    const geometry::QueryObject<double>& query_object =
        plant_->get_geometry_query_input_port()
            .Eval<geometry::QueryObject<double>>(*plant_context_);
    const geometry::SceneGraphInspector<double>& inspector =
        query_object.inspector();
    const std::vector<geometry::GeometryId>& geometries =
        plant_->GetCollisionGeometriesForBody(body);
    DRAKE_DEMAND(geometries.size() == 1u);
    return contact_manager_->GetDissipationTimeConstant(geometries[0],
                                                        inspector);
  }

  const std::vector<PenetrationAsPointPair<double>>& EvalPointPairPenetrations(
      const Context<double>& context) const {
    return contact_manager_->EvalPointPairPenetrations(context);
  }

  const std::vector<geometry::ContactSurface<double>>& EvalContactSurfaces(
      const Context<double>& context) const {
    return contact_manager_->EvalContactSurfaces(context);
  }

  const std::vector<DiscreteContactPair<double>>& EvalDiscreteContactPairs(
      const Context<double>& context) const {
    return contact_manager_->EvalDiscreteContactPairs(context);
  }

  const internal::ContactJacobianCache<double>& EvalContactJacobianCache(
      const systems::Context<double>& context) const {
    return contact_manager_->EvalContactJacobianCache(context);
  }

  static ProximityProperties MakeProximityProperties(
      const ContactParameters& params) {
    DRAKE_DEMAND(params.point_stiffness || params.hydro_modulus);
    ProximityProperties properties;
    if (params.point_stiffness) {
      properties.AddProperty(geometry::internal::kMaterialGroup,
                             geometry::internal::kPointStiffness,
                             *params.point_stiffness);
    }

    if (params.hydro_modulus) {
      if (params.hydro_modulus == std::numeric_limits<double>::infinity()) {
        properties.AddProperty(geometry::internal::kHydroGroup,
                               geometry::internal::kComplianceType,
                               geometry::internal::HydroelasticType::kRigid);
      } else {
        properties.AddProperty(geometry::internal::kHydroGroup,
                               geometry::internal::kComplianceType,
                               geometry::internal::HydroelasticType::kSoft);
        properties.AddProperty(geometry::internal::kMaterialGroup,
                               geometry::internal::kElastic,
                               *params.hydro_modulus);
      }
      // N.B. Add the slab thickness property by default so that we can model a
      // half space (either compliant or rigid).
      properties.AddProperty(geometry::internal::kHydroGroup,
                             geometry::internal::kSlabThickness, 1.0);
      properties.AddProperty(geometry::internal::kHydroGroup,
                             geometry::internal::kRezHint, 1.0);
    }

    properties.AddProperty(geometry::internal::kMaterialGroup,
                           "dissipation_time_constant",
                           params.dissipation_time_constant);
    properties.AddProperty(
        geometry::internal::kMaterialGroup, geometry::internal::kFriction,
        CoulombFriction<double>(params.friction_coefficient,
                                params.friction_coefficient));
    return properties;
  }

  void VerifyDiscreteContactPairsFromPointContact(
      const ContactParameters& sphere1_point_params,
      const ContactParameters& sphere2_point_params) {
    // This test is specific to point contact. Both spheres must have point
    // contact properties.
    DRAKE_DEMAND(sphere1_point_params.point_stiffness.has_value());
    DRAKE_DEMAND(sphere2_point_params.point_stiffness.has_value());

    ContactParameters sphere1_contact_params = sphere1_point_params;
    sphere1_contact_params.hydro_modulus = 1.0e5;
    const ContactParameters sphere2_contact_params = sphere2_point_params;

    const ContactParameters hard_hydro_contact{
        std::nullopt, std::numeric_limits<double>::infinity(), 0.0, 1.0};
    const SphereParameters sphere1_params{"Sphere1",
                                          {0.0, 0.0, 1.0, 1.0} /* blue */,
                                          10.0 /* mass */,
                                          0.2 /* size */,
                                          sphere1_contact_params};
    const SphereParameters sphere2_params{"Sphere2",
                                          {1.0, 0.0, 0.0, 1.0} /* red */,
                                          10.0 /* mass */,
                                          0.2 /* size */,
                                          sphere2_contact_params};

    // Soft sphere/hard ground.
    MakeModel(hard_hydro_contact, sphere1_params, sphere2_params);

    const std::vector<PenetrationAsPointPair<double>>& point_pairs =
        EvalPointPairPenetrations(*plant_context_);
    const int num_point_pairs = point_pairs.size();
    const std::vector<geometry::ContactSurface<double>>& surfaces =
        EvalContactSurfaces(*plant_context_);
    ASSERT_EQ(surfaces.size(), 1u);
    const int num_hydro_pairs = surfaces[0].mesh_W().num_faces();
    const std::vector<DiscreteContactPair<double>>& pairs =
        EvalDiscreteContactPairs(*plant_context_);
    EXPECT_EQ(pairs.size(), num_point_pairs + num_hydro_pairs);

    constexpr double kTolerance = 1.0e-14;

    const GeometryId sphere2_geometry =
        plant_->GetCollisionGeometriesForBody(*sphere2_)[0];

    const int sign = pairs[0].id_A == sphere2_geometry ? 1 : -1;
    const Vector3d normal_expected = sign * Vector3d::UnitZ();
    EXPECT_TRUE(CompareMatrices(pairs[0].nhat_BA_W, normal_expected));

    const double phi_expected = -penetration_distance_;
    EXPECT_NEAR(pairs[0].phi0, phi_expected, kTolerance);

    const double k1 = *sphere1_contact_params.point_stiffness;
    const double k2 = *sphere2_contact_params.point_stiffness;
    const double stiffness_expected = (k1 * k2) / (k1 + k2);
    EXPECT_NEAR(pairs[0].stiffness, stiffness_expected,
                kTolerance * stiffness_expected);

    const double tau1 = sphere1_contact_params.dissipation_time_constant;
    const double tau2 = sphere2_contact_params.dissipation_time_constant;
    const double dissipation_expected = stiffness_expected * (tau1 + tau2);
    EXPECT_NEAR(pairs[0].damping, dissipation_expected,
                kTolerance * dissipation_expected);

    const double pz_WS1 =
        plant_->GetFreeBodyPose(*plant_context_, *sphere1_).translation().z();
    const double pz_WC = -k2 / (k1 + k2) * penetration_distance_ + pz_WS1 +
                         sphere1_params.radius;
    EXPECT_NEAR(pairs[0].p_WC.z(), pz_WC, kTolerance);
    EXPECT_TRUE(std::isnan(pairs[0].fn0));  // Expect NaN since not used.
  }

  double time_step_{0.001};
  const double penetration_distance_{1.0e-3};

  std::unique_ptr<systems::Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const RigidBody<double>* sphere1_{nullptr};
  const RigidBody<double>* sphere2_{nullptr};
  CompliantContactManager<double>* contact_manager_{nullptr};
  std::unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{nullptr};
};

#if 0
TEST_F(CompliantContactManagerTest, PointContactProperties) {
  auto verify_point_contact_parameters =
      [this](const RigidBody<double>& body,
             const ContactParameters& parameters) {
        EXPECT_EQ(GetPointContactStiffness(body), parameters.compliance);
        EXPECT_EQ(GetDissipationTimeConstant(body),
                  parameters.dissipation_time_constant);
      };
  verify_point_contact_parameters(plant_->world_body(),
                                  default_ground_parameters_);
  verify_point_contact_parameters(
      *sphere1_, default_sphere_parameters_.contact_parameters);
}
#endif

TEST_F(CompliantContactManagerTest,
       VerifyDiscreteContactPairsFromPointContact) {
  ContactParameters soft_point_contact{1.0e3, std::nullopt, 0.01, 1.0};
  ContactParameters hard_point_contact{1.0e40, std::nullopt, 0.0, 1.0};

  // Hard ground/soft sphere.
  VerifyDiscreteContactPairsFromPointContact(hard_point_contact,
                                             soft_point_contact);

  // Equally soft ground and sphere.
  VerifyDiscreteContactPairsFromPointContact(soft_point_contact,
                                             soft_point_contact);

  // Soft ground/hard sphere.
  VerifyDiscreteContactPairsFromPointContact(soft_point_contact,
                                             hard_point_contact);
}

TEST_F(CompliantContactManagerTest,
       VerifyDiscreteContactPairsFromHydroelasticContact) {
  const ContactParameters soft_contact{1.0e5, 1.0e5, 0.01, 1.0};
  const ContactParameters hard_hydro_contact{
      std::nullopt, std::numeric_limits<double>::infinity(), 0.0, 1.0};
  const SphereParameters sphere1_params{"Sphere1",
                                        {0.0, 0.0, 1.0, 1.0} /* blue */,
                                        10.0 /* mass */,
                                        0.2 /* size */,
                                        soft_contact};
  const SphereParameters sphere2_params{"Sphere2",
                                        {1.0, 0.0, 0.0, 1.0} /* red */,
                                        10.0 /* mass */,
                                        0.2 /* size */,
                                        soft_contact};

  // Soft sphere/hard ground.
  MakeModel(hard_hydro_contact, sphere1_params, sphere2_params);

  const std::vector<PenetrationAsPointPair<double>>& point_pairs =
      EvalPointPairPenetrations(*plant_context_);
  const int num_point_pairs = point_pairs.size();
  EXPECT_EQ(num_point_pairs, 1);
  const std::vector<DiscreteContactPair<double>>& pairs =
      EvalDiscreteContactPairs(*plant_context_);

  const std::vector<geometry::ContactSurface<double>>& surfaces =
      EvalContactSurfaces(*plant_context_);
  ASSERT_EQ(surfaces.size(), 1u);
  const geometry::SurfaceMesh<double>& patch = surfaces[0].mesh_W();
  EXPECT_EQ(pairs.size(), patch.num_faces() + num_point_pairs);
  PRINT_VAR(pairs.size());
}

TEST_F(CompliantContactManagerTest, EvalContactJacobianCache) {
  MakeDefaultSetup();

  const std::vector<DiscreteContactPair<double>>& pairs =
      EvalDiscreteContactPairs(*plant_context_);
  const auto& cache = EvalContactJacobianCache(*plant_context_);
  const auto& Jc = cache.Jc;
  EXPECT_EQ(Jc.cols(), plant_->num_velocities());
  EXPECT_EQ(Jc.rows(), 3 * pairs.size());

  // For this model we know the first entry corresponds to the single point pair
  // between sphere 1 and sphere 2.

  // Test that Jc*v gives v_WB1 and v_B1B2_W
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
