#include "drake/multibody/plant/compliant_contact_manager.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/pgs_solver.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

using drake::geometry::GeometryId;
using drake::geometry::PenetrationAsPointPair;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::SurfaceMesh;
using drake::geometry::VolumeMesh;
using drake::geometry::VolumeMeshFieldLinear;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
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

// In this fixture we set a simple model consisting of:
//  1. The flat ground.
//  2. A sphere (sphere 1) on top of the ground.
//  3. A second sphere (sphere 2) on top of the first sphere.
// The flat ground is modeled as rigid-hydroelastic.
// Sphere 1 interacts with the ground using the hydroelastic contact model
// Sphere 2 interacts with sphere 1 using the point contact model.
// We use this fixture to verify the contact quantities computed by the
// CompliantContactManager.
class CompliantContactManagerTest : public ::testing::Test {
 public:
  // Contact model parameters.
  struct ContactParameters {
    // Point contact stiffness. If nullopt, this property is not added to the
    // model.
    std::optional<double> point_stiffness;
    // Hydroelastic modulus. If nullopt, this property is not added to the
    // model.
    std::optional<double> hydro_modulus;
    // Dissipation time constant τ is used to setup the linear dissipation model
    // where dissipation is c = τ⋅k, with k the point pair stiffness.
    double dissipation_time_constant;
    // Coefficient of dynamic friction.
    double friction_coefficient;
  };

  // Parameters used to setup the model of a compliant sphere.
  struct SphereParameters {
    const std::string name;
    const Vector4<double> color;
    double mass;
    double radius;
    ContactParameters contact_parameters;
  };

  // The default setup for this fixture corresponds to:
  //   - A rigid-hidroelastic half-space for the ground.
  //   - A compliant-hidroelastic sphere on top of the ground.
  //   - A second compliant-hidroelastic sphere on top of the first sphere.
  //   - Both spheres also have point contact compliance.
  //   - We set MultibodyPlant to use hydroelastic contact with fallback.
  //   - Sphere 1 penetrates into the ground penetration_distance_.
  //   - Sphere 1 and 2 penetrate penetration_distance_.
  //   - Velocities are zero.
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

  // Sphere 1 is set on top of the ground and sphere 2 sits right on top of
  // sphere 1. We set the state of the model so that sphere 1 penetrates into
  // the ground a distance penetration_distance_ and so that sphere 1 and 2 also
  // interpenetrate a distance penetration_distance_.
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

  // Utility to make ProximityProperties from ContactParameters.
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

  // This method makes a model with the specified sphere 1 and sphere 2
  // properties and verifies the resulting contact pairs.
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

  // In the methods below we use CompliantContactManagerTest's friendship with
  // CompliantContactManager to provide access to private methods for unit
  // testing.

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

 protected:
  // Arbitrary positive value so that the model is discrete.
  double time_step_{0.001};

  // Default penetration distance. The configuration of the model is set so that
  // ground/sphere1 and sphere1/sphere2 interpenetrate by this amount.
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

// Unit test to verify discrete contact pairs computed by the manger for
// different combinations of compliance.
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

// Unit test to verify discrete contact pairs computed by the manger for
// hydroelastic contact.
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
}

// Unit test to verify the computation of the contact Jacobian.
TEST_F(CompliantContactManagerTest, EvalContactJacobianCache) {
  MakeDefaultSetup();
  const double radius = 0.2;  // Spheres's radii in the default setup.
  const double kTolerance = std::numeric_limits<double>::epsilon();

  const std::vector<DiscreteContactPair<double>>& pairs =
      EvalDiscreteContactPairs(*plant_context_);
  const auto& cache = EvalContactJacobianCache(*plant_context_);
  const auto& Jc = cache.Jc;
  EXPECT_EQ(Jc.cols(), plant_->num_velocities());
  EXPECT_EQ(Jc.rows(), 3 * pairs.size());

  // Arbitrary velocity of sphere 1.
  const Vector3d v_WS1(1, 2, 3);
  const Vector3d w_WS1(4, 5, 6);
  const SpatialVelocity<double> V_WS1(w_WS1, v_WS1);

  // Arbitrary velocity of sphere 2.
  const Vector3d v_WS2(7, 8, 9);
  const Vector3d w_WS2(10, 11, 12);
  const SpatialVelocity<double> V_WS2(w_WS2, v_WS2);

  plant_->SetFreeBodySpatialVelocity(plant_context_, *sphere1_, V_WS1);
  plant_->SetFreeBodySpatialVelocity(plant_context_, *sphere2_, V_WS2);
  const VectorXd v = plant_->GetVelocities(*plant_context_);

  const GeometryId sphere1_geometry =
      plant_->GetCollisionGeometriesForBody(*sphere1_)[0];

  // Verify contact Jacobian for the point pair.
  // For this model we know the first entry corresponds to the single point pair
  // between sphere 1 and sphere 2.
  {
    // For the default setup both spheres are equally compliant and therefore
    // the contact point C lies right in the middle.
    const Vector3d p_S1C_W(0, 0, radius - penetration_distance_ / 2.0);
    const Vector3d p_S2C_W(0, 0, -(radius - penetration_distance_ / 2.0));

    // Compute expected contact point velocity.
    const Vector3d v_WS1c = V_WS1.Shift(p_S1C_W).translational();
    const Vector3d v_WS2c = V_WS2.Shift(p_S2C_W).translational();
    const Vector3d expected_v_S1cS2c_W = v_WS2c - v_WS1c;

    const int sign = pairs[0].id_A == sphere1_geometry ? 1 : -1;
    const MatrixXd J_S1cS2c_C = sign * Jc.topRows(3);
    const RotationMatrixd& R_WC = cache.R_WC_list[0];
    const MatrixXd J_S1cS2c_W = R_WC.matrix() * J_S1cS2c_C;
    const Vector3d v_S1cS2c_W = J_S1cS2c_W * v;
    EXPECT_TRUE(CompareMatrices(v_S1cS2c_W, expected_v_S1cS2c_W, kTolerance,
                                MatrixCompareType::relative));
  }

  // Verify contact Jacobian for hydroelastic pairs.
  // We know hydroelastic pairs come after point pairs.
  {
    const Vector3d p_WS1(0, 0, radius - penetration_distance_);
    for (size_t q = 1; q < pairs.size(); ++q) {
      const Vector3d& p_WC = pairs[q].p_WC;
      const Vector3d p_S1C_W = p_WC - p_WS1;
      const Vector3d expected_v_WS1c = V_WS1.Shift(p_S1C_W).translational();
      const int sign = pairs[q].id_B == sphere1_geometry ? 1 : -1;
      const MatrixXd J_WS1c_C =
          sign * Jc.block(3 * q, 0, 3, plant_->num_velocities());
      const RotationMatrixd& R_WC = cache.R_WC_list[q];
      const MatrixXd J_WS1c_W = R_WC.matrix() * J_WS1c_C;
      const Vector3d v_WS1c_W = J_WS1c_W * v;
      EXPECT_TRUE(CompareMatrices(v_WS1c_W, expected_v_WS1c, kTolerance,
                                  MatrixCompareType::relative));
    }
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
