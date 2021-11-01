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

class CompliantContactManagerTest : public ::testing::Test {
 protected:
  // Model parameters.
  struct ContactParameters {
    // Compliance:
    //   - For point contact, the linear spring constant in N/m.
    //   - For hydroelastic contact, the hydroelastic modulus in N/m².
    double compliance;
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

  void SetUp() override {
    MakeModel(default_ground_parameters_, default_sphere_parameters_);
  }

  void MakeModel(const ContactParameters& ground_params,
                 const SphereParameters& sphere1_params) {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder, time_step_);

    // Add model of the ground.
    {
      const ProximityProperties ground_properties =
          MakePointContactProximityProperties(ground_params);
      plant_->RegisterCollisionGeometry(plant_->world_body(), RigidTransformd(),
                                        geometry::HalfSpace(),
                                        "ground_collision", ground_properties);
      const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
      plant_->RegisterVisualGeometry(plant_->world_body(), RigidTransformd(),
                                     geometry::HalfSpace(), "ground_visual",
                                     green);
    }

    sphere1_ = &AddSphere(sphere1_params);

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

    SetContactState();
  }

  void SetContactState() const {
    const double sphere1_com_z =
        default_sphere_parameters_.radius - penetration_distance_;
    const RigidTransformd X_WB1(Vector3d(0, 0, sphere1_com_z));
    plant_->SetFreeBodyPose(plant_context_, *sphere1_, X_WB1);
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
        MakePointContactProximityProperties(params.contact_parameters);
    sphere1_geometry_ = plant_->RegisterCollisionGeometry(
        body, RigidTransformd(), shape, params.name + "_collision", properties);

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

  const std::vector<DiscreteContactPair<double>>& EvalDiscreteContactPairs(
      const Context<double>& context) const {
    return contact_manager_->EvalDiscreteContactPairs(context);
  }

  static ProximityProperties MakePointContactProximityProperties(
      const ContactParameters& params) {
    ProximityProperties properties;
    properties.AddProperty(geometry::internal::kMaterialGroup,
                           geometry::internal::kPointStiffness,
                           params.compliance);
    properties.AddProperty(geometry::internal::kMaterialGroup,
                           "dissipation_time_constant",
                           params.dissipation_time_constant);
    properties.AddProperty(
        geometry::internal::kMaterialGroup, geometry::internal::kFriction,
        CoulombFriction<double>(params.friction_coefficient,
                                params.friction_coefficient));
    return properties;
  }

  void VerifyDiscreteContactPairs(
      const ContactParameters& ground_contact_params,
      const ContactParameters& sphere_contact_params) {
    SphereParameters sphere_params = default_sphere_parameters_;
    sphere_params.contact_parameters = sphere_contact_params;

    // Soft sphere/hard ground.
    MakeModel(ground_contact_params, sphere_params);

    const std::vector<DiscreteContactPair<double>>& pairs =
        EvalDiscreteContactPairs(*plant_context_);
    EXPECT_EQ(pairs.size(), 1u);

    constexpr double kTolerance = 1.0e-14;

    const int sign = pairs[0].id_A == sphere1_geometry_ ? 1 : -1;
    const Vector3d normal_expected = sign * Vector3d::UnitZ();
    EXPECT_TRUE(CompareMatrices(pairs[0].nhat_BA_W, normal_expected));

    const double phi_expected = -penetration_distance_;
    EXPECT_NEAR(pairs[0].phi0, phi_expected, kTolerance);

    const double k1 = ground_contact_params.compliance;
    const double k2 = sphere_contact_params.compliance;
    const double stiffness_expected = (k1 * k2) / (k1 + k2);
    EXPECT_NEAR(pairs[0].stiffness, stiffness_expected,
                kTolerance * stiffness_expected);

    const double tau1 = ground_contact_params.dissipation_time_constant;
    const double tau2 = sphere_contact_params.dissipation_time_constant;
    const double dissipation_expected = stiffness_expected * (tau1 + tau2);
    EXPECT_NEAR(pairs[0].damping, dissipation_expected,
                kTolerance * dissipation_expected);

    const double pz_WC = -k2 / (k1 + k2) * penetration_distance_;
    EXPECT_NEAR(pairs[0].p_WC.z(), pz_WC, kTolerance);
    EXPECT_TRUE(std::isnan(pairs[0].fn0));  // Expect NaN since not used.
  }

  double time_step_{0.001};
  ContactParameters default_ground_parameters_{1.0e20, 0.0, 1.0};
  SphereParameters default_sphere_parameters_{
      "box1",
      {51 / 255, 1.0, 1.0, 1.0} /* cyan */,
      10.0 /* mass */,
      0.2 /* size */,
      {1.0e5 /* stiffness */, 0.001 /* dissipation */, 0.5 /* friction */}};
  const double penetration_distance_{1.0e-3};

  std::unique_ptr<systems::Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{nullptr};
  SceneGraph<double>* scene_graph_{nullptr};
  const RigidBody<double>* sphere1_{nullptr};
  GeometryId sphere1_geometry_;
  CompliantContactManager<double>* contact_manager_{nullptr};
  std::unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{nullptr};
};

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

TEST_F(CompliantContactManagerTest, EvalDiscreteContactPairs) {
  ContactParameters soft_contact{1.0e3, 0.01, 1.0};
  ContactParameters hard_contact{1.0e40, 0.0, 1.0};

  // Hard ground/soft sphere.
  VerifyDiscreteContactPairs(hard_contact, soft_contact);

  // Equally soft ground and sphere.
  VerifyDiscreteContactPairs(soft_contact, soft_contact);

  // Soft ground/hard sphere.
  VerifyDiscreteContactPairs(soft_contact, hard_contact);
}

}  // namespace multibody
}  // namespace drake
