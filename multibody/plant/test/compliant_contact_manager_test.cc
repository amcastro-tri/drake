#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/pgs_solver.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

#include <iostream>
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
  struct PointContactParameters {
    double stiffness;
    double dissipation_time_constant;
    double friction_coefficient;
  };

  struct BoxParameters {
    const std::string name;
    const Vector4<double> color;
    double mass;
    double size;
    PointContactParameters contact_parameters;
  };

  /* Sets up a model containing a rigid hydroelastics box on top of a compliant
  hydroelastics ground. A second rigid box is placed on top of the first box.
  Compliant point contact is used between the two boxes.
  We the set a CompliantContactManager as the discrete update manager for the
  MultibodyPlant. */
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder, time_step_);

    // Add model of the ground.
    {
      const ProximityProperties ground_properties =
          MakePointContactProximityProperties(ground_contact_parameters_);
      plant_->RegisterCollisionGeometry(plant_->world_body(), RigidTransformd(),
                                        geometry::HalfSpace(),
                                        "ground_collision", ground_properties);
      const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
      plant_->RegisterVisualGeometry(plant_->world_body(), RigidTransformd(),
                                     geometry::HalfSpace(), "ground_visual",
                                     green);
    }

    box1_ = &AddBox(box1_parameters_);

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
  }

  const RigidBody<double>& AddBox(const BoxParameters& params) {
    // Add rigid body.
    const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
    const UnitInertia<double> G_BBcm_B =
        UnitInertia<double>::SolidBox(params.size, params.size, params.size);
    const SpatialInertia<double> M_BBcm_B(params.mass, p_BoBcm_B, G_BBcm_B);
    const RigidBody<double>& body = plant_->AddRigidBody(params.name, M_BBcm_B);

    // Add collision geometry.
    const geometry::Box shape(params.size, params.size, params.size);
    const ProximityProperties properties =
        MakePointContactProximityProperties(params.contact_parameters);
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

  static ProximityProperties MakePointContactProximityProperties(
      const PointContactParameters& params) {
    ProximityProperties properties;
    properties.AddProperty(geometry::internal::kMaterialGroup,
                           geometry::internal::kPointStiffness,
                           params.stiffness);
    properties.AddProperty(geometry::internal::kMaterialGroup,
                           "dissipation_time_constant",
                           params.dissipation_time_constant);
    properties.AddProperty(
        geometry::internal::kMaterialGroup, geometry::internal::kFriction,
        CoulombFriction<double>(params.friction_coefficient,
                                params.friction_coefficient));
    return properties;
  }

  double time_step_{0.001};
  PointContactParameters ground_contact_parameters_{1.0e20, 0.0, 1.0};
  BoxParameters box1_parameters_{
      "box1",
      {51 / 255, 1.0, 1.0, 1.0} /* cyan */,
      10.0 /* mass */,
      0.2 /* size */,
      {1.0e5 /* stiffness */, 0.001 /* dissipation */, 0.5 /* friction */}};  

  const RigidBody<double>* box1_{nullptr};

  SceneGraph<double>* scene_graph_{nullptr};
  MultibodyPlant<double>* plant_{nullptr};
  CompliantContactManager<double>* contact_manager_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{nullptr};
};

TEST_F(CompliantContactManagerTest, PointContactProperties) {
  auto verify_point_contact_parameters =
      [this](const RigidBody<double>& body,
             const PointContactParameters& parameters) {
        EXPECT_EQ(GetPointContactStiffness(body), parameters.stiffness);
        EXPECT_EQ(GetDissipationTimeConstant(body),
                  parameters.dissipation_time_constant);
      };
  verify_point_contact_parameters(plant_->world_body(),
                                  ground_contact_parameters_);
  verify_point_contact_parameters(*box1_, box1_parameters_.contact_parameters);
}

}  // namespace multibody
}  // namespace drake
