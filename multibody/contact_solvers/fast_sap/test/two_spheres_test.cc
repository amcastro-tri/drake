// Add SAP header?

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph_inspector.h"
#include "drake/multibody/contact_solvers/contact_configuration.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"
#include "drake/multibody/contact_solvers/sap/sap_hunt_crossley_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_properties.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::FrameId;
using drake::geometry::SceneGraphInspector;
using drake::math::RigidTransformd;
using drake::multibody::contact_solvers::internal::ContactConfiguration;
using drake::multibody::contact_solvers::internal::SapConstraintJacobian;
using drake::multibody::contact_solvers::internal::SapContactProblem;
using drake::multibody::contact_solvers::internal::SapHuntCrossleyApproximation;
using drake::multibody::contact_solvers::internal::SapHuntCrossleyConstraint;
using drake::multibody::contact_solvers::internal::SapSolver;
using drake::multibody::contact_solvers::internal::SapSolverResults;
using drake::multibody::contact_solvers::internal::SapSolverStatus;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

class TwoSpheres : public testing::Test {
 public:
  TwoSpheres() {
    systems::DiagramBuilder<double> builder{};

    multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};

    geometry::SceneGraphConfig scene_graph_config{
        .default_proximity_properties = {.compliance_type = "compliant"}};

    std::tie(plant_, scene_graph_) = multibody::AddMultibodyPlant(
        plant_config, scene_graph_config, &builder);

    // std::tie(plant_, scene_graph_) =
    //     multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

    // Add two spheres with arbitrary mass/inertia.
    std::string xml = fmt::format(R"""(
    <mujoco model="twospheres">
      <worldbody>
         <body name="sphere1"> <geom type="sphere" size="{}"/><freejoint name="freejoint1"/></body>
         <body name="sphere2"> <geom type="sphere" size="{}"/><freejoint name="freejoint2"/></body>
      </worldbody>
    </mujoco>
    )""",
                                  kRadius1, kRadius2);
    multibody::Parser parser(plant_);
    parser.AddModelsFromString(xml, "xml");
    sphere1_ = &plant_->GetBodyByName("sphere1");
    sphere2_ = &plant_->GetBodyByName("sphere2");
    sphere1_index_ = sphere1_->index();
    sphere2_index_ = sphere2_->index();
    plant_->Finalize();
    diagram_ = builder.Build();

    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());
  }

  void SetInContact(double penetration) {
    plant_->SetFreeBodyPoseInWorldFrame(plant_context_, *sphere1_,
                                        RigidTransformd::Identity());
    plant_->SetFreeBodyPoseInWorldFrame(
        plant_context_, *sphere1_,
        RigidTransformd(Vector3d(kRadius1 + kRadius2 - penetration, 0, 0)));
  }

 protected:
  // Parameters (in sync with xml)
  double kRadius1{0.1};
  double kRadius2{0.2};

  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  const multibody::RigidBody<double>* sphere1_;
  const multibody::RigidBody<double>* sphere2_;
  multibody::BodyIndex sphere1_index_;
  multibody::BodyIndex sphere2_index_;
  std::unique_ptr<systems::Context<double>> diagram_context_;
  systems::Context<double>* plant_context_;
};

std::vector<geometry::ContactSurface<double>> CalcContactSurfaces(
    const MultibodyPlant<double>& plant, const Context<double>& context) {
  auto& query_object =
      plant.get_geometry_query_input_port().Eval<geometry::QueryObject<double>>(
          context);

  return query_object.ComputeContactSurfaces(
      geometry::HydroelasticContactRepresentation::kPolygon);
}

SapContactProblem<double> MakeSapProblem(double time_step,
                                         const MultibodyPlant<double>& plant,
                                         const Context<double>& context) {
  const int nv = plant.num_velocities();

  // Single clique problem, for testing.
  MatrixXd M(nv, nv);
  plant.CalcMassMatrix(context, &M);
  std::vector<MatrixXd> A = {M};

  const Eigen::VectorBlock<const VectorXd> v0 = plant.GetVelocities(context);

  // For the two-spheres problem, we know there are no Coriolis terms nor
  // external forces (no-gravity, only contact). Therefore v* = v0.
  VectorXd v_star = v0;

  SapContactProblem<double> problem(time_step, std::move(A), std::move(v_star));
  problem.set_num_objects(plant.num_bodies());  // Simply an API requirement.

  // Add contact constraints for hydro.
  const std::vector<geometry::ContactSurface<double>> surfaces =
      CalcContactSurfaces(plant, context);

  const SceneGraphInspector<double>& inspector =
      plant.EvalSceneGraphInspector(context);

  Matrix3X<double> Jv_WAc_W(3, nv);
  Matrix3X<double> Jv_WBc_W(3, nv);
  Matrix3X<double> Jv_AcBc_W(3, nv);
  const auto& world_frame = plant.world_frame();

  // TODO(amcastro-tri): This should be retrieved from the default contact
  // properties.
  const double kDefaultDissipation = 50.0;

  const int num_surfaces = surfaces.size();
  for (int surface_index = 0; surface_index < num_surfaces; ++surface_index) {
    const auto& s = surfaces[surface_index];
    const bool M_is_compliant = s.HasGradE_M();
    const bool N_is_compliant = s.HasGradE_N();
    DRAKE_DEMAND(M_is_compliant || N_is_compliant);

    // Retrieve participating geometries and bodies.
    const FrameId Mid = inspector.GetFrameId(s.id_M());
    const FrameId Nid = inspector.GetFrameId(s.id_N());
    const RigidBody<double>* bodyA = plant.GetBodyFromFrameId(Mid);
    const RigidBody<double>* bodyB = plant.GetBodyFromFrameId(Nid);
    DRAKE_DEMAND(bodyA != nullptr && bodyB != nullptr);

    const auto& X_WA = bodyA->EvalPoseInWorld(context);
    const auto& X_WB = bodyB->EvalPoseInWorld(context);
    const Vector3d& p_WAo = X_WA.translation();
    const Vector3d& p_WBo = X_WB.translation();
    const auto& R_WA = X_WA.rotation();
    const auto& R_WB = X_WB.rotation();

    // Get hydro properties.
    const double Em = multibody::internal::GetHydroelasticModulus(
        s.id_M(), std::numeric_limits<double>::infinity(), inspector);
    const double En = multibody::internal::GetHydroelasticModulus(
        s.id_N(), std::numeric_limits<double>::infinity(), inspector);
    const double d = multibody::internal::GetCombinedHuntCrossleyDissipation(
        s.id_M(), s.id_N(), Em, En, kDefaultDissipation, inspector);
    const double mu = multibody::internal::GetCombinedDynamicCoulombFriction(
        s.id_M(), s.id_N(), inspector);

    for (int face = 0; face < s.num_faces(); ++face) {
      const double Ae = s.area(face);  // Face element area.
      if (Ae > 1.0e-14) {
        const Vector3d& nhat_BA_W = s.face_normal(face);
        const double gM = M_is_compliant
                              ? s.EvaluateGradE_M_W(face).dot(nhat_BA_W)
                              : std::numeric_limits<double>::infinity();
        const double gN = N_is_compliant
                              ? -s.EvaluateGradE_N_W(face).dot(nhat_BA_W)
                              : std::numeric_limits<double>::infinity();
        constexpr double kGradientEpsilon = 1.0e-14;
        if (gM < kGradientEpsilon || gN < kGradientEpsilon) {
          continue;
        }
        const double g = 1.0 / (1.0 / gM + 1.0 / gN);
        const Vector3d& p_WC = s.centroid(face);

        const Vector3d p_AoC_A = R_WA.transpose() * (p_WC - p_WAo);
        const Vector3d p_BoC_B = R_WB.transpose() * (p_WC - p_WBo);

        const Vector3d nhat_AB_W = -nhat_BA_W;
        math::RotationMatrixd R_WC =
            math::RotationMatrixd::MakeFromOneVector(nhat_AB_W, 2);

        //   J_AcBc_W = Jv_WBc_W - Jv_WAc_W.
        // That is the relative velocity at C is v_AcBc_W = J_AcBc_W * v.
        plant.CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable::kV, bodyA->body_frame(), p_AoC_A,
            world_frame, world_frame, &Jv_WAc_W);
        plant.CalcJacobianTranslationalVelocity(
            context, JacobianWrtVariable::kV, bodyB->body_frame(), p_BoC_B,
            world_frame, world_frame, &Jv_WBc_W);
        Jv_AcBc_W = Jv_WBc_W - Jv_WAc_W;
        Matrix3X<double> Jv_AcBc_C = R_WC.matrix().transpose() * Jv_AcBc_W;

        const Vector3d v_AcBc_W = Jv_AcBc_W * v0;
        const Vector3d v_AcBc_C = R_WC.transpose() * v_AcBc_W;
        const double vn0 = v_AcBc_C(2);

        // Pressure at the quadrature point.
        const Vector3d tri_centroid_barycentric(1 / 3., 1 / 3., 1 / 3.);
        const double p0 =
            s.is_triangle()
                ? s.tri_e_MN().Evaluate(face, tri_centroid_barycentric)
                : s.poly_e_MN().EvaluateCartesian(face, p_WC);

        const double fn0 = Ae * p0;
        const double k = Ae * g;
        const double vs = plant.stiction_tolerance();
        constexpr double sigma = 1.0e-3;

        internal::SapHuntCrossleyConstraint<double>::Parameters params{
            SapHuntCrossleyApproximation::kLagged, mu, k, d, vs, sigma};
        // All other params are irrelevant for this test.
        ContactConfiguration<double> config{.objectA = bodyA->index(),
                                            .objectB = bodyB->index(),
                                            .vn = vn0,
                                            .fe = fn0,
                                            .R_WC = R_WC};
        SapConstraintJacobian<double> J(0 /* single clique */,
                                        std::move(Jv_AcBc_C));

        problem.AddConstraint(
            std::make_unique<SapHuntCrossleyConstraint<double>>(
                std::move(config), std::move(J), params));
      }
    }
  }

  return problem;
}

TEST_F(TwoSpheres, GetContact) {
  SetInContact(0.001);

  const double time_step = 0.01;

  const std::vector<geometry::ContactSurface<double>> surfaces =
      CalcContactSurfaces(*plant_, *plant_context_);

  fmt::print("Num surfaces: {}\n", ssize(surfaces));
  fmt::print("Num pairs: {}\n", surfaces[0].num_faces());

  SapContactProblem<double> problem =
      MakeSapProblem(time_step, *plant_, *plant_context_);
  fmt::print("Problem:\n");
  fmt::print("  cliques    : {}\n", problem.num_cliques());
  fmt::print("  velocities : {}\n", problem.num_velocities());
  fmt::print("  constraints: {}\n", problem.num_constraints());

  const VectorXd v0 = plant_->GetVelocities(*plant_context_);
  SapSolver<double> sap;
  SapSolverResults<double> results;
  const SapSolverStatus status = sap.SolveWithGuess(problem, v0, &results);
  ASSERT_EQ(status, SapSolverStatus::kSuccess);

  const double accel_ratio = results.v(3) / results.v(9);
  const double mass_ratio = sphere2_->default_mass() / sphere1_->default_mass();

  fmt::print("v0: {}\n", fmt_eigen(v0.transpose()));
  fmt::print("v : {}\n", fmt_eigen(results.v.transpose()));
  fmt::print("j : {}\n", fmt_eigen(results.j.transpose()));

  EXPECT_NEAR(accel_ratio, -mass_ratio, std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(results.j(3), -results.j(9),
              std::numeric_limits<double>::epsilon());

  fmt::print("Acc. ratio : {}\n", accel_ratio);
  fmt::print("Mass ratio : {}\n", mass_ratio);
}

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
