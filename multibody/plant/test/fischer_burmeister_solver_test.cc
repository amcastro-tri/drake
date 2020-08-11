#include "drake/multibody/plant/fischer_burmeister_solver.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/lcm/lcm_interface_system.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;

class ConstactSolverDriver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstactSolverDriver);

  ConstactSolverDriver() = default;

  void LoadModel(const std::string& model_file) {
    systems::DiagramBuilder<double> builder;
    auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
    auto pair = AddMultibodyPlantSceneGraph(&builder, 0.0);
    plant_ = &pair.plant;
    scene_graph_ = &pair.scene_graph;

    //"drake/examples/atlas/urdf/atlas_convex_hull.urdf"
    const std::string full_name = FindResourceOrThrow(model_file);
    multibody::Parser(plant_).AddModelFromFile(full_name);

    // We set gravity to a simpler number for tests.
    plant_->mutable_gravity_field().set_gravity_vector(
        Vector3<double>(0.0, 0.0, -10.0));

    const double kGroundDynamicFriction = 1.0;
    const double kGroundStiffness = 1.0e10;
    const double kGroundDamping = 0.0;
    AddGround(kGroundStiffness, kGroundDamping, kGroundDynamicFriction);        

    plant_->Finalize();

    // Add visualization.
    geometry::DispatchLoadMessage(*scene_graph_, lcm);
    geometry::ConnectDrakeVisualizer(&builder, pair.scene_graph);
    diagram_ = builder.Build();
  }

  systems::Context<double>& CreateDefaultContext() {
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_->GetMyMutableContextFromRoot(diagram_context_.get());
    return *plant_context_;
  }

  const MultibodyPlant<double>& plant() const {
    DRAKE_DEMAND(plant_);
    return *plant_;
  }

  MultibodyPlant<double>& mutable_plant() {
    DRAKE_DEMAND(plant_);
    return *plant_;
  }

  void AddGround(double stiffness, double damping, double dynamic_friction) {
    // We demand all geometry registration happens pre- context creation so that
    // it is all well defined within the context post- context creation.
    DRAKE_DEMAND(diagram_context_ == nullptr);
    const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
    plant_->RegisterVisualGeometry(
        plant_->world_body(), math::RigidTransformd(), geometry::HalfSpace(),
        "GroundVisualGeometry", green);
    // For a time-stepping model only static friction is used.
    const multibody::CoulombFriction<double> ground_friction(dynamic_friction,
                                                             dynamic_friction);
    plant_->RegisterCollisionGeometry(
        plant_->world_body(), math::RigidTransformd(), geometry::HalfSpace(),
        "GroundCollisionGeometry", ground_friction);

    SetPointContactParameters(plant_->world_body(), stiffness, damping);
  }

  void SetPointContactParameters(const Body<double>& body, double stiffness,
                                 double damping) {
    // We demand all geometry registration happens pre- context creation so that
    // it is all well defined within the context post- context creation.
    DRAKE_DEMAND(diagram_context_ == nullptr);
    const std::vector<geometry::GeometryId>& geometries =
        plant_->GetCollisionGeometriesForBody(body);

    PRINT_VAR(body.name());

    for (const auto id : geometries) {
      PRINT_VAR(id);
      const geometry::ProximityProperties* old_props =
          scene_graph_->model_inspector().GetProximityProperties(id);
      DRAKE_DEMAND(old_props);
      geometry::ProximityProperties new_props(*old_props);
      // Add a new property.
      PRINT_VAR(geometry::internal::kMaterialGroup);
      PRINT_VAR(geometry::internal::kHcDissipation);
      new_props.AddProperty(geometry::internal::kMaterialGroup,
                            geometry::internal::kPointStiffness, stiffness);
      new_props.AddProperty(geometry::internal::kMaterialGroup,
                            geometry::internal::kHcDissipation, damping);

      // Remove a property previously assigned.
      // new_props.RemoveProperty("old_group", "old_name_1");
      // Update the *value* of an existing property (but enforce same type).
      // new_props.UpdateProperty("old_group", "old_name_2", new_value);

      scene_graph_->AssignRole(*plant_->get_source_id(),
                               id, new_props, geometry::RoleAssign::kReplace);
    }
  }

  // For viz.
  void Publish() const { diagram_->Publish(*diagram_context_); }

  void AdvanceOneStep(const VectorX<double>& v0, double dt) {
    auto& context = *plant_context_;
    const int nv = plant_->num_velocities();
    MatrixX<double> M(nv, nv);
    plant_->CalcMassMatrix(context, &M);
    const auto penetration_pairs = CalcPointPairPenetrations(context);
    const MatrixX<double> Jc = CalcContactJacobian(context, penetration_pairs);

    // Load contact data into Eigen arrays for the solver.
    const int nc = penetration_pairs.size();
    VectorX<double> phi0(nc);
    VectorX<double> stiffness(nc);
    VectorX<double> dissipation(nc);
    VectorX<double> mu(nc);
    PackContactInfo(context, penetration_pairs, &phi0, &stiffness, &dissipation,
                    &mu);

    // Add forces due to force elements.
    MultibodyForces<double> forces(plant());
    plant_->CalcForceElementsContribution(context, &forces);

    // ID computes: tau_id = M(q)v̇ + C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W
    // So that with v̇ = 0 we get:
    //   tau_id = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W
    // We define tau = -tau_id.
    const VectorX<double> tau = -plant().CalcInverseDynamics(
        context, VectorX<double>::Zero(nv), forces);

    // Split contact Jacobian Jc into normal Jn and tangent Jt.
    MatrixX<double> Jn(nc, nv);
    MatrixX<double> Jt(2 * nc, nv);
    for (int i = 0; i < nc; ++i) {      
      Jt.row(2 * i)     = Jc.row(3 * i);
      Jt.row(2 * i + 1) = Jc.row(3 * i + 1);
      Jn.row(i)         = Jc.row(3 * i + 2);
    }

    PRINT_VARn(M);
    PRINT_VARn(Jc);
    PRINT_VAR(tau.transpose());
    PRINT_VAR(v0.transpose());
    PRINT_VAR(stiffness.transpose());
    PRINT_VAR(dissipation.transpose());
    PRINT_VAR(mu.transpose());

    ProblemData<double> data(dt, &M, &Jn, &Jt, &tau, &v0, &phi0, &stiffness,
                             &dissipation, &mu);
    FBSolver<double> solver(&data);

    // TODO: attempt other initial guesses with FBSolver::State for more
    // control.
    FBSolverResult info = solver.SolveWithGuess(v0);
    if (info != FBSolverResult::kSuccess) {
      throw std::runtime_error("Solver did not succeed.");
    }

    const FBSolver<double>::State& state = solver.get_state();

    const int nq = plant().num_positions();
    VectorX<double> x(plant().num_multibody_states());
    auto q = x.head(nq);
    auto v = x.tail(nv);
    const auto q0 = plant().GetPositions(context);
    plant().MapVelocityToQDot(context, v, &q);  // q = qdot
    q *= dt;  // q = dt * qdot
    q += q0;  // q = q0 + dt * qdot.
    plant().SetPositionsAndVelocities(&context, x);
  }

  void PackContactInfo(
      const systems::Context<double>& context,
      const std::vector<geometry::PenetrationAsPointPair<double>>& pairs,
      VectorX<double>* phi0, VectorX<double>* stiffness,
      VectorX<double>* dissipation, VectorX<double>* mu) const {
    plant_->ValidateContext(context);         
    const int nc = static_cast<int>(pairs.size());
    phi0->resize(nc);
    stiffness->resize(nc);
    dissipation->resize(nc);
    mu->resize(nc);

    const auto& inspector = GetInspector(context);
    for (size_t icontact = 0; icontact < pairs.size(); ++icontact) {
      const auto& pair = pairs[icontact];
      const auto geometryA_id = pair.id_A;
      const auto geometryB_id = pair.id_B;      
      const auto [kA, dA] = GetPointContactParameters(geometryA_id, inspector);
      const auto [kB, dB] = GetPointContactParameters(geometryB_id, inspector);
      const auto [k, d] = CombinePointContactParameters(kA, kB, dA, dB);      

      const auto frictionA = GetCoulombFriction(geometryA_id, inspector);
      const auto frictionB = GetCoulombFriction(geometryB_id, inspector);
      const auto friction =
          CalcContactFrictionFromSurfaceProperties(frictionA, frictionB);

      (*phi0)[icontact] = pair.depth;
      (*stiffness)[icontact] = k;
      (*dissipation)[icontact] = d;
      (*mu)[icontact] = friction.dynamic_friction();
    }
  }


  std::vector<geometry::PenetrationAsPointPair<double>> CalcPointPairPenetrations(
      const systems::Context<double>& context) {
    plant_->ValidateContext(context);    
    const geometry::QueryObject<double>& query_object =
        plant_->get_geometry_query_input_port()
            .template Eval<geometry::QueryObject<double>>(context);
    return query_object.ComputePointPairPenetration();
  }

  std::pair<double, double> GetPointContactParameters(
      geometry::GeometryId geometry_id,
      const geometry::SceneGraphInspector<double>& inspector) const {
    const geometry::ProximityProperties* prop =
        inspector.GetProximityProperties(geometry_id);
    DRAKE_DEMAND(prop != nullptr);

    auto frame = inspector.GetFrameId(geometry_id);
    const auto& body = *plant_->GetBodyFromFrameId(frame);
    PRINT_VAR(geometry_id);
    PRINT_VAR(body.name());

    // These properties are required by the driver. GetProperty() will throw if
    // not present.
    return std::pair(
        prop->template GetProperty<double>(geometry::internal::kMaterialGroup,
                                           geometry::internal::kPointStiffness),
        prop->template GetProperty<double>(geometry::internal::kMaterialGroup,
                                           geometry::internal::kHcDissipation));
  }

  static std::pair<double, double> CombinePointContactParameters(double k1,
                                                                 double k2,
                                                                 double d1,
                                                                 double d2) {
    // Simple utility to detect 0 / 0. As it is used in this method, denom
    // can only be zero if num is also zero, so we'll simply return zero.
    auto safe_divide = [](double num, double denom) {
      return denom == 0.0 ? 0.0 : num / denom;
    };
    return std::pair(
        safe_divide(k1 * k2, k1 + k2),                                   // k
        safe_divide(k2, k1 + k2) * d1 + safe_divide(k1, k1 + k2) * d2);  // d
  }

  CoulombFriction<double> GetCoulombFriction(
      geometry::GeometryId id,
      const geometry::SceneGraphInspector<double>& inspector) const {    
    const geometry::ProximityProperties* prop =
        inspector.GetProximityProperties(id);
    DRAKE_DEMAND(prop != nullptr);
    DRAKE_THROW_UNLESS(prop->HasProperty(geometry::internal::kMaterialGroup,
                                         geometry::internal::kFriction));
    return prop->GetProperty<CoulombFriction<double>>(
        geometry::internal::kMaterialGroup, geometry::internal::kFriction);
  }

  const geometry::SceneGraphInspector<double>& GetInspector(
      const systems::Context<double>& context) const {
    plant_->ValidateContext(context);
    const geometry::QueryObject<double>& query_object =
        plant_->get_geometry_query_input_port()
            .template Eval<geometry::QueryObject<double>>(context);
    return query_object.inspector();
  }

  MatrixX<double> CalcContactJacobian(
      const systems::Context<double>& context,
      const std::vector<geometry::PenetrationAsPointPair<double>>& pairs) {
    plant_->ValidateContext(context);        
    const geometry::QueryObject<double>& query_object =
        plant_->get_geometry_query_input_port()
            .template Eval<geometry::QueryObject<double>>(context);
    const auto& inspector = query_object.inspector();

    auto body_from_geometry =
        [&, this](geometry::GeometryId id) -> const Body<double>& {
      const geometry::FrameId frame_id = inspector.GetFrameId(id);
      const auto& body = plant_->GetBodyFromFrameId(frame_id);
      DRAKE_DEMAND(body != nullptr);
      return *body;
    };

    const int nv = plant_->num_velocities();
    const int nc = static_cast<int>(pairs.size());
    Matrix3X<double> Jv_WAc(3, nv);
    Matrix3X<double> Jv_WBc(3, nv);
    MatrixX<double> Jc(3 * nc, nv);

    const auto& world_frame = plant_->world_frame();

    int ic = 0;
    for (const auto& p : pairs) {
      const Vector3<double>& p_WC = 0.5 * (p.p_WCa + p.p_WCb);

      const Body<double>& bodyA = body_from_geometry(p.id_A);    
      const auto& X_WA = bodyA.EvalPoseInWorld(context);
      const auto& p_WAo = X_WA.translation();
      const auto& R_WA = X_WA.rotation();    
      const Vector3<double> p_AoC_A = R_WA.transpose() * (p_WC - p_WAo);
      plant_->CalcJacobianTranslationalVelocity(
          context, JacobianWrtVariable::kV, bodyA.body_frame(), p_AoC_A,
          world_frame, world_frame, &Jv_WAc);

      const Body<double>& bodyB = body_from_geometry(p.id_B);
      const auto& X_WB = bodyB.EvalPoseInWorld(context);
      const auto& p_WBo = X_WB.translation();
      const auto& R_WB = X_WB.rotation();
      const Vector3<double> p_BoC_B = R_WB.transpose() * (p_WC - p_WBo);
      plant_->CalcJacobianTranslationalVelocity(
          context, JacobianWrtVariable::kV, bodyB.body_frame(), p_BoC_B,
          world_frame, world_frame, &Jv_WBc);      

      const Vector3<double>& nhat_BA_W = p.nhat_BA_W;        
      const math::RotationMatrix<double> R_WC(
          math::ComputeBasisFromAxis(2, nhat_BA_W));
      const Vector3<double> that1_W = R_WC.matrix().col(0);  // that1 = Cx.
      const Vector3<double> that2_W = R_WC.matrix().col(1);  // that2 = Cy.
      const Vector3<double> nhat_W = R_WC.matrix().col(2);   // nhat = nhat_BA_W

      Jc.row(3 * ic)     = that1_W.transpose() * (Jv_WAc - Jv_WBc);
      Jc.row(3 * ic + 1) = that2_W.transpose() * (Jv_WAc - Jv_WBc);
      Jc.row(3 * ic + 2) = nhat_W.transpose() * (Jv_WAc - Jv_WBc);
      ++ic;
    }

    return Jc;
  }

 private:
  MultibodyPlant<double>* plant_{nullptr};
  geometry::SceneGraph<double>* scene_graph_{nullptr};
  std::unique_ptr<systems::Diagram<double>> diagram_;
  std::unique_ptr<systems::Context<double>> diagram_context_;
  systems::Context<double>* plant_context_{nullptr};
};

GTEST_TEST(FBSolver, Particle) {
  const std::string model_file = "drake/multibody/plant/test/particle.sdf";
  ConstactSolverDriver driver;
  driver.LoadModel(model_file);
  const double dt = 1.0e-3;
  const auto& plant = driver.plant();
  const int nv = driver.plant().num_velocities();
  const int nq = driver.plant().num_positions();  
  ASSERT_EQ(nq, 7);
  ASSERT_EQ(nv, 6);

  const double kStiffness = 1.0e10;
  const double kDamping = 0.0;

  const auto& sphere = plant.GetBodyByName("spherical_mass");
  driver.SetPointContactParameters(sphere, kStiffness, kDamping);
  auto& context = driver.CreateDefaultContext();

  const double phi0 = -1.0e-3;  // Initial signed distance.
  plant.SetFreeBodyPose(&context, sphere,
                        RigidTransformd(Vector3d(0, 0, 0.05 + phi0)));

  // Visualize initial condition.
  driver.Publish();  // for viz.  

  const VectorX<double> v0 = VectorX<double>::Zero(nv);
  driver.AdvanceOneStep(v0, dt);
  driver.Publish();  // for viz.  
}

GTEST_TEST(ScratchWorkspace, NestedScopes) {
  const int kNv = 20;
  const int kNc = 30;
  const int kMaxVectors = 6;
  ScratchWorkspace<double> w(kNv, kNc, kMaxVectors);
  GrantScratchWorkspaceAccess<double> a1(w);
  auto& xc11 = a1.xc_sized_vector();
  auto& xc12 = a1.xc_sized_vector();
  ASSERT_EQ(xc11.size(), 3 * kNc);
  ASSERT_EQ(xc12.size(), 3 * kNc);
  EXPECT_EQ(w.num_xc_vectors(), 2);
  {  // A nested scope.
    GrantScratchWorkspaceAccess<double> a2(w);
    auto& xc21 = a2.xc_sized_vector();
    auto& xc22 = a2.xc_sized_vector();    

    // Expected sizes for the new vectors.
    ASSERT_EQ(xc21.size(), 3 * kNc);
    ASSERT_EQ(xc22.size(), 3 * kNc);

    // Vectors from the previous scope should still be valid.
    ASSERT_EQ(xc11.size(), 3 * kNc);
    ASSERT_EQ(xc12.size(), 3 * kNc);

    EXPECT_EQ(w.num_xc_vectors(), 4);
  }
  EXPECT_EQ(w.xc_vectors_capacity(), 4);

  {  // Another nested scope.
    GrantScratchWorkspaceAccess<double> a2(w);
    auto& xc21 = a2.xc_sized_vector();

    // Expected sizes for the new vectors.
    ASSERT_EQ(xc21.size(), 3 * kNc);

    EXPECT_EQ(w.num_xc_vectors(), 3);
    EXPECT_EQ(w.xc_vectors_capacity(), 4);
  }

  {  // Another nested scope.
    GrantScratchWorkspaceAccess<double> a2(w);
    auto& xc21 = a2.xc_sized_vector();
    auto& xc22 = a2.xc_sized_vector();
    auto& xc23 = a2.xc_sized_vector();

    // Expected sizes for the new vectors.
    ASSERT_EQ(xc21.size(), 3 * kNc);
    ASSERT_EQ(xc22.size(), 3 * kNc);
    ASSERT_EQ(xc23.size(), 3 * kNc);

    EXPECT_EQ(w.num_xc_vectors(), 5);
    EXPECT_EQ(w.xc_vectors_capacity(), 5);
  }
  EXPECT_EQ(w.num_xc_vectors(), 2);
  EXPECT_EQ(w.xc_vectors_capacity(), 5);
  
  // Within nested loops.
  for (int i = 0; i< 10; ++i) {
    GrantScratchWorkspaceAccess<double> a3(w);
    auto& xc31 = a3.xc_sized_vector();
    auto& xc32 = a3.xc_sized_vector();
    ASSERT_EQ(xc31.size(), 3 * kNc);
    ASSERT_EQ(xc32.size(), 3 * kNc);    
    EXPECT_EQ(w.num_xc_vectors(), 4);
    for (int j = 0; j< 10; ++j) {
      GrantScratchWorkspaceAccess<double> a4(w);
      auto& xc41 = a4.xc_sized_vector();
      auto& xc42 = a4.xc_sized_vector();
      ASSERT_EQ(xc41.size(), 3 * kNc);
      ASSERT_EQ(xc42.size(), 3 * kNc);
      EXPECT_EQ(w.num_xc_vectors(), 6);
      EXPECT_EQ(w.xc_vectors_capacity(), 6);
    }
    EXPECT_EQ(w.num_xc_vectors(), 4);
    EXPECT_EQ(w.xc_vectors_capacity(), 6);
  }
}

class Particle : public ::testing::Test {
 public:
  void SetUp() override {
    // Now we'll set up each term in the equation:
    //   Mv̇ = τ + Dᵀ fₜ
    // where τ =[Fx, Fy, Fz, Mz] contains the external force in x, the external
    // force in y and the external moment about z (out of plane).
    M_ << m_, 0, 0, m_;
  }

  void SetProblem(const VectorX<double>& v0, const VectorX<double>& tau,
                  double mu, double dt) {
    // Next time step generalized momentum if there are no friction forces.
    p_star_ = M_ * v0 + dt * tau;

    // Normal forces. Assume they are equally distributed.
    const double k = m_ * g_ / penetration_;
    stiffness_ = Vector1<double>(k);
    dissipation_ = Vector1<double>::Zero();
    phi0_ = Vector1<double>(penetration_);
    mu_ = Vector1<double>(mu);

    Jn_.resize(1, 2);
    Jn_ << 0.0, 1.0;
    Jt_.resize(2, 2);
    Jt_ << 1, 0, 
           0, 0;

    data_ = std::make_unique<ProblemData<double>>(dt, &M_, &Jn_, &Jt_, &tau,
                                                  &v0, &phi0_, &stiffness_,
                                                  &dissipation_, &mu_);

    solver_ = std::make_unique<FBSolver<double>>(data_.get());
  }

 protected:
  // Problem parameters.
  const double m_{1.0};   // Mass of the pizza saver.
  const double g_{10.0};  // Acceleration of gravity.

  // This determines a level of compliance for which the solution is only 1e-5
  // (relative) away from the rigid solution (measured in the total normal
  // force.)
  const double penetration_{1.0e-40};

  // Problem sizes.
  const int nv_{2};  // number of generalized velocities.
  const int nc_{1};  // number of contact points.

  // Mass matrix.
  MatrixX<double> M_{nv_, nv_};

  // The separation velocities Jacobian.
  MatrixX<double> Jn_{nc_, nv_};

  // Tangential velocities Jacobian.
  MatrixX<double> Jt_{2 * nc_, nv_};

  VectorX<double> stiffness_;
  VectorX<double> dissipation_;

  std::unique_ptr<ProblemData<double>> data_;

  // The TAMSI solver for this problem.
  std::unique_ptr<FBSolver<double>> solver_;

  // Additional solver data that must outlive solver_ during solution.
  VectorX<double> p_star_;  // Generalized momentum.
  VectorX<double> phi0_;      // Normal forces at each contact point.
  VectorX<double> mu_;      // Friction coefficient at each contact point.
};

// This tests the solver when we apply a moment Mz about COM to the pizza saver.
// If Mz < mu * m * g * R, the saver should be in stiction (that is, the sliding
// velocity should be smaller than the regularization parameter). Otherwise the
// saver will start sliding. For this setup the transition occurs at
// M_transition = mu * m * g * R = 5.0
TEST_F(Particle, SteadyState) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 1.0;  // Irrelevant for this problem.
  const double Fx = 15.0; // External force in x.

  const double weight = m_ * g_;
  const VectorX<double> tau = Vector2<double>(Fx, -weight);
  const VectorX<double> zero = Vector2<double>::Zero();
  const VectorX<double> v0 = zero;

  SetProblem(v0, tau, mu, dt);

  FBSolverParameters parameters;
  parameters.stiction_tolerance = 0.0;
  parameters.alpha_stab = -1;  // for 0 < v + Rn * pi ⊥ pi > 0

  parameters.outer_loop_max_iters = 100;
  parameters.outer_loop_tolerance = 1.0e-4;
  parameters.inner_loop_max_iters = 1;
  parameters.initialization_max_iters = 1;
  parameters.inner_loop_tolerance = 0.5;

  parameters.relaxation = 1.0;
  parameters.delta = 0.;
  parameters.max_ls_iters = 0;
  parameters.limit_to_feasible_values = false;
  parameters.complementary_slackness_tolerance = 1.0;

  solver_->set_parameters(parameters);

  FBSolver<double>::State s_guess(nv_, nc_);
  s_guess.SetDt(dt);
  s_guess.SetComplementaritySlackness(0.0);
  s_guess.SetVelocities(zero);
  s_guess.SetImpulses(Vector3<double>::Zero());

  FBSolverResult info = solver_->SolveWithGuess(s_guess);  
  EXPECT_EQ(info, FBSolverResult::kSuccess);

  auto& stats = solver_->get_stats();
  PRINT_VAR(stats.initialization_iters);
  PRINT_VAR(stats.outer_iters);
  PRINT_VAR(stats.total_iterations);
  VectorX<double> num_inner_iters(stats.num_inner_iters.size());
  std::copy(stats.num_inner_iters.begin(), stats.num_inner_iters.end(),
            num_inner_iters.data());
  PRINT_VAR(num_inner_iters.transpose());
  PRINT_VAR(stats.dvn_max_norm);
  PRINT_VAR(stats.gpi_max_norm);  

  PRINT_VAR(solver_->get_normal_forces());

  const VectorX<double> pi = solver_->get_normal_forces();
  const double total_force = pi.sum() / dt;
  PRINT_VAR(1.0 - total_force / weight);
  EXPECT_NEAR(total_force, weight, parameters.outer_loop_tolerance * weight);

  const auto& state = solver_->get_state();
  PRINT_VAR(state.m());
  PRINT_VAR(state.v().transpose());
  PRINT_VAR(state.gamma().transpose());
  PRINT_VAR(state.lambda().transpose());

}

/* Top view of the pizza saver:

  ^ y               C
  |                 ◯
  |                /\
  ----> x         /  \
               b /    \ a
                /      \
               /        \
              ◯----------◯
              A     c    B

It is modeled as an equilateral triangle with a contact point at each of the
legs. The total mass of the pizza saver is m and its rotational inertia about
the triangle's barycenter is I.
If h is the height of the triangle from any of its sides, the distance from
any point to the triangle's center is 2h/3. The height h relates to the length
a of a side by h = 3/2/sqrt(3) a.
The generalized positions vector for this case is q = [x, y, theta], with
theta = 0 for the triangle in the configuration shown in the schematic. */
class PizzaSaver : public ::testing::Test {
 public:
  void SetUp() override {
    // Now we'll set up each term in the equation:
    //   Mv̇ = τ + Dᵀ fₜ
    // where τ =[Fx, Fy, Fz, Mz] contains the external force in x, the external
    // force in y and the external moment about z (out of plane).
    M_ << m_,  0,  0, 0,
           0, m_,  0, 0,
           0,  0, m_, 0,
           0,  0,  0, I_;
  }

  MatrixX<double> ComputeTangentialJacobian(double theta) {
    MatrixX<double> Jt(2 * nc_, nv_);
    const double c = cos(theta);
    const double s = sin(theta);

    // 2D rotation matrix of the body frame B in the world frame W.
    Matrix2<double> R_WB;
    R_WB <<  c, s,
            -s, c;

    // Position of each contact point in the body frame B.
    const Vector2<double> p_BoA(-sqrt(3) / 2.0, -0.5);
    const Vector2<double> p_BoB(sqrt(3) / 2.0, -0.5);
    const Vector2<double> p_BoC(0.0, 1.0);

    // Position of each contact point in the world frame W.
    const Vector2<double> p_BoA_W = R_WB * p_BoA;
    const Vector2<double> p_BoB_W = R_WB * p_BoB;
    const Vector2<double> p_BoC_W = R_WB * p_BoC;

    // Point A
    Jt.block(0, 0, 2, nv_) << 1, 0, 0, -p_BoA_W.y(),
        0, 1, 0, p_BoA_W.x();

    // Point B
    Jt.block(2, 0, 2, nv_) << 1, 0, 0, -p_BoB_W.y(),
        0, 1, 0, p_BoB_W.x();

    // Point C
    Jt.block(4, 0, 2, nv_) << 1, 0, 0, -p_BoC_W.y(),
        0, 1, 0, p_BoC_W.x();

    return Jt;
  }

  MatrixX<double> ComputeNormalJacobian() {
    MatrixX<double> Jn(nc_, nv_);
    Eigen::RowVector4d row(0.0, 0.0, 1.0, 0.0);
    Jn << row, row, row;
    return Jn;
  }

  void SetProblem(const VectorX<double>& v0, const VectorX<double>& tau,
                  double mu, double theta, double dt) {
    // Next time step generalized momentum if there are no friction forces.
    p_star_ = M_ * v0 + dt * tau;

    // Normal forces. Assume they are equally distributed.
    const double k = m_ * g_ / penetration_ / nc_;
    stiffness_ = VectorX<double>::Constant(nc_, k);
    dissipation_ = VectorX<double>::Zero(nc_);
    phi0_ = VectorX<double>::Constant(nc_, penetration_);

    // All contact points have the same friction for this case.
    mu_ = mu * VectorX<double>::Ones(nc_);

    Jn_ = ComputeNormalJacobian();
    Jt_ = ComputeTangentialJacobian(theta);

    // TODO: make ProblemData to take const pointers instead, to communicate we
    // keep references to this data.
    data_ = std::make_unique<ProblemData<double>>(dt, &M_, &Jn_, &Jt_, &tau,
                                                  &v0, &phi0_, &stiffness_,
                                                  &dissipation_, &mu_);

    solver_ = std::make_unique<FBSolver<double>>(data_.get());
  }

 protected:
  // Problem parameters.
  const double m_{1.0};   // Mass of the pizza saver.
  const double R_{1.0};   // Distance from COM to any contact point.
  const double g_{10.0};  // Acceleration of gravity.
  // The radius of the circumscribed circle R is the distance from each
  // contact point to the triangle's center.
  // If we model the pizza saver as three point masses m/3 at each contact
  // point, the moment of inertia is I = 3 * (m/3 R²):
  const double I_{R_ * R_ * m_};  // = 1.0 in this case.

  // This determines a level of compliance for which the solution is only 1e-5
  // (relative) away from the rigid solution (measured in the total normal
  // force.)
  const double penetration_{1.0e-10};

  // Problem sizes.
  const int nv_{4};  // number of generalized velocities.
  const int nc_{3};  // number of contact points.

  // Mass matrix.
  MatrixX<double> M_{nv_, nv_};

  // The separation velocities Jacobian.
  MatrixX<double> Jn_{nc_, nv_};

  // Tangential velocities Jacobian.
  MatrixX<double> Jt_{2 * nc_, nv_};

  VectorX<double> stiffness_;
  VectorX<double> dissipation_;

  std::unique_ptr<ProblemData<double>> data_;

  // The TAMSI solver for this problem.
  std::unique_ptr<FBSolver<double>> solver_;

  // Additional solver data that must outlive solver_ during solution.
  VectorX<double> p_star_;  // Generalized momentum.
  VectorX<double> phi0_;      // Normal forces at each contact point.
  VectorX<double> mu_;      // Friction coefficient at each contact point.
};

// This tests the solver when we apply a moment Mz about COM to the pizza saver.
// If Mz < mu * m * g * R, the saver should be in stiction (that is, the sliding
// velocity should be smaller than the regularization parameter). Otherwise the
// saver will start sliding. For this setup the transition occurs at
// M_transition = mu * m * g * R = 5.0
TEST_F(PizzaSaver, SmallAppliedMoment) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.5;

  // Some arbitrary orientation. This particular case has symmetry of
  // revolution (meaning the result is independent of angle theta).
  const double theta = M_PI / 5;

  // External forcing.
  const double Mz = 0.0;
   //3.0;  // M_transition = 5.0
  const VectorX<double> tau = Vector4<double>(0.0, 0.0, -m_ * g_, Mz);

  // Initial velocity.
  const VectorX<double> v0 = VectorX<double>::Zero(nv_);

  SetProblem(v0, tau, mu, theta, dt);

  FBSolverParameters parameters;
  parameters.stiction_tolerance = 1e-6;
  parameters.alpha_stab = -1;  // for 0 < v + Rn * pi ⊥ pi > 0

  parameters.outer_loop_max_iters = 20;
  parameters.outer_loop_tolerance = 1.0e-3;
  parameters.inner_loop_max_iters = 5;
  parameters.inner_loop_tolerance = 0.5;
  parameters.initialization_max_iters = 5;

  parameters.relaxation = 1.0;
  parameters.delta = 0.01;
  parameters.max_ls_iters = 0;
  parameters.limit_to_feasible_values = false;
  parameters.complementary_slackness_tolerance = 1.0;

  solver_->set_parameters(parameters);

  FBSolverResult info = solver_->SolveWithGuess(v0);  

  auto& stats = solver_->get_stats();
  PRINT_VAR(stats.initialization_iters);
  PRINT_VAR(stats.outer_iters);
  PRINT_VAR(stats.total_iterations);
  VectorX<double> num_inner_iters(stats.num_inner_iters.size());
  std::copy(stats.num_inner_iters.begin(), stats.num_inner_iters.end(),
            num_inner_iters.data());
  PRINT_VAR(num_inner_iters.transpose());
  PRINT_VAR(stats.dvn_max_norm);
  PRINT_VAR(stats.gpi_max_norm);

  ASSERT_EQ(info, FBSolverResult::kSuccess);

  PRINT_VAR(solver_->get_normal_forces().transpose());

  const VectorX<double> pi = solver_->get_normal_forces();
  const double total_force = pi.sum() / dt;
  PRINT_VAR(1.0 - total_force / (m_ * g_));
  EXPECT_NEAR(total_force, m_ * g_, parameters.outer_loop_tolerance * m_ * g_);

#if 0
  VectorX<double> tau_f = solver_.get_generalized_friction_forces();

  const auto& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      // Dimensionless relative (to the stiction tolerance) tolerance.
      solver_.get_solver_parameters().outer_loop_tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  // For this problem we expect the x and y components of the forces due to
  // friction to be zero.
  EXPECT_NEAR(tau_f(0), 0.0, kTolerance);
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

  // The moment due to friction should balance the applied Mz. However, it will
  // take several time steps until tau_f balances Mz (eventually it will).
  // Therefore, here we just sanity check that Mz is at least relatively close
  // (to the value of Mz) to tau_f. In other words, with only a single time
  // step, we are still accelerating towards the final steady state slip
  // introduced by having a finite stiction tolerance.
  EXPECT_NEAR(tau_f(2), -Mz, 5.0e-4);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // The problem has symmetry of revolution. Thus, for any rotation theta,
  // the three tangential velocities should have the same magnitude.
  const double v_slipA = vt.segment<2>(0).norm();
  const double v_slipB = vt.segment<2>(2).norm();
  const double v_slipC = vt.segment<2>(4).norm();
  EXPECT_NEAR(v_slipA, v_slipB, kTolerance);
  EXPECT_NEAR(v_slipA, v_slipC, kTolerance);
  EXPECT_NEAR(v_slipC, v_slipB, kTolerance);

  // For this case where Mz < M_transition, we expect stiction (slip velocities
  // are smaller than the regularization parameter).
  EXPECT_LT(v_slipA, parameters.stiction_tolerance);
  EXPECT_LT(v_slipB, parameters.stiction_tolerance);
  EXPECT_LT(v_slipC, parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities to be zero.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);

  // Compute the Newton-Raphson Jacobian of the residual J = ∇ᵥR using the
  // solver's internal implementation.
  MatrixX<double> J =
      TamsiSolverTester::CalcJacobian(solver_, v, dt);

  // Compute the same Newton-Raphson Jacobian of the residual J = ∇ᵥR but with
  // a completely separate implementation using automatic differentiation.
  const double v_stiction = parameters.stiction_tolerance;
  const double epsilon_v = v_stiction * parameters.outer_loop_tolerance;
  MatrixX<double> J_expected = test::CalcOneWayCoupledJacobianWithAutoDiff(
      M_, Jn_, Jt_, p_star_, mu_, fn_, dt, v_stiction, epsilon_v, v);

  // We use a tolerance scaled by the norm and size of the matrix.
  const double J_tolerance =
      J_expected.rows() * J_expected.norm() *
          std::numeric_limits<double>::epsilon();

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));
#endif      
}


#if 0
// Exactly the same problem as in PizzaSaver::SmallAppliedMoment but with an
// applied moment Mz = 6.0 > M_transition = 5.0. In this case the pizza saver
// transitions to sliding with a net moment of Mz - M_transition during a
// period (time stepping interval) dt. Therefore we expect a change of angular
// velocity given by Δω = dt (Mz - Mtransition) / I.
TEST_F(PizzaSaver, LargeAppliedMoment) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.5;  // Friction coefficient.

  // Some arbitrary orientation. This particular case has symmetry of
  // revolution.
  const double theta = M_PI / 5;

  // External forcing.
  const double M_transition = 5.0;
  const double Mz = 6.0;
  const Vector3<double> tau(0.0, 0.0, Mz);

  // Initial velocity.
  const Vector3<double> v0 = Vector3<double>::Zero();

  SetProblem(v0, tau, mu, theta, dt);

  TamsiSolverParameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  parameters.outer_loop_tolerance = 1.0e-4;
  solver_.set_solver_parameters(parameters);

  TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, TamsiSolverResult::kSuccess);

  VectorX<double> tau_f = solver_.get_generalized_friction_forces();

  const auto& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      // Dimensionless relative (to the stiction tolerance) tolerance.
      solver_.get_solver_parameters().outer_loop_tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  // For this problem we expect the x and y components of the forces due to
  // friction to be zero.
  EXPECT_NEAR(tau_f(0), 0.0, kTolerance);
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);
  // Since we are sliding, the total moment should match M_transition.
  // The difference with Mz is what makes the saver to start accelerating.
  EXPECT_NEAR(tau_f(2), -M_transition, 1.0e-13);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // The problem has symmetry of revolution. Thus, for any rotation theta,
  // the three tangential velocities should have the same magnitude.
  const double v_slipA = vt.segment<2>(0).norm();
  const double v_slipB = vt.segment<2>(2).norm();
  const double v_slipC = vt.segment<2>(4).norm();
  EXPECT_NEAR(v_slipA, v_slipB, kTolerance);
  EXPECT_NEAR(v_slipA, v_slipC, kTolerance);
  EXPECT_NEAR(v_slipC, v_slipB, kTolerance);

  // For this case where Mz > M_transition, we expect sliding, so that expected
  // velocities are larger than the stiction tolerance.
  EXPECT_GT(v_slipA, parameters.stiction_tolerance);
  EXPECT_GT(v_slipB, parameters.stiction_tolerance);
  EXPECT_GT(v_slipC, parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities for the COM to be
  // zero. Still, there is slip at points A, B, C.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);

  const double omega = dt * (Mz - 5.0) / I_;
  EXPECT_NEAR(v(2), omega, kTolerance);

  // Slip velocities should only be due to rotation.
  EXPECT_NEAR(v_slipA, R_ * omega, kTolerance);
  EXPECT_NEAR(v_slipB, R_ * omega, kTolerance);
  EXPECT_NEAR(v_slipC, R_ * omega, kTolerance);

  // Compute the Newton-Raphson Jacobian of the residual J = ∇ᵥR using the
  // solver's internal implementation.
  MatrixX<double> J =
      TamsiSolverTester::CalcJacobian(solver_, v, dt);

  // Compute the same Newton-Raphson Jacobian of the residual J = ∇ᵥR but with
  // a completely separate implementation using automatic differentiation.
  const double v_stiction = parameters.stiction_tolerance;
  const double epsilon_v = v_stiction * parameters.outer_loop_tolerance;
  MatrixX<double> J_expected = test::CalcOneWayCoupledJacobianWithAutoDiff(
      M_, Jn_, Jt_, p_star_, mu_, fn_, dt, v_stiction, epsilon_v, v);

  // We use a tolerance scaled by the norm and size of the matrix.
  const double J_tolerance = J_expected.rows() * J_expected.norm() *
      std::numeric_limits<double>::epsilon();

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));
}

// Verify the solver behaves correctly when the problem data contains no
// contact points.
TEST_F(PizzaSaver, NoContact) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  const double dt = 1.0e-3;  // time step in seconds.

  // External forcing.
  const double Mz = 6.0;
  const Vector3<double> tau(0.0, 0.0, Mz);

  // Initial velocity.
  const Vector3<double> v0 = Vector3<double>::Zero();

  SetNoContactProblem(v0, tau, dt);

  TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, TamsiSolverResult::kSuccess);

  EXPECT_EQ(solver_.get_generalized_friction_forces(), Vector3<double>::Zero());

  const auto& stats = solver_.get_iteration_statistics();
  EXPECT_EQ(stats.vt_residual(), 0);
  EXPECT_EQ(stats.num_iterations, 1);

  // Verify solution.
  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // For this problem we expect the translational velocities to be zero.
  EXPECT_NEAR(v(0), 0.0, kTolerance);
  EXPECT_NEAR(v(1), 0.0, kTolerance);
  // Expected angular velocity change about z due to the applied moment Mz.
  const double omega = dt * Mz / I_;
  EXPECT_NEAR(v(2), omega, kTolerance);

  // No contact.
  EXPECT_EQ(solver_.get_tangential_velocities().size(), 0);
}

// This test verifies that TAMSI can correctly predict transitions in a problem
// with impact. In this test the y axis is in the "up" vertical direction, the x
// axis points to the right and the z axis comes out of the x-y plane forming a
// right handed basis. Gravity points in the minus y direction. The x-z plane is
// the ground.
// In this test a cylinder moves parallel to the ground with its revolute axis
// parallel to the z axis at all times. The cylinder's COM is constrained to
// move in the x-y plane. Therefore, the cylinder's motions are described by
// three degrees of freedom (DOFs); two translational DOFs in the x-y plane and
// a rotational DOF about its revolute axis. The cylinder has radius R,
// mass m, and rotational inertia I, and it is initially at height y0 = h0 + R
// from a flat ground.
// At t = 0, the cylinder is given an initial horizontal velocity vx0 and zero
// vertical velocity vy0 = 0. The cylinder will undergo a parabolic free flight
// towards the ground until the moment of impact at which the vertical velocity
// goes to zero (a purely inelastic collision). Since we know the initial
// height, we therefore know that the vertical velocity at the time of impact
// will be vy = -sqrt(2 g h0), with g the acceleration of gravity. Therefore the
// change of momentum, in the vertical direction, at the time of impact will be
// py = m * vy.
// TAMSI needs to know the normal forces in advance. We compute the normal force
// in order to exactly bring the cylinder to a stop in the vertical direction
// within a time interval dt from the time that the bodies first make contact.
// That is, we set the normal force to fn = -m * vy / dt + m * g, where the
// small contribution due to gravity is needed to exactly bring the cylinder's
// vertical velocity to zero. The solver keeps this value constant throughout
// the computation.
// The equations governing the motion for the cylinder during impact are:
//   (1)  I Δω = pt R,  Δω  = ω, since ω0 = 0.
//   (2)  m Δvx = pt ,  Δvx = vx - vx0
//   (3)  vt = vx + ω R
//   (4)  |pt| ≤ μ pn
// where pt = dt ft and pn = dt fn are the impulses due to friction (in the
// tangential direction) and due to the normal force, respectively.
// The problem above can be solved analytically for vx, ω and ft. We will find
// the condition for either stiction or sliding after impact by solving the
// easier stiction problem and subsequently verifying the condition given by
// Eq. (4).
//
//                  Stiction After Impact, vt = 0.
// Setting vt = 0 in Eq. (3), solving for ω and substituting the result in
// Eq. (1) leads now, together with Eq. (2), to a system of equations in vx and
// pt. We solve it for pt to find:
//   pt = -m vx0 / (1 + m R²/I)
// From Eq. (4), stiction occurs if vx0 < vx_transition, with vx_transition
// defined as:
//   vx_transition =  μ (1 + m R²/I) pn/m
// Otherwise the cylinder will be sliding after impact.
class RollingCylinder : public ::testing::Test {
 public:
  void SetUp() override {
    // Mass matrix corresponding to free (in 2D) cylinder.
    // Generalized velocities are v = [vx, vy, ω].
    M_ << m_,  0,  0,
           0, m_,  0,
           0,  0,  I_;
  }

  // Computes tangential velocity Jacobian s.t. vt = Jt * v.
  // Where vt is a vector in ℝ². Its first component corresponds to the
  // tangential velocity along the x-axis and its second component corresponds
  // to the out-of-plane (z-axis) tangential velocity. Since the problem is 2D,
  // the second component along the z axis is zero always.
  MatrixX<double> ComputeTangentialJacobian() {
    MatrixX<double> D(2, nv_);
    // vt = vx + w * R = [1, 0, R] * v
    D << 1.0, 0.0, R_,   // Along the x axis
        0.0, 0.0, 0.0;  // Along the z axis out of the plane.
    return D;
  }

  // Sets the TamsiSolver to solve this cylinder case from the
  // input data:
  //   v0: velocity right before impact.
  //   tau: vector of externally applied generalized forces.
  //   mu: friction coefficient between the cylinder and the ground.
  //   height: the initial height the cylinder is dropped from.
  //   dt: time step used by the solver.
  //   num_contacts_multiplier: for testing purposes, we repeat the same contact
  //   point num_contacts_multiplier times. This allow us to test how the solver
  //   does in situations with multiple points of contact even if coincident.
  void SetImpactProblem(const Vector3<double>& v0, const Vector3<double>& tau,
                        double mu, double height, double dt,
                        int num_contacts_multiplier = 1) {
    // Next time step generalized momentum if there are no contact forces.
    p_star_ = M_ * v0 + dt * tau;

    // This problem has a single contact point. We multiply it to emulate
    // a system with multiple points of contact.
    nc_ = num_contacts_multiplier;

    // Friction coefficient for the only contact point in the problem.
    mu_vector_ = VectorX<double>::Constant(nc_, mu);

    // Compute Jacobian matrices.
    Jn_.resize(nc_, nv_);
    Jn_ = RowVector3<double>(0, 1, 0).replicate(num_contacts_multiplier, 1);
    Jt_.resize(2 * nc_, nv_);
    Jt_ = ComputeTangentialJacobian().replicate(num_contacts_multiplier, 1);

    // A very small penetration allowance for practical purposes.
    const double penetration_allowance = 1.0e-6;
    // Initial penetration of O(dt), in tests below we use dt = 1.0e-3.
    const double xini = 1.0e-3;
    // We evenly distribute xinitial among all contact points.
    x0_ = VectorX<double>::Constant(nc_, xini / nc_);
    const double k = m_ * g_ / penetration_allowance;
    stiffness_ = VectorX<double>::Constant(nc_, k);
    fn0_ = stiffness_.array() * x0_.array();
    const double omega = sqrt(stiffness_(0) / m_);
    const double time_scale = 1.0 / omega;
    const double damping_ratio = 1.0;
    const double dissipation =
        damping_ratio * time_scale / penetration_allowance;
    dissipation_ = VectorX<double>::Constant(nc_, dissipation);

    solver_.SetTwoWayCoupledProblemData(&M_, &Jn_, &Jt_, &p_star_, &fn0_,
                                        &stiffness_, &dissipation_,
                                        &mu_vector_);
  }

 protected:
  // For this unit test we have:
  //   R = 1.0
  //   m = 1.0
  //   I = 1.0
  //   h0 = 1 / 2
  //   g = 9.0
  //   mu = 0.1
  // With this set of parameters we have:
  //   vy = -3.0 m/s (velocity at impact).
  //   pn = 3.0 Ns
  //   vx_transition = 0.6 m/s
  // with pn = −m vy = m sqrt(2 g h0) and vx_transition determined from
  // vx_transition =  μ (1 + m R²/I) pn/m as described in the documentation of
  // this test fixture.
  const double m_{1.0};   // Mass of the cylinder, kg.
  const double R_{1.0};   // Radius of the cylinder, m.
  const double g_{9.0};   // Acceleration of gravity, m/s².
  // For a thin cylindrical shell the moment of inertia is I = m R². We use this
  // inertia so that numbers are simpler for debugging purposes
  // (I = 1.0 kg m² in this case).
  const double I_{R_ * R_ * m_};  // kg m².
  const double mu_{0.1};  // Coefficient of friction, dimensionless.

  // Problem sizes.
  const int nv_{3};  // number of generalized velocities.
  int nc_{1};  // number of contact points.

  // Mass matrix.
  MatrixX<double> M_{nv_, nv_};

  // Tangential velocities Jacobian.
  MatrixX<double> Jt_;

  // Normal separation velocities Jacobian.
  MatrixX<double> Jn_;

  VectorX<double> stiffness_;
  VectorX<double> dissipation_;
  VectorX<double> x0_;
  VectorX<double> fn0_;

  // TAMSI solver for this problem.
  TamsiSolver<double> solver_{nv_};

  // Additional solver data that must outlive solver_ during solution.
  VectorX<double> p_star_{nv_};  // Generalized momentum.
  VectorX<double> mu_vector_;  // Friction at each contact point.
};

TEST_F(RollingCylinder, StictionAfterImpact) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.1;  // Friction coefficient.

  // Other than normal contact forces, external forcing for this problem
  // includes gravity.
  const Vector3<double> tau(0.0, -m_ * g_, 0.0);

  // Initial height. We choose it so that vy at impact is exactly 3.0 m/s.
  const double h0 = 0.5;

  // Vertical velocity at the moment of impact.
  const double vy0 = -sqrt(2.0 * g_ * h0);

  // Initial horizontal velocity (recall vx_transition = 0.6 m/s)
  const double vx0 = 0.5;  // m/s.

  // Initial velocity.
  const Vector3<double> v0(vx0, vy0, 0.0);

  // We solve exactly the same problem but repeating the contact point multiple
  // times to emulate multi-point contact.
  for (int num_contacts = 1; num_contacts <= 256; num_contacts *= 4) {
    SetImpactProblem(v0, tau, mu, h0, dt, num_contacts);

    // Verify solver has allocated the proper workspace size.
    EXPECT_GE(TamsiSolverTester::get_capacity(solver_), num_contacts);

    TamsiSolverParameters parameters;  // Default parameters.
    parameters.stiction_tolerance = 1.0e-6;
    solver_.set_solver_parameters(parameters);

    TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
    ASSERT_EQ(info, TamsiSolverResult::kSuccess);

    VectorX<double> tau_f = solver_.get_generalized_friction_forces();

    const auto& stats = solver_.get_iteration_statistics();

    const double vt_tolerance =
        solver_.get_solver_parameters().outer_loop_tolerance *
        solver_.get_solver_parameters().stiction_tolerance;
    EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

    // Friction should only act horizontally.
    EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

    // The moment due to friction Mf should exactly match R * ft.
    EXPECT_NEAR(tau_f(2), R_ * tau_f(0), kTolerance);

    const VectorX<double>& vt = solver_.get_tangential_velocities();
    ASSERT_EQ(vt.size(), 2 * num_contacts);
    for (int ic = 0; ic < num_contacts; ++ic) {
      // There should be no spurious out-of-plane tangential velocity.
      EXPECT_NEAR(vt(2 * ic + 1), 0.0, kTolerance);

      // We expect stiction, to within the stiction tolerance.
      EXPECT_LT(std::abs(vt(2 * ic)), parameters.stiction_tolerance);
    }

    const VectorX<double>& v = solver_.get_generalized_velocities();
    ASSERT_EQ(v.size(), nv_);

    // We expect rolling, i.e. vt = vx + omega * R = 0, to within the stiction
    // tolerance.
    EXPECT_LT(std::abs(v(0) + R_ * v(2)), parameters.stiction_tolerance);

    // Compute the Newton-Raphson Jacobian of the residual J = ∇ᵥR using the
    // solver's internal implementation.
    MatrixX<double> J = TamsiSolverTester::CalcJacobian(solver_, v, dt);

    // Compute the same Newton-Raphson Jacobian of the residual J = ∇ᵥR but with
    // a completely separate implementation using automatic differentiation.
    const double v_stiction = parameters.stiction_tolerance;
    const double epsilon_v = v_stiction * parameters.outer_loop_tolerance;
    MatrixX<double> J_expected = test::CalcTwoWayCoupledJacobianWithAutoDiff(
        M_, Jn_, Jt_, p_star_, x0_, mu_vector_, stiffness_, dissipation_, dt,
        v_stiction, epsilon_v, v);

    // Verify the result.
    const double J_tolerance = 30 * std::numeric_limits<double>::epsilon();
    EXPECT_TRUE(CompareMatrices(J, J_expected, J_tolerance,
                                MatrixCompareType::relative));
  }
}

// Same tests a RollingCylinder::StictionAfterImpact but with a smaller friction
// coefficient of mu = 0.1 and initial horizontal velocity of vx0 = 1.0 m/s,
// which leads to the cylinder to be sliding after impact.
// This is a case for which the initial horizontal velocity is
// vx0 > vx_transition and therefore we expect sliding after impact.
TEST_F(RollingCylinder, SlidingAfterImpact) {
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  const double dt = 1.0e-3;  // time step in seconds.
  const double mu = 0.1;     // Friction coefficient.

  // Other than normal contact forces, external forcing for this problem
  // includes gravity.
  const Vector3<double> tau(0.0, -m_ * g_, 0.0);

  // Initial height. We choose it so that vy at impact is exactly 3.0 m/s.
  const double h0 = 0.5;

  // Vertical velocity at the moment of impact.
  const double vy0 = -sqrt(2.0 * g_ * h0);

  // Initial horizontal velocity (vx_transition = 0.6 m/s).
  const double vx0 = 0.7;  // m/s.

  // Initial velocity.
  const Vector3<double> v0(vx0, vy0, 0.0);

  SetImpactProblem(v0, tau, mu, h0, dt);

  TamsiSolverParameters parameters;  // Default parameters.
  parameters.stiction_tolerance = 1.0e-6;
  solver_.set_solver_parameters(parameters);

  TamsiSolverResult info = solver_.SolveWithGuess(dt, v0);
  ASSERT_EQ(info, TamsiSolverResult::kSuccess);

  VectorX<double> tau_f = solver_.get_generalized_friction_forces();

  const auto& stats = solver_.get_iteration_statistics();

  const double vt_tolerance =
      solver_.get_solver_parameters().outer_loop_tolerance *
          solver_.get_solver_parameters().stiction_tolerance;
  EXPECT_TRUE(stats.vt_residual() < vt_tolerance);

  // Friction should only act horizontally.
  EXPECT_NEAR(tau_f(1), 0.0, kTolerance);

  // The moment due to friction Mf should exactly match R * ft.
  EXPECT_NEAR(tau_f(2), R_ * tau_f(0), kTolerance);

  const VectorX<double>& vt = solver_.get_tangential_velocities();
  ASSERT_EQ(vt.size(), 2 * nc_);

  // There should be no spurious out-of-plane tangential velocity.
  EXPECT_NEAR(vt(1), 0.0, kTolerance);

  // We expect sliding, i.e. a (positive) sliding velocity larger than the
  // stiction tolerance.
  EXPECT_GT(vt(0), parameters.stiction_tolerance);

  const VectorX<double>& v = solver_.get_generalized_velocities();
  ASSERT_EQ(v.size(), nv_);

  // Even though not rolling, the cylinder should be rotating clockwise.
  EXPECT_TRUE(v(2) < 0.0);

  // We expect the solver to update vt accordingly based on v before return.
  EXPECT_NEAR(v(0) + R_ * v(2), vt(0), kTolerance);

  // Compute the Newton-Raphson Jacobian of the (two-way coupled)
  // residual J = ∇ᵥR using the solver's internal implementation.
  MatrixX<double> J =
      TamsiSolverTester::CalcJacobian(solver_, v, dt);

  // Compute the same Newton-Raphson Jacobian of the residual J = ∇ᵥR but with
  // a completely separate implementation using automatic differentiation.
  const double v_stiction = parameters.stiction_tolerance;
  const double epsilon_v = v_stiction * parameters.outer_loop_tolerance;
  MatrixX<double> J_expected = test::CalcTwoWayCoupledJacobianWithAutoDiff(
      M_, Jn_, Jt_, p_star_, x0_, mu_vector_,
      stiffness_, dissipation_, dt, v_stiction, epsilon_v, v);

  // We use a tolerance scaled by the norm and size of the matrix.
  const double J_tolerance =
      J_expected.rows() * J_expected.norm() *
          std::numeric_limits<double>::epsilon();

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      J, J_expected, J_tolerance, MatrixCompareType::absolute));
}

GTEST_TEST(EmptyWorld, Solve) {
  const int nv = 0;
  TamsiSolver<double> solver{nv};

  // (Empty) problem data.
  VectorX<double> p_star, mu_vector, fn0, stiffness, dissipation;
  MatrixX<double> M, Jn, Jt;

  DRAKE_EXPECT_NO_THROW(solver.SetTwoWayCoupledProblemData(
      &M, &Jn, &Jt, &p_star, &fn0, &stiffness, &dissipation, &mu_vector));
}
#endif

}  // namespace
}  // namespace multibody
}  // namespace drake

