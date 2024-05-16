#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/discrete_contact_data.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/multibody/plant/dummy_physical_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/iiwa7_model.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using multibody::RevoluteSpring;
using multibody::test::RobotModel;
using symbolic::Expression;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::System;

namespace multibody {
namespace {

template <typename T>
class MultibodyPlantDefaultScalarsTest : public ::testing::Test {
 public:
  MultibodyPlantDefaultScalarsTest() = default;
};

TYPED_TEST_SUITE_P(MultibodyPlantDefaultScalarsTest);

// We test that we can scalar convert a plant containing a revolute joint and
// spring. In particular, we verify that the spring element correctly references
// the joint both before and after scalar conversion.
TYPED_TEST_P(MultibodyPlantDefaultScalarsTest, RevoluteJointAndSpring) {
  using U = TypeParam;

  MultibodyPlant<double> plant(0.0);
  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const RigidBody<double>& body =
      plant.AddRigidBody("Body", SpatialInertia<double>::MakeUnitary());
  const RevoluteJoint<double>& pin = plant.AddJoint<RevoluteJoint>(
      "Pin", plant.world_body(), std::nullopt, body, std::nullopt,
      Vector3<double>::UnitZ());
  const auto& spring = plant.AddForceElement<RevoluteSpring>(pin, 0, 1500);

  // We verify the correct reference to the pin joint before conversion.
  EXPECT_EQ(&pin, &spring.joint());

  // We are done defining the model.
  plant.Finalize();

  // Sanity check for the model's size.
  DRAKE_DEMAND(plant.num_velocities() == 1);
  DRAKE_DEMAND(plant.num_positions() == 1);
  // The plant has a UniformGravityFieldElement by default plus the
  // RevoluteSpring.
  DRAKE_DEMAND(plant.num_force_elements() == 2);

  std::unique_ptr<MultibodyPlant<U>> plant_u;
  EXPECT_NO_THROW(plant_u = System<double>::ToScalarType<U>(plant));
  EXPECT_EQ(plant.num_velocities(), plant_u->num_velocities());
  EXPECT_EQ(plant.num_positions(), plant_u->num_positions());
  EXPECT_EQ(plant.num_force_elements(), plant_u->num_force_elements());

  // We verify the correct reference to the pin joint after conversion.
  const auto& pin_u =
      plant_u->template GetJointByName<RevoluteJoint>(pin.name());
  const auto& spring_u =
      plant_u->template GetForceElement<RevoluteSpring>(spring.index());

  // Verify correct cross-referencing in the scalar converted model.
  EXPECT_EQ(&pin_u, &spring_u.joint());
}

// Verifies that two MultibodyPlants have the same port indices.
template <typename T, typename U>
void CompareMultibodyPlantPortIndices(const MultibodyPlant<T>& plant_t,
                                      const MultibodyPlant<U>& plant_u) {
  // Check input ports.
  // (Except actuation input ports because there is no actuation source.)
  EXPECT_EQ(plant_t.get_applied_generalized_force_input_port().get_index(),
            plant_u.get_applied_generalized_force_input_port().get_index());
  EXPECT_EQ(plant_t.get_applied_spatial_force_input_port().get_index(),
            plant_u.get_applied_spatial_force_input_port().get_index());
  EXPECT_EQ(plant_t.get_geometry_query_input_port().get_index(),
            plant_u.get_geometry_query_input_port().get_index());
  // Check output ports.
  EXPECT_EQ(plant_t.get_body_poses_output_port().get_index(),
            plant_u.get_body_poses_output_port().get_index());
  EXPECT_EQ(plant_t.get_body_spatial_velocities_output_port().get_index(),
            plant_u.get_body_spatial_velocities_output_port().get_index());
  EXPECT_EQ(plant_t.get_body_spatial_accelerations_output_port().get_index(),
            plant_u.get_body_spatial_accelerations_output_port().get_index());
  EXPECT_EQ(plant_t.get_state_output_port().get_index(),
            plant_u.get_state_output_port().get_index());
  EXPECT_EQ(plant_t.get_generalized_acceleration_output_port().get_index(),
            plant_u.get_generalized_acceleration_output_port().get_index());
  EXPECT_EQ(plant_t.get_reaction_forces_output_port().get_index(),
            plant_u.get_reaction_forces_output_port().get_index());
  EXPECT_EQ(plant_t.get_contact_results_output_port().get_index(),
            plant_u.get_contact_results_output_port().get_index());
  EXPECT_EQ(plant_t.get_geometry_poses_output_port().get_index(),
            plant_u.get_geometry_poses_output_port().get_index());
  EXPECT_EQ(
      plant_t.get_state_output_port(default_model_instance()).get_index(),
      plant_u.get_state_output_port(default_model_instance()).get_index());
  EXPECT_EQ(
      plant_t.get_generalized_acceleration_output_port(default_model_instance())
          .get_index(),
      plant_u.get_generalized_acceleration_output_port(default_model_instance())
          .get_index());
  EXPECT_EQ(
      plant_t
          .get_generalized_contact_forces_output_port(default_model_instance())
          .get_index(),
      plant_u
          .get_generalized_contact_forces_output_port(default_model_instance())
          .get_index());
}

// This test verifies that the port indices of the input/output ports of
// MultibodyPlant remain the same after scalar conversion.
TYPED_TEST_P(MultibodyPlantDefaultScalarsTest, PortIndexOrdering) {
  using U = TypeParam;

  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  std::unique_ptr<Diagram<U>> diagram_u =
      System<double>::ToScalarType<U>(*diagram);
  const auto& plant_u =
      dynamic_cast<const MultibodyPlant<U>&>(*diagram_u->GetSystems().at(0));

  CompareMultibodyPlantPortIndices(plant, plant_u);
}

// Verifies that we can AddMultibodyPlantSceneGraph, without any conversion.
TYPED_TEST_P(MultibodyPlantDefaultScalarsTest, DirectlyAdded) {
  using U = TypeParam;
  systems::DiagramBuilder<U> builder;
  MultibodyPlant<U>& plant = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant.Finalize();
  std::unique_ptr<Diagram<U>> diagram = builder.Build();
  diagram->CreateDefaultContext();
}

REGISTER_TYPED_TEST_SUITE_P(MultibodyPlantDefaultScalarsTest,
                            RevoluteJointAndSpring, PortIndexOrdering,
                            DirectlyAdded);

using NonDoubleScalarTypes = ::testing::Types<AutoDiffXd, Expression>;
INSTANTIATE_TYPED_TEST_SUITE_P(My, MultibodyPlantDefaultScalarsTest,
                               NonDoubleScalarTypes);

// A simple concrete DiscreteUpdateManager that does not support scalar
// conversion to either AutoDiffXd or Expression.
template <typename T>
class DoubleOnlyDiscreteUpdateManager final
    : public multibody::internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoubleOnlyDiscreteUpdateManager);
  DoubleOnlyDiscreteUpdateManager() = default;
  ~DoubleOnlyDiscreteUpdateManager() final = default;

  bool is_cloneable_to_double() const final { return true; }

 private:
  std::unique_ptr<multibody::internal::DiscreteUpdateManager<double>>
  CloneToDouble() const final {
    auto clone = std::make_unique<DoubleOnlyDiscreteUpdateManager<double>>();
    return clone;
  }

  void DoCalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const final {}

  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      internal::AccelerationKinematicsCache<T>*) const final {}

  void DoCalcDiscreteValues(const systems::Context<T>&,
                            systems::DiscreteValues<T>*) const final {}

  void DoCalcDiscreteUpdateMultibodyForces(
      const systems::Context<T>& context,
      MultibodyForces<T>* forces) const final {}

  void DoCalcActuation(const systems::Context<T>&, VectorX<T>*) const final {}
};

// This test verifies that adding external components that do not support some
// scalar types removes MultibodyPlant's ability to scalar convert to those
// scalar types.
GTEST_TEST(ScalarConversionTest, ExternalComponent) {
  MultibodyPlant<double> plant(0.1);
  std::unique_ptr<PhysicalModel<double>> dummy_physical_model =
      std::make_unique<internal::DummyPhysicalModel<double>>(&plant);
  // The dummy model supports all scalar types.
  EXPECT_TRUE(dummy_physical_model->is_cloneable_to_double());
  EXPECT_TRUE(dummy_physical_model->is_cloneable_to_autodiff());
  EXPECT_TRUE(dummy_physical_model->is_cloneable_to_symbolic());
  plant.AddPhysicalModel(std::move(dummy_physical_model));
  plant.Finalize();

  // double -> AutoDiffXd
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff;
  EXPECT_NO_THROW(plant_autodiff = System<double>::ToAutoDiffXd(plant));
  // AutoDiffXd -> double
  EXPECT_NO_THROW(plant_autodiff->ToScalarType<double>());
  // double -> Expression
  std::unique_ptr<MultibodyPlant<Expression>> plant_double_to_symbolic;
  EXPECT_NO_THROW(plant_double_to_symbolic = System<double>::ToSymbolic(plant));
  // double -> Expression
  std::unique_ptr<MultibodyPlant<Expression>> plant_autodiff_to_symbolic;
  EXPECT_NO_THROW(plant_autodiff_to_symbolic =
                      System<AutoDiffXd>::ToSymbolic(*plant_autodiff));

  // Verify that adding a component that doesn't allow scalar conversion to
  // autodiff does not prevent scalar conversion to double.
  auto discrete_update_manager =
      std::make_unique<DoubleOnlyDiscreteUpdateManager<AutoDiffXd>>();
  EXPECT_TRUE(discrete_update_manager->is_cloneable_to_double());
  EXPECT_FALSE(discrete_update_manager->is_cloneable_to_autodiff());
  EXPECT_FALSE(discrete_update_manager->is_cloneable_to_symbolic());
  plant_autodiff->SetDiscreteUpdateManager(std::move(discrete_update_manager));
  EXPECT_NO_THROW(plant_autodiff->ToScalarType<double>());
}

}  // namespace

class MultibodyPlantTester {
 public:
  template <typename T>
  static std::map<MultibodyConstraintId, internal::CouplerConstraintSpec>&
  get_mutable_specs(MultibodyPlant<T>* plant) {
    return plant->coupler_constraints_specs_;
  }
};

namespace {

// Verify that constraint specs survive scalar conversions. Here we only test
// that the number of constraints before and aftter scalar conversion are the
// same. The correctness of the scalar copying semantics for the constraints are
// tested in their own unit tests.
GTEST_TEST(ScalarConversionTest, CouplerConstraintSpec) {
  MultibodyPlant<double> plant_double(0.1);

  const JointIndex j0(3);
  const JointIndex j1(5);
  constexpr double kGearRatio = 1.2;
  constexpr double kOffset = 0.3;
  const internal::CouplerConstraintSpec reference_spec{j0, j1, kGearRatio,
                                                       kOffset};

  // Directly add dummy constraint specs through the tester so that we don't
  // need to actually add any joints.
  MultibodyPlantTester::get_mutable_specs(
      &plant_double)[MultibodyConstraintId::get_new_id()] = reference_spec;
  plant_double.Finalize();
  // double -> AutoDiffXd.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_double_to_autodiff =
      System<double>::ToAutoDiffXd(plant_double);
  EXPECT_EQ(plant_double_to_autodiff->num_constraints(),
            plant_double.num_constraints());
  // AutoDiffXd -> double.
  std::unique_ptr<MultibodyPlant<double>> plant_autodiff_to_double =
      System<AutoDiffXd>::ToScalarType<double>(*plant_double_to_autodiff);
  EXPECT_EQ(plant_autodiff_to_double->num_constraints(),
            plant_double.num_constraints());
}

struct IiwaRobotTestConfig {
  // This is a gtest test suffix; no underscores or spaces at the start.
  std::string description;

  DiscreteContactApproximation contact_approximation{
      DiscreteContactApproximation::kSimilar};

  bool with_contact{true};

  ContactModel contact_model{ContactModel::kHydroelasticWithFallback};
};

// This provides the suffix for each test parameter: the test config
// description.
std::ostream& operator<<(std::ostream& out, const IiwaRobotTestConfig& c) {
  out << c.description;
  return out;
}

template <typename T>
class IiwaRobotTest : public ::testing::TestWithParam<IiwaRobotTestConfig> {
 public:
  void SetUp() override {
    const IiwaRobotTestConfig& config = GetParam();
    auto model_double = std::make_unique<RobotModel<double>>(
        config.contact_approximation, config.contact_model);
    if (config.with_contact) {
      model_double->SetState(
          RobotModel<double>::RobotStateWithOneContactStiction());
    } else {
      model_double->SetState(VectorX<double>::Zero(14));
    }
    if constexpr (std::is_same_v<T, double>) {
      model_ = std::move(model_double);
    } else {
      model_ = model_double->ToScalarType<T>();
    }
  }

 protected:
  std::unique_ptr<RobotModel<T>> model_;
};

using IiwaRobotTestDouble = IiwaRobotTest<double>;
using IiwaRobotTestAutoDiff = IiwaRobotTest<AutoDiffXd>;
using IiwaRobotTestExpression = IiwaRobotTest<symbolic::Expression>;

std::vector<IiwaRobotTestConfig> MakeSupportMatrixTestCases() {
  return std::vector<IiwaRobotTestConfig>{
      // SAP
      {
          .description = "Sap_HydroWithFallback_NoContact",
          .contact_approximation = DiscreteContactApproximation::kSimilar,
          .with_contact = false,
          .contact_model = ContactModel::kHydroelasticWithFallback,
      },
      {
          .description = "Sap_HydroWithFallback_WithContact",
          .contact_approximation = DiscreteContactApproximation::kSimilar,
          .with_contact = true,
          .contact_model = ContactModel::kHydroelasticWithFallback,
      },
      {
          .description = "Sap_Point_WithContact",
          .contact_approximation = DiscreteContactApproximation::kSimilar,
          .with_contact = true,
          .contact_model = ContactModel::kPoint,
      },
      // TAMSI
      {
          .description = "Tamsi_HydroWithFallback_NoContact",
          .contact_approximation = DiscreteContactApproximation::kTamsi,
          .with_contact = false,
          .contact_model = ContactModel::kHydroelasticWithFallback,
      },
      {
          .description = "Tamsi_HydroWithFallback_WithContact",
          .contact_approximation = DiscreteContactApproximation::kTamsi,
          .with_contact = true,
          .contact_model = ContactModel::kHydroelasticWithFallback,
      },
      {
          .description = "Tamsi_Point_WithContact",
          .contact_approximation = DiscreteContactApproximation::kTamsi,
          .with_contact = true,
          .contact_model = ContactModel::kPoint,
      },
  };
}

INSTANTIATE_TEST_SUITE_P(SupportMatrixTests, IiwaRobotTestDouble,
                         testing::ValuesIn(MakeSupportMatrixTestCases()),
                         testing::PrintToStringParamName());

TEST_P(IiwaRobotTestDouble, ForcedUpdate) {
  const auto& diagram = model_->diagram();
  auto updates = diagram.AllocateDiscreteVariables();
  EXPECT_NO_THROW(diagram.CalcForcedDiscreteVariableUpdate(model_->context(),
                                                           updates.get()));
}

INSTANTIATE_TEST_SUITE_P(SupportMatrixTests, IiwaRobotTestAutoDiff,
                         testing::ValuesIn(MakeSupportMatrixTestCases()),
                         testing::PrintToStringParamName());

TEST_P(IiwaRobotTestAutoDiff, ForcedUpdate) {
  const auto& diagram = model_->diagram();
  auto updates = diagram.AllocateDiscreteVariables();
  EXPECT_NO_THROW(diagram.CalcForcedDiscreteVariableUpdate(model_->context(),
                                                           updates.get()));
}

INSTANTIATE_TEST_SUITE_P(SupportMatrixTests, IiwaRobotTestExpression,
                         testing::ValuesIn(MakeSupportMatrixTestCases()),
                         testing::PrintToStringParamName());

TEST_P(IiwaRobotTestExpression, ForcedUpdate) {
  const auto& diagram = model_->diagram();
  auto updates = diagram.AllocateDiscreteVariables();

  // N.B. Notice that even when most MultibodyPlant's implementation checks for
  // zero contact pairs to allow the computation to continue, the proximity
  // engine still throws an exception even if there is no contact. Therefore the
  // cases with no contact are not supported per-proximity constraints.

  if (model_->plant().get_contact_model() ==
      ContactModel::kHydroelasticWithFallback) {
    // QueryObject<T>::ComputeContactSurfaces(), and hence
    // QueryObject<T>::ComputeContactSurfacesWithFallback() does not exist for T
    // = Expression. Therefore
    // MultibodyPlant<Expression>::CalcHydroelasticWithFallback() is the one
    // that generates this exception.
    DRAKE_EXPECT_THROWS_MESSAGE(
        diagram.CalcForcedDiscreteVariableUpdate(model_->context(),
                                                 updates.get()),
        "MultibodyPlant<T>::CalcHydroelasticWithFallback\\(\\): This method "
        "doesn't support T = symbolic::Expression.");
  } else if (model_->plant().get_contact_model() == ContactModel::kPoint) {
    // This exception comes from deep within the proximity engine. Even though
    // we can call QueryObject<T>::ComputePointPairPenetration() with T =
    // Expression, this method's support table throws for every shape pair
    // combination.
    DRAKE_EXPECT_THROWS_MESSAGE(
        diagram.CalcForcedDiscreteVariableUpdate(model_->context(),
                                                 updates.get()),
        "Penetration queries between shapes .* are not supported for scalar "
        "type drake::symbolic::Expression. .*");
  } else {
    throw std::runtime_error("Update unit test to verify this case.");
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
