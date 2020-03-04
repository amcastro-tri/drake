#include <limits>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {

using multibody::Parser;
using systems::Context;

namespace multibody {
namespace {

// We verify the computation of the mass matrix by comparing two significantly
// different implementations: 
//   - CalcMassMatrix(): uses the Composite Body Algorithm.
//   - CalcMassMatrixViaInverseDynamics(): uses inverse dynamics to compute each
//     column of the mass matrix at a time.
GTEST_TEST(MultibodyPlantMassMatrix, Atlas) {
  const std::string atlas_path =
     FindResourceOrThrow("drake/examples/atlas/urdf/atlas_convex_hull.urdf");

  // FindResourceOrThrow("drake/examples/multibody/cart_pole/cart_pole.sdf");
  // FindResourceOrThrow("drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  // FindResourceOrThrow("drake/examples/atlas/urdf/atlas_convex_hull.urdf");

  // So far these models work:
  //  - drake/examples/multibody/cart_pole/cart_pole.sdf  
  //  - drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf  

  //const std::string mug_sdf_path =
  //    FindResourceOrThrow("drake/examples/simple_gripper/simple_mug.sdf");

  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  const ModelInstanceIndex robot1_model =
      parser.AddModelFromFile(atlas_path, "Robot1");
  //const Body<double>& base1 = plant.GetBodyByName("Cart", robot1_model);
  (void)robot1_model;
  //(void)base1;

//  const ModelInstanceIndex robot2_model =
//      parser.AddModelFromFile(atlas_path, "Robot2");
//  const Body<double>& base2 = plant.GetBodyByName("Cart", robot2_model);

  //const ModelInstanceIndex mug_model = parser.AddModelFromFile(mug_sdf_path);
  //const Body<double>& mug = plant.GetBodyByName("main_body", mug_model);  

  //plant.WeldFrames(mug.body_frame(), base1.body_frame());
  //plant.WeldFrames(mug.body_frame(), base2.body_frame());

  plant.Finalize();

  PRINT_VAR(plant.num_velocities());

  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  // Compute mass matrix via the Composite Body Algorithm.  
  MatrixX<double> Hcba(plant.num_velocities(), plant.num_velocities());
  plant.CalcMassMatrix(*context, &Hcba);

  // Compute mass matrix using inverse dynamics for each column.
  MatrixX<double> Hid(plant.num_velocities(), plant.num_velocities());
  plant.CalcMassMatrixViaInverseDynamics(*context, &Hid);

  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(Hcba,
                              Hid, kTolerance,
                              MatrixCompareType::relative));

  PRINT_VARn(Hid);
  PRINT_VARn(Hcba);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
