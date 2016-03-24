
#include "LCMSystem.h"
#include "RigidBodySystem.h"
#include "LinearSystem.h"
#include "BotVisualizer.h"
#include "drakeAppUtil.h"
#include "lcmtypes/drake/lcmt_driving_control_cmd_t.hpp"

#include "drake/systems/plants/shapes/HeightMapTerrain.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[]) {
  PRINT_FUNCTION_NAME;
  
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " vehicle_urdf [world sdf files ...]"
              << std::endl;
    return 1;
  }

  bool flat_terrain = false;

  // todo: consider moving this logic into the RigidBodySystem class so it can
  // be reused
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;

  auto rigid_body_sys = make_shared<RigidBodySystem>();  
  rigid_body_sys->addRobotFromFile(argv[1], floating_base_type);
  rigid_body_sys->use_multi_contact = false;
  auto const & tree = rigid_body_sys->getRigidBodyTree();
  for (int i = 2; i < argc; i++)
    tree->addRobotFromSDF(argv[i], DrakeJoint::FIXED);  // add environment

  if (flat_terrain) {  // add flat terrain
    double box_width = 10;
    double box_depth = 1;
    double angle = 0.25*M_PI;    
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    Matrix3d m;
    m = AngleAxisd(angle,  Vector3d::UnitY());
    //Vector3d center(0, 0, -1.5);
    T_element_to_link.linear() = m;
    //This makes the impact to happen when the sphere is at z=-0.5 regardless of the angle. Just useful for debugging.
    T_element_to_link.translation() = m*Vector3d(0.0,0.0,-1.0) + Vector3d(0.0,0.0,-0.5); 
    auto& world = tree->bodies[0]; //world is a body with zero mass and zero moment of inertia
    Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;
    world->addVisualElement(DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(RigidBody::CollisionElement(geom, T_element_to_link, world), *world, "terrain");
    tree->updateStaticCollisionElements();
  }else
  {//Add Height map
    double terrain_width = 10;
    double terrain_height = 1.0;
    int terrain_ncells = 16;
    DrakeShapes::FlatTerrain terrain_geom("terrain",Vector2i(terrain_ncells,terrain_ncells),Vector2d(terrain_width, terrain_width));

    Isometry3d T_element_to_link = Isometry3d::Identity();
    //T_element_to_link.translation() << -terrain_width/2.0, -terrain_width/2.0, 0.0;
    T_element_to_link.translation() << 0.0, 0.0, -2.0;

    auto& world = tree->bodies[0]; 

    Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;

    world->addVisualElement(DrakeShapes::VisualElement(terrain_geom, T_element_to_link, color));

    tree->addCollisionElement(RigidBody::CollisionElement(terrain_geom, T_element_to_link, world), *world, "terrain");
    tree->updateStaticCollisionElements();
  }

  //tree->drawKinematicTree("graphiviz_test.dot"); //Convert to png image file: dot -Tpng graphiviz_test.dot -o graphiviz_test.png

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  PRINT_VAR(tree->bodies.size());
  for(auto body: tree->bodies){
    PRINT_VAR(body->linkname);
    PRINT_VAR(body->mass);
    PRINT_VAR(body->hasParent())
    if(body->hasParent()){
      PRINT_VAR(body->parent->linkname);
      PRINT_VAR(body->getJoint().getName());
    }
    std::cout << std::endl;
  }

  PRINT_VAR(getNumInputs(*rigid_body_sys));
  PRINT_VAR(tree->num_positions);
  PRINT_VAR(tree->num_velocities);

  //this replaces the above commented out code with the "auto sys = cascade(vehicle_sys, visualizer);" at the end
  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);
  auto sys = cascade(rigid_body_sys, visualizer);  

  SimulationOptions options = default_simulation_options;
  rigid_body_sys->penetration_stiffness = 5000.0;
  rigid_body_sys->penetration_damping = 0.0; //rigid_body_sys->penetration_stiffness / 10.0;
  //rigid_body_sys->friction_coefficient = 10.0;  // essentially infinite friction. Causes ball to rotate
  rigid_body_sys->friction_coefficient = 0.0;
  options.initial_step_size = 1.0e-3;
  options.timeout_seconds = numeric_limits<double>::infinity();
  options.wait_for_keypress = false;
  options.rk2 = true;

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());  
  x0.head(tree->num_positions) = tree->getZeroConfiguration();
  // todo:  call getInitialState instead?  (but currently, that would require
  // snopt).  needs #1627
  // I'm getting away without it, but might be generating large internal forces
  // initially as the ackerman constraint (hopefully) gets enforced by the
  // stabilization terms.

  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), x0, options);
  //  simulate(*sys,0,std::numeric_limits<double>::infinity(),x0,options);

  return 0;
}
