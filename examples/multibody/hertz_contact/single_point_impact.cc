/// @file
///
/// This demo sets up a passive Quadrotor plant in a world described by the
/// warehouse model. The robot simply passively falls to the floor within the
/// walls of the warehouse, falling from the initial_height command line
/// argument.

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/math/rigid_transform.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/custom_force_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/analysis/simulator_print_stats.h"

namespace drake {
namespace examples {
namespace hertz_contact {
namespace {

using Eigen::Vector3d;
using multibody::BodyIndex;
using multibody::CoulombFriction;
using multibody::MultibodyPlant;
using multibody::Body;
using multibody::RigidBody;
using multibody::UnitInertia;
using multibody::SpatialInertia;
using systems::Context;
using math::RigidTransform;
using multibody::SpatialForce;

DEFINE_double(duration, 0.5, "Total duration of simulation.");

template <typename T>
class HertzSphere final : public multibody::CustomForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HertzSphere)

  HertzSphere(const RigidBody<T>& body, const Vector3<double>& p_BS,
              double radius, double elastic_modulus, double dissipation)
      : multibody::CustomForceElement<T>(body.model_instance()),
        p_BS_(p_BS),
        body_index_(body.index()),
        radius_(radius),
        elastic_modulus_(elastic_modulus),
        dissipation_(dissipation) {}

 protected:
  void AddForceContribution(const systems::Context<T>&,
                            multibody::MultibodyForces<T>* forces) const final {
    const MultibodyPlant<T>& plant = this->GetParentPlant();
    const Body<T>& body = plant.get_body(body_index_);
    SpatialForce<T>& F_B_W = forces->mutable_body_forces()[body.node_index()];
    F_B_W += SpatialForce<T>::Zero();
  }

 private:
  Vector3<double> p_BS_;
  BodyIndex body_index_;
  double radius_;
  double elastic_modulus_;
  double dissipation_;
};

template <typename T>
class BlockWithHertzCorners : public systems::Diagram<T> {
 public:
  BlockWithHertzCorners() {
    this->set_name("Block with Hertz corners");

    systems::DiagramBuilder<T> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
    plant_ = &plant;        

    const double mass = 2.0;
    const double friction = 0.125; // 0.125, 0.225 or 0.325
    const Vector3<double> box_dimensions(0.4, 0.6, 0.8);
    const Vector4<double> blue(0.0, 0.0, 1.0, 1.0);
    const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);

    AddGround(friction, orange);
    box_ = &AddBox("box", box_dimensions, mass, friction, blue);

    plant.Finalize();

    ConnectContactResultsToDrakeVisualizer(&builder, plant);
    geometry::ConnectDrakeVisualizer(&builder, scene_graph);

    DRAKE_DEMAND(plant.num_actuators() == 0);
    DRAKE_DEMAND(plant.num_positions() == 7);

    builder.BuildInto(this);
  }

  const MultibodyPlant<T>& plant() const { return *plant_; }

  const RigidBody<T>& box() const { return *box_; }

  Context<T>& GetPlantContext(Context<T>* context) const {
    return this->GetMutableSubsystemContext(plant(), context);
  }

  void SetBoxPose(const RigidTransform<T>& X_WB, Context<T>* context) const {
    Context<T>& plant_context = GetPlantContext(context);
    plant().SetFreeBodyPose(&plant_context, box(), X_WB);
  }

#if 0
  void SetDefaultState(const systems::Context<T>* context) const override {
    DRAKE_DEMAND(state != nullptr);
    systems::Diagram<T>::SetDefaultState(context, state);
    const systems::Context<T>& plant_context =
        this->GetSubsystemContext(*plant_, context);
    systems::State<T>& plant_state =
        this->GetMutableSubsystemState(*plant_, state);
    const math::RigidTransform<T> X_WB(
        Vector3<T>{0.0, 0.0, FLAGS_initial_height});
    plant_->SetFreeBodyPose(
        plant_context, &plant_state, plant_->GetBodyByName("base_link"), X_WB);
  }
#endif

 private:
  multibody::MultibodyPlant<T>* plant_{nullptr};
  const RigidBody<double>* box_;

  void AddGround(double friction, const Vector4<double>& color) {
    const double Lx = 10;
    const double Ly = Lx;
    const double Lz = 1.0;

    const RigidTransform<double> X_BG(Vector3<double>(0.0, 0.0, -Lz / 2));
    plant_->RegisterVisualGeometry(plant_->world_body(), X_BG,
                                   geometry::Box(Lx, Ly, Lz), "ground_visual",
                                   color);

    plant_->RegisterCollisionGeometry(
        plant_->world_body(), X_BG, geometry::Box(Lx, Ly, Lz),
        "ground_collision", CoulombFriction<double>(friction, friction));
  }

  const RigidBody<double>& AddBox(const std::string& name,
                           const Vector3<double>& block_dimensions,
                           double mass, double friction,
                           const Vector4<double>& color) {
  DRAKE_DEMAND(plant_ != nullptr);

  // Ensure the block's dimensions are mass are positive.
  const double LBx = block_dimensions.x();
  const double LBy = block_dimensions.y();
  const double LBz = block_dimensions.z();

  // Describe body B's mass, center of mass, and inertia properties.
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBcm_B =
      UnitInertia<double>::SolidBox(LBx, LBy, LBz);
  const SpatialInertia<double> M_BBcm_B(mass, p_BoBcm_B, G_BBcm_B);

  // Create a rigid body B with the mass properties of a uniform solid block.
  const RigidBody<double>& box = plant_->AddRigidBody(name, M_BBcm_B);

  // Box's visual.
  // The pose X_BG of block B's geometry frame G is an identity transform.
  const RigidTransform<double> X_BG;   // Identity transform.  
  plant_->RegisterVisualGeometry(box, X_BG,
                                geometry::Box(LBx, LBy, LBz),
                                name + "_visual", color);

  plant_->RegisterCollisionGeometry(box, X_BG, geometry::Box(LBx, LBy, LBz),
                                   name + "_collision",
                                   CoulombFriction<double>(friction, friction));

  const double hertz_radius = 0.1;
  const double hertz_modulus = 10.0e9;    // 10 GPa
  const double hertz_dissipation = 0.26;  // s/m
  HertzSphere<double> HertzModel(
      box, Vector3d(-LBx / 2, -LBy / 2, -LBz / 2), hertz_radius, hertz_modulus,
      hertz_dissipation);
      (void)HertzModel;
#if 0      
  plant_->AddForceElement<HertzSphere>(
      box, Vector3d(-LBx / 2, -LBy / 2, -LBz / 2), hertz_radius, hertz_modulus,
      hertz_dissipation);
#endif


  // Box's collision geometry is a solid box.
#if 0  
  if (only_sphere_collision) {
    const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
    const Vector4<double> red_50(1.0, 0.0, 0.0, 0.5);
    const double radius_x = LBx / FLAGS_num_spheres / 2.0;
    const double radius_y = LBy / FLAGS_num_spheres / 2.0;
    const double radius_z = LBz / FLAGS_num_spheres / 2.0;
    int i = 0;
    std::vector<double> x_range, y_range, z_range;
    double dx = 2 * radius_x;
    double dy = 2 * radius_y;
    double dz = 2 * radius_z;
    for (int j = 0; j< FLAGS_num_spheres;++j) {
      x_range.push_back(-LBx/2 + radius_x + j*dx);
      y_range.push_back(-LBy/2 + radius_y + j*dy);
      z_range.push_back(-LBz/2 + radius_z + j*dz);
    }
    for (double x_sign : x_range) {
      for (double y_sign : y_range) {
        for (double z_sign : z_range) {
          const std::string name_spherei =
              name + "_sphere" + std::to_string(++i) + "_collision";
          const double x = x_sign;
          const double y = y_sign;
          const double z = z_sign;
          const Vector3<double> p_BoSpherei_B(x, y, z);
          const RigidTransform<double> X_BSpherei(p_BoSpherei_B);
          geometry::Sphere shape(radius_x);
          // Ellipsoid might not be accurate. From console [warning]:
          // "Ellipsoid is primarily for ComputeContactSurfaces in hydroelastic
          // contact model. The accuracy of other collision queries and signed
          // distance queries are not guaranteed."
          // geometry::Ellipsoid shape(radius_x, radius_y, radius_z);
          plant->RegisterCollisionGeometry(
              box, X_BSpherei, shape, name_spherei,
              CoulombFriction<double>(friction, friction));
          plant->RegisterVisualGeometry(
              box, X_BSpherei, shape, name_spherei, red);
        }  // z
      }  // y
    }  // x
  } else {
#endif      
  
  return box;                                     
}


};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const BlockWithHertzCorners<double> model;
  auto context = model.CreateDefaultContext();
  const RigidTransform<double> X_WB(Vector3<double>(0, 0, 0.5));
  model.SetBoxPose(X_WB, context.get());

  //Context<double>& plant_context = model.GetPlantContext(context.get());

  auto simulator = MakeSimulatorFromGflags(model, std::move(context));
  //auto* context = simulator->get_mutable_context();
  simulator->AdvanceTo(FLAGS_duration);

  PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // namespace hertz_contact
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::hertz_contact::do_main(argc, argv);
}
