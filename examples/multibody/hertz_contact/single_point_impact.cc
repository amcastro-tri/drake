/// @file
///
/// This demo sets up a passive Quadrotor plant in a world described by the
/// warehouse model. The robot simply passively falls to the floor within the
/// walls of the warehouse, falling from the initial_height command line
/// argument.

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
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
using Eigen::Vector4d;
using Eigen::AngleAxisd;
using multibody::BodyIndex;
using multibody::ModelInstanceIndex;
using multibody::CoulombFriction;
using multibody::MultibodyPlant;
using multibody::Body;
using multibody::RigidBody;
using multibody::UnitInertia;
using math::RollPitchYawd;
using math::RotationMatrix;
using math::RotationMatrixd;
using multibody::SpatialInertia;
using multibody::SpatialVelocity;
using systems::Context;
using math::RigidTransform;
using multibody::SpatialForce;

DEFINE_double(duration, 0.5, "Total duration of simulation.");
DEFINE_double(roll, 0.0, "Roll angle.");
DEFINE_double(pitch, 0.0, "Pitch angle.");
DEFINE_double(yaw, 0.0, "Yaw angle.");
DEFINE_double(friction, 0.125, "Friction. From paper: 0.125, 0.225 or 0.325.");

const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
const Vector4<double> blue(0.0, 0.0, 1.0, 1.0);
const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);    

template <typename T>
class HertzSphere final : public multibody::CustomForceElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HertzSphere)

  HertzSphere(const std::string& name, const RigidBody<T>& body,
              const Vector3<double>& p_BS, double radius,
              double elastic_modulus, double dissipation, double mu, double vs)
      : multibody::CustomForceElement<T>(body.model_instance()),
        name_(name),
        p_BS_(p_BS),
        body_index_(body.index()),
        radius_(radius),
        elastic_modulus_(elastic_modulus),
        dissipation_(dissipation),
        mu_(mu),
        vs_(vs) {}

  const Body<T>& body() const {
    const MultibodyPlant<T>& plant = this->GetParentPlant();
    return plant.get_body(body_index_);
  }

  void RegisterVisualGeometry(const Vector4d& color,
                              MultibodyPlant<T>* plant) const {
    const MultibodyPlant<T>* this_plant = &this->GetParentPlant();
    DRAKE_DEMAND(this_plant == plant);

    plant->RegisterVisualGeometry(body(), RigidTransform<double>(p_BS_),
                                  geometry::Sphere(radius_), name_ + "_visual",
                                  color);
  }

 protected:
  void AddForceContribution(const systems::Context<T>& context,
                            multibody::MultibodyForces<T>* forces) const final {
    const MultibodyPlant<T>& plant = this->GetParentPlant();
    const Body<T>& body = plant.get_body(body_index_);
    SpatialForce<T>& F_B_W = forces->mutable_body_forces()[body.node_index()];
    F_B_W += CalcSpatialForceOnBody(context);
  }

  std::unique_ptr<multibody::CustomForceElement<symbolic::Expression>>
  ToSymbolic() const final {
    std::unique_ptr<HertzSphere<symbolic::Expression>> clone(
        new HertzSphere<symbolic::Expression>(name_, this->model_instance(),
                                              body_index_, p_BS_, radius_,
                                              elastic_modulus_, dissipation_, mu_, vs_));
    return clone;
  }

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class HertzSphere;

  HertzSphere(const std::string& name, ModelInstanceIndex model_instance,
              BodyIndex body_index, const Vector3<double>& p_BS, double radius,
              double elastic_modulus, double dissipation, double mu, double vs)
      : multibody::CustomForceElement<T>(model_instance),
        name_(name),
        p_BS_(p_BS),
        body_index_(body_index),
        radius_(radius),
        elastic_modulus_(elastic_modulus),
        dissipation_(dissipation),
        mu_(mu),
        vs_(vs) {}

  static T step5(const T& x) {
    DRAKE_ASSERT(0 <= x && x <= 1);
    const T x3 = x * x * x;
    return x3 * (10 + x * (6 * x - 15));  // 10x³ - 15x⁴ + 6x⁵
  }

  static T CalcRegularizedFriction(const T& v, double vs, double mu) {
    DRAKE_DEMAND(v >= 0);
    if (v < vs) {
      return mu * step5(v / vs);
    }
    return mu;
  }

  SpatialForce<T> CalcSpatialForceOnBody(
      const systems::Context<T>& context) const {
    const MultibodyPlant<T>& plant = this->GetParentPlant();
    //body();
    const auto& X_WB = plant.EvalBodyPoseInWorld(context, body());
    const RigidTransform<T> X_BS(p_BS_);
    const RigidTransform<T> X_WS = X_WB * X_BS;

    // Lowest (in z) contact point C on the sphere.
    Vector3<T> p_WC = X_WS.translation();
    p_WC[2] -= radius_;
    const T z = p_WC[2];

    // Obtain p_BC_W
    // p_WC = p_WB + p_BC_W
    const Vector3<T>& p_WB = X_WB.translation();    
    const Vector3<T> p_BoCo_W =  p_WC - p_WB;
    
    if (z >=0) return SpatialForce<T>::Zero();

    // Penetration.
    const T x = -z;

    const auto& V_WB = plant.EvalBodySpatialVelocityInWorld(context, body());
    //const RotationMatrix<T>& R_WB = X_WB.rotation();
    //const Vector3<T> p_BS_W = R_WB * p_BS_.cast<T>();
    const SpatialVelocity<T> V_WC = V_WB.Shift(p_BoCo_W);
    const Vector3<T>& v_WC = V_WC.translational();
    const T& xdot = -v_WC[2];

    // Normal forces
    using std::sqrt;
    const T fHz = 4. / 3. * elastic_modulus_ * sqrt(radius_ * x * x * x);

    using std::max;
    const T fHC = fHz * max(0.0, 1 + 1.5 * dissipation_ * xdot);
    const Vector3<T> fn(0, 0, fHC);

    // Friction
    const Vector3<T> vt(v_WC[0], v_WC[1], 0.0);
    const T slip2 = vt.squaredNorm();
    const T slip = sqrt(slip2);
    const T mu = CalcRegularizedFriction(slip, vs_, mu_);
    const double ev = vs_ / 1000.0;
    const T soft_slip = sqrt(slip2 + ev * ev);
    const Vector3<T> ft = -vt / soft_slip * mu * fHC;

    SpatialForce<T> F_BCo_W(Vector3<T>::Zero(), ft + fn);

    // p_WC = p_WB + p_BC_W
    const Vector3<T> p_CoBo_W = -p_BoCo_W;

    const SpatialForce<T> F_BBo_W = F_BCo_W.Shift(p_CoBo_W);

    return F_BBo_W;
  }

  std::string name_;
  Vector3<double> p_BS_;
  BodyIndex body_index_;
  double radius_;
  double elastic_modulus_;
  double dissipation_;
  double mu_;
  double vs_;
};

template <typename T>
class BlockWithHertzCorners : public systems::Diagram<T> {
 public:
  struct Parameters {
    double mass{2.0};
    double friction{0.125};  // 0.125, 0.225 or 0.325
    double transition_speed{0.01};  // m/s
    Vector3<double> box_dimensions{0.4, 0.6, 0.8};
    Vector4<double> box_color{blue};
    Vector4<double> ground_color{orange};
    Vector4<double> sphere_color{red};

    // Hertz model parameters.
    double hertz_radius{0.1};
    double hertz_modulus{10.0e9};    // 10 GPa
    double hertz_dissipation{0.26};  // s/m

    // Initial conditions.
    Vector3d rpy_WB_init{M_PI / 4.0, M_PI / 6.0, 0.0};
    SpatialVelocity<double> V_WB_init{Vector3d::Zero(),
                                      Vector3d{-5.0, 0.0, -5.74}};
  };

  BlockWithHertzCorners(const Parameters& parameters)
      : parameters_(parameters) {
    this->set_name("Block with Hertz corners");

    systems::DiagramBuilder<T> builder;
    auto [plant, scene_graph] =
        multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
    plant_ = &plant;

    AddGround( parameters_.friction, parameters_.ground_color);
    box_ = &AddBox("box", parameters_.box_dimensions, parameters_.mass,
                   parameters_.friction, parameters_.box_color);

    plant.Finalize();

    ConnectContactResultsToDrakeVisualizer(&builder, plant);
    geometry::ConnectDrakeVisualizer(&builder, scene_graph);

    DRAKE_DEMAND(plant.num_actuators() == 0);
    DRAKE_DEMAND(plant.num_positions() == 7);

    builder.BuildInto(this);
  }

  const MultibodyPlant<T>& plant() const { return *plant_; }

  const RigidBody<T>& box() const { return *box_; }

  const Parameters& parameters() const { return parameters_; }

  Context<T>& GetPlantContext(Context<T>* context) const {
    return this->GetMutableSubsystemContext(plant(), context);
  }

  void SetBoxPose(const RigidTransform<T>& X_WB, Context<T>* context) const {
    Context<T>& plant_context = GetPlantContext(context);
    plant().SetFreeBodyPose(&plant_context, box(), X_WB);
  }

  void SetBoxSpatialVelocity(const SpatialVelocity<T>& V_WB,
                             Context<T>* context) const {
    Context<T>& plant_context = GetPlantContext(context);
    plant().SetFreeBodySpatialVelocity(&plant_context, box(), V_WB);
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
  Parameters parameters_;
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

  void AddHertzSphere(const RigidBody<T>& body, const std::string& name,
                      const Vector3d& p_BS) {
    const auto& hertz_sphere = plant_->template AddForceElement<HertzSphere>(
        name, body, p_BS, parameters_.hertz_radius, parameters_.hertz_modulus,
        parameters_.hertz_dissipation, parameters_.friction,
        parameters_.transition_speed);
    hertz_sphere.RegisterVisualGeometry(parameters_.sphere_color, plant_);
  }

  const RigidBody<T>& AddBox(const std::string& name,
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

  // -z
  AddHertzSphere(box, "sphere_mxmymz", Vector3d(-LBx / 2, -LBy / 2, -LBz / 2));
  AddHertzSphere(box, "sphere_mxpymz", Vector3d(-LBx / 2, +LBy / 2, -LBz / 2));
  AddHertzSphere(box, "sphere_pxmymz", Vector3d(+LBx / 2, -LBy / 2, -LBz / 2));
  AddHertzSphere(box, "sphere_pxpymz", Vector3d(+LBx / 2, +LBy / 2, -LBz / 2));

  // +z
  AddHertzSphere(box, "sphere_mxmypz", Vector3d(-LBx / 2, -LBy / 2, +LBz / 2));
  AddHertzSphere(box, "sphere_mxpypz", Vector3d(-LBx / 2, +LBy / 2, +LBz / 2));
  AddHertzSphere(box, "sphere_pxmypz", Vector3d(+LBx / 2, -LBy / 2, +LBz / 2));
  AddHertzSphere(box, "sphere_pxpypz", Vector3d(+LBx / 2, +LBy / 2, +LBz / 2));

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

  BlockWithHertzCorners<double>::Parameters params;
  params.friction = FLAGS_friction;  

  const BlockWithHertzCorners<double> model(params);
  auto context = model.CreateDefaultContext();  

  // Initial orientation R_WB.
  // Obtained by first rotating pi/4 about the body's x followed by rotating
  // pi/6 about the new y body axis.
  RotationMatrixd Rx = RotationMatrixd::MakeXRotation(params.rpy_WB_init(0));
  const Vector3d yhat_new = Rx.matrix().col(1);
  RotationMatrixd Ry(AngleAxisd(params.rpy_WB_init(1), yhat_new));
  const RotationMatrixd R_WB = Ry * Rx;  

  // We want sphere at +x, -y, -z to be in contact with the groud.
  const Vector3d size = model.parameters().box_dimensions;
  const Vector3d p_BS(+size.x() / 2.0, -size.y() / 2.0, -size.z() / 2.0);

  // Position of contact point C from S, measured in world.
  const Vector3d p_SC_W(0.0, 0.0, -model.parameters().hertz_radius);

  // p_WC_W = p_WB_W + p_BS_W + p_SC_W = 0.
  // Then p_WB_W = -(p_BS_W + p_SC_W)
  const Vector3d p_BS_W = R_WB * p_BS;
  const Vector3d p_WB = -(p_BS_W + p_SC_W);

  const RigidTransform<double> X_WB(R_WB, p_WB);
  model.SetBoxPose(X_WB, context.get());
  model.SetBoxSpatialVelocity(params.V_WB_init, context.get());

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
