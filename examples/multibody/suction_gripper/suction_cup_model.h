#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace examples {
namespace multibody {
namespace suction_gripper {

#if 0
/** Model for a suction cup.

 Assumptions:
  - Point model: The cup is modeled as a single point C on a body B.
  - Massless cup: The mass of the cup itself is neglected. If the mass of the
    cup must be accounted for, the modeler can lump its mass to the body B on
    which it attaches.
  - Ad-hoc force law: The force model is ad-hoc, not based on physics nor
    available experimental data. However, the model does encode the main
    attributes that we would expect. That is, the force is a decaying function
    of distance, with symmetry of revolution around the cup's axis and it goes
    to zero as points move behind the plane defined by the cup's normal.
   - Geometry: All geometry of the cup is lumped into its area. The model does
     not include deformations nor it predicts how the cup might conform to
     external objects.

 To be more specific, consider an object B in the neighborhood of the cup. Let ϕ
 be the distance of the object to the cup point C, and let P be the point on B
 closest to C. The position vector p_CP defines an angle θ with the suction
 normal n̂. Notice that ϕ = ‖p_CP‖. We then model the pressure at P with an
 ad-hoc law that vanishes at distances larger than a model threshold dₘₐₓ and
 that goes to zero "behind" the cup smoothly with the angle to the normal:

   p(P) = (dₘₐₓ-ϕ)₊/dₘₐₓ ⋅ pₘₐₓ ⋅ cos(θ)₊

 where pₘₐₓ is the maximum pressure when the cup is blocked (the flow rate is
 zero) and (x)₊ = max(0, x).

 @tparam_default_scalar
*/
template <typename T>
class SuctionCupModel {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SuctionCupModel)

  SuctionCupModel(SuctionCupModelParameters parameters)
      : parameters_(std::move(parameters)) {
    // TODO: check parameters invariants. E.g. positive pressures, etc.
  }

  /* Computes the suction force at a point P specified by its relative position
    to the center of the cup C, expressed in the frame B in which the cup is
    defined. Refer to the class's documentation for further details. 
    param[in] p_CP_B position of an external point P, relative to the cup's center C, expressed in a frame B. 
    */
  Vector3<T> CalcSuctionForce(const Vector3<T>& p_CP_B) const {
    using std::max;
    const double d_max = parameters_.max_suction_distance;
    // Distance along the normal. Put it another way, distance from P to C,
    // multiplied by cos(θ), with θ the angle between the normal and the vector
    // p_BC. This gives the model a sense of "flow direction", though completely
    // ad-hoc. Better models based on published data could use a more realistic
    // function of θ.
    // Only the positive part is considered (that is, the force should at least
    // become zero "behind the cup").
    const T phi = p_CP_B.norm();
    constexpr double epsilon =
        1.0e-14;  // small tolerance to avoid division by zero. This effectively
                  // smooths out the distance function for values lower than
                  // epsilon.
    const T cost_theta = parameters_.cup_normal_B.dot(p_CP_B) / (phi + epsilon);

    // Ad-hoc functional form of the pressure with distance and angle to the
    // cup's normal (i.e. the force should be zero if "behind" the cup).
    // In reality, pressure will be a  function of the pump's characteristic
    // curve and the blockage parameter at the cup's exit (in turn function of
    // external geometry). These will determine the flow rate and pressure for a
    // given pump head. Here this ad-hoc model is simply lumping this
    // computation into a single algebraic relationship.
    const T pressure =
        p_max * max(0.0, (d_max - phi) / d_max) * max(0.0, cost_theta);

    // Force at pint P, expressed in frame B.
    return pressure * parameters_.area * parameters_.cup_normal_B;
  }

 private:
  SuctionCupModelParameters parameters_;
};
#endif

/* Parameters for a SuctionCup system. */
struct SuctionCupParameters {
  // The mass of the suction cup device, in kilograms.
  double mass;
  // The effective linear stiffness k, in N/m, of the suction cup device. That
  // is, if we press the suction cup a distance d, we'd need to apply a force f
  // = k⋅d.
  double stiffness;
  // Radius of the circular rim of the suction cup, in meters.
  double suction_rim_radius;
  // Force at distances farther than max_suction_distance are zero, in meters.
  double max_suction_distance;  
  // The rim geometry is discretized with a number of discrete points.
  int num_rim_nodes;
};

/** A System that connects to the MultibodyPlant in order to model the effects
 of one or more suction cups attached to specific bodies.

 Each suction cup is modeled according to SuctionCupModel, refer to
 SuctionCupModel's documentation for details.

 @system
 name: SuctionCup
 input_ports:
 - suction_pressure
 - body_poses
 - query_object
 output_ports:
 - spatial_forces
 TODO: consider adding a flow entrance loss output port (see Section 6.9 of
 White, F.M., 2011. Fluid mechanics, seventh edition) to allow modeling
hydraulic circuits involving this suction cup model.
 @endsystem

 - The suction_pressure input is a scalar value for the pressure on the
   suction side of the cup. It is a gage pressure relative to atmospheric (i.e.
   negative for vacuum).
 - The query_object port must be the same query object used by the
   MultibodyPlant model. It can be requested with MultibodPlat::
   get_geometry_query_input_port().
 - It is expected that the body_poses input should be connected to the
  @ref MultibodyPlant::get_body_poses_output_port() "MultibodyPlant body_poses
  output port".
 - The output is of type std::vector<ExternallyAppliedSpatialForce<T>>; it is
  expected that this output will be connected to the @ref
  MultibodyPlant::get_applied_spatial_force_input_port()
  "externally_applied_spatial_force input port" of the MultibodyPlant.
- This system does not have any state.
- "command" and "query_object" direct feedthrough to the output spatial_forces.

@tparam_default_scalar
**/
template <typename T>
class SuctionCup : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SuctionCupModel)

  /** This function adds the necessary mass and spring elements into  `plant` to
   model a compliant suction cup model, and returns a new SuctionCup system
   modeled with these elements. Refer to this class's documentation for further
   details on the model.

   @param[in,out] plant On ouput, `plant` will contain new mass and spring
   elements needed to model a suction cup. After this call, plant.num_bodies()
   and plant.num_velocities() will increase by cup_parameters.num_rim_nodes+1.
   @param[in] body_index Index of a body already present in the input `plant` to
   which this new suction cup model is attached.
   @param[in] X_BC The pose of the cup frame C in the body frame B. Per this
   class's documentation, the z-axis points towards the outside of the cup,
   perpendicular to its suction area.
   @param[in] cup_parameters Parameters of the model defining the cup's size,
   mass and compliance. **/
  static std::unique_ptr<SuctionCup<T>> MakeAndAddToPlant(
      MultibodyPlant<T>* plant, BodyIndex body_index,
      const RigidTransform<double>& X_BC,
      SuctionCupModelParameters cup_parameters);

  /** This function makes a new SuctionCup model with MakeAndAddToPlant(), adds
   it to `builder` and connects its ports accordingly. **/
  static SuctionCup<T>* AddToBuilder(systems::DiagramBuilder<T>* builder,
                                     MultibodyPlant<T>* plant,
                                     BodyIndex body_index,
                                     const RigidTransform<double>& X_BC,
                                     SuctionCupModelParameters cup_parameters);

  /** The plant model to which this cup is added. **/
  const MultibodyPlant<T>& plant() const { return plant_; }

 private:
  // Struct to store MultibodyPlant quantities used to model a suction cup model
  // of mass m and linear stiffness k, with its rim discretized in N nodes.
  // There is a total of N + 1 bodies, with a total
  // mass of m.
  struct MultibodyElements {
    // Main body of the cup model, of mass m / 2.
    BodyIndex cup_body;
    // Bodies conforming the discretized rim of the cup.
    // Each of mass m / (2 * N).
    std::vector<BodyIndex> rim_bodies_;
    // rim_contact_points_[i] stores he id for the contact point (zero radius
    // sphere) for body rim_bodies_[i].
    std::vector<GeometryId> rim_contact_points_;
  };

  /* Constructor for a suction cup model attached on a body B.
   N.B. Since making a cup model requires the coordination of constructing the
   system, adding new elements to a MultibodyPlant and appropriately wiring
   input ports, we make construction private and provide users with helper
   methods MakeAndAddToPlant() and AddToBuilder(). */
  SuctionCupModel(MultibodyPlant<T>* plant, BodyIndex body_index,
                  const RigidTransform<double>& X_BC,
                  SuctionCupModelParameters cup_parameters)
      : plant_(plant),
        body_index_(body_index),
        X_BC_(X_BC),
        parameters_{std::move(parameters)} {
    // N.B. This system has not state. It is a feedthrough with its outputs
    // being completely determined from the inputs.

    // Input ports.
    suction_pressure_index_ =
        this->DeclareInputPort("suction_pressure", systems::kVectorValued, 1)
            .get_index();
    body_poses_index_ =
        this->DeclareAbstractInputPort("body_poses",
                                       Value<std::vector<RigidTransform<T>>>())
            .get_index();
    query_object_input_port_ =
        DeclareAbstractInputPort(
            "query_object",
            drake::Value<drake::geometry::QueryObject<double>>())
            .get_index();

    // Output ports.
    this->DeclareAbstractOutputPort(
        "spatial_forces", std::vector<ExternallyAppliedSpatialForce<T>>(),
        &SuctionCup<T>::CalcSpatialForces);

    // Add elements to `plant` to model the suction cup.
    multibody_elements_ = MakeMultibodyPlantModel(plant);
  }

  // The suction cup is modeled as a network of spring and masses. 
  MultibodyElements MakeMultibodyPlantModel(MultibodyPlant<T>* plant) const {

    
  }

  // Helper method to get the body corresponding to a given geometry.
  const Body<T>& GetBodyGivenGeometryId(
      const drake::geometry::QueryObject<T>& query_object,
      GeometryId id) const {
    // TODO: Fill in the blanks. Get inspector form the query objet.
    const FrameId f_id = inspector.GetFrameId(id);
    const Body<T>* body = plant.GetBodyFromFrameId(f_id);
    DRAKE_DEMAND(body != nullptr);
    return *body;
  }

  void CalcSpatialForces(
      const systems::Context<T>& context,
      std::vector<ExternallyAppliedSpatialForce<T>>* spatial_forces) const {
    // spatial_forces->resize(num_propellers());
    const auto& command = get_command_input_port().Eval(context);
    const auto& query_object =
        get_query_object_input_port()
            .Eval<drake::geometry::QueryObject<T>>(context);

    // N.B. We do not know the number of externally applied force a priory.
    // Therefore we clear and the accumulate the results as we perform distance
    // queries.
    spatial_forces->clear();
    int cup_index = 0;
    for (const auto& body_cup : parameters_) {
      const BodyIndex body_index = body_cup.body_index;
      const Body<T>& body = plant().get_body(body_index);
      const math::RigidTransform<T>& X_WB = body.EvalPoseInWorld(context);
      const Vector3<T> p_WCup = X_WB * body_cup.p_BC;
      const std::vector<SignedDistanceToPoint<T>> distances =
          query_object.ComputeSignedDistanceToPoint(
              p_WCup, body_cup.parameters.max_suction_distance);

      const SuctionCupModel<T> cup_model(body_cup.parameters);

      // Compute suction force applied by cup C on all bodies at distance below
      // max_suction_distance.
      for (const SignedDistanceToPoint<T>& point_distance : distances) {
        const Body<T>& body = GetBodyGivenGeometryId(point_distance.id_G);
        const Vector3<T>& p_GP = point_distance.p_GN;
        // TODO: use inspector to query X_BG and transform p_GP to p_BP, the
        // postion of point P expressed in the body frame B.
        const T& distance = point_distance.distance;

        // Force on body B at point P, expressed in body frame B.
        const Vector3<T> f_Bp_B =
            command[cup_index] * cup_model.CalcSuctionForce(p_BP);

        // TODO: Transform to world frame, compute relative postion p_BoP_W and
        // load the externally applied forces vector.

        // TODO: Force on suction cup body is minus the force on B.

        spatial_forces->push_back({body.index(), p_BP, F_Bp_B});

        // Spatial force on cup body (usually a gripper or end effector), at
        // point C, expressed in the cup body frame.
        spatial_forces->push_back(
            {body_cup.body_index, body_cup.p_BC, F_CupBodyAtC});
      } 
      ++cup_index; 
    }
  }

  const MultibodyPlant<T>* plant_{nullptr};
  std::vector<SuctionCupPerBodyParameters> parameters_;
  systems::InputPortIndex suction_pressure_index_{};
  systems::InputPortIndex body_poses_index_{};
  systems::InputPortIndex query_object_input_port_{};
  // MultibodyPlant elements used in the modeling of the suction cup.
  MultibodyElements multibody_elements_;
};

}  // namespace suction_gripper
}  // namespace multibody
}  // namespace examples
}  // namespace drake