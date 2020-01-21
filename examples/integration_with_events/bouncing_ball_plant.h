#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/leaf_system.h"

#include <iostream>
//#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VAR(a) (void) a;

namespace drake {
namespace examples {
namespace scene_graph {
namespace bouncing_ball {

/** A model of a bouncing ball with Hunt-Crossley compliant contact model.
 The model supports 1D motion in a 3D world.

 @tparam T The vector element type, which must be a valid Eigen scalar.

 Instantiated templates for the following kinds of T's are provided:
 - double */
template <typename T>
class BouncingBallPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BouncingBallPlant)

  /** Constructor
   @param source_id             The source id for this plant to interact with
                                GeoemtrySystem.
   @param scene_graph           Pointer to the geometry system instance on which
                                this plant's geometry will be registered. It
                                must be the same system the source id was
                                extracted from.
   @param p_WB                  The 2D, projected position vector of the ball
                                onto the ground plane relative to the world.
   */
  BouncingBallPlant(geometry::SourceId source_id,
                    geometry::SceneGraph<T>* scene_graph,
                    const Vector2<double>& p_WB, bool with_normal_event);
  ~BouncingBallPlant() override;

  int num_positions() const { return 1; }
  int num_velocities() const { return 1; }
  int num_states() const { return num_positions() + num_velocities(); }

  const systems::InputPort<T>& get_geometry_query_input_port() const;
  /** Returns the port to output state. */
  const systems::OutputPort<T>& get_state_output_port() const;
  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  void set_z(systems::Context<T>* context, const T& z) const {
    auto xc = dynamic_cast<systems::BasicVector<T>&>(
                  context->get_mutable_continuous_state_vector())
                  .get_mutable_value();

    xc[0] = z;
  }

  void set_zdot(systems::Context<T>* context, const T& zdot) const {
    auto xc = dynamic_cast<systems::BasicVector<T>&>(
                  context->get_mutable_continuous_state_vector())
                  .get_mutable_value();
    xc[1] = zdot;
  }

  const T get_z(const systems::Context<T>& context) const {
    const auto xc = dynamic_cast<const systems::BasicVector<T>&>(
                  context.get_continuous_state_vector())
                  .get_value();
  return xc[0];
  }

  const T get_zdot(const systems::Context<T>& context) const {
    const auto xc = dynamic_cast<const systems::BasicVector<T>&>(
                  context.get_continuous_state_vector())
                  .get_value();
    return xc[1];
  }

  /** Mass in kg. */
  double m() const { return m_; }

  /** Stiffness constant in N/m */
  double k() const { return k_; }

  /** Hunt-Crossley's dissipation factor in s/m. */
  double d() const { return d_; }

  /** Gravity in m/s^2. */
  double g() const { return g_; }

 private:
  // Callback for writing the state vector to an output.
  void CopyStateToOutput(const systems::Context<T>& context,
                         systems::BasicVector<T>* state_output_vector) const;

  // Calculate the frame pose set output port value.
  void CalcFramePoseOutput(const systems::Context<T>& context,
                           geometry::FramePoseVector<T>* poses) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void DoCalcNextUpdateTime(const systems::Context<T>& context,
                            systems::CompositeEventCollection<T>* events_collection,
                            T* time) const override {                              
    using std::abs;                             
    using std::sqrt; 

    PRINT_VAR(context.get_time());

    if (!with_normal_event_) {      
      *time = std::numeric_limits<double>::infinity();
      return;
    }

    const double radius = diameter_ / 2.0;
    const T phi = get_z(context) - radius;  // signed distance.                              
    const T phidot = get_zdot(context);

    PRINT_VAR(phi);
    PRINT_VAR(phidot);

    if (phi > 0.0 && phidot < 0.0) {
      const T max_penetration = sqrt(m_/k_) * abs(phidot);

      PRINT_VAR(max_penetration);

      // Approximate time to when penetration is max_penetration/2.
      const T approximate_time_to_collision =
          (phi + max_penetration / 2.0) / abs(phidot);

      PRINT_VAR(approximate_time_to_collision);

      *time = approximate_time_to_collision + context.get_time();

#if 0
      // no-op callback
      systems::PublishEvent<double>::PublishCallback callback =
          [](const Context<double>&, const PublishEvent<double>&) {};
      events_collection->get_mutable_publish_events().add_event(
          std::make_unique<systems::PublishEvent<double>>(
              systems::TriggerType::kTimed, callback));
#endif
      systems::PublishEvent<T> event(systems::TriggerType::kTimed);
      event.AddToComposite(events_collection);

    } else {
      *time = std::numeric_limits<double>::infinity();
    }
  }

  // The projected position of the ball onto the ground plane. I.e., it's
  // "where the ball bounces".
  const Vector2<double> p_WB_;
  // The id for the ball's frame.
  geometry::FrameId ball_frame_id_;
  // The id for the ball's geometry.
  geometry::GeometryId ball_id_;

  int geometry_pose_port_{-1};
  int state_port_{-1};
  int geometry_query_port_{-1};

  const double diameter_{0.1};  // Ball diameter, just for visualization (m).
  const double m_{0.1};         // kg
  const double g_{9.81};        // m/s^2
  // Stiffness constant [N/m]. Calculated so that under its own weight the ball
  // penetrates the plane by 1 mm when the contact force and gravitational
  // force are in equilibrium.
  const double k_{m_ * g_ / 0.000001};
  // Hunt-Crossley's dissipation factor.
  const double d_{0.0};  // s/m

  const bool with_normal_event_{false};
};

}  // namespace bouncing_ball
}  // namespace scene_graph
}  // namespace examples
}  // namespace drake
