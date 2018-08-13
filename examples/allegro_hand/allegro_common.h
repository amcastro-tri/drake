#pragma once

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace allegro_hand {

constexpr int kAllegroNumJoints = 8;

/// Used to set the feedback gains for the simulated position control
void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp,
                                    Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd);



}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake