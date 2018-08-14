#include "drake/examples/allegro_hand/allegro_common.h"

namespace drake {
namespace examples {
namespace allegro_hand {

void SetPositionControlledIiwaGains(Eigen::VectorXd* Kp, Eigen::VectorXd* Ki,
                                    Eigen::VectorXd* Kd) {
  // All the gains are for acceleration, not directly responsible for generating
  // torques. These are set to high values to ensure good tracking. These gains
  // are picked arbitrarily.
  Kp->resize(kAllegroNumJoints);
  *Kp = Eigen::VectorXd::Ones(kAllegroNumJoints) * 0.01;
  // (*Kp)[0] /= 5;
  Kd->resize(Kp->size());
  for (int i = 0; i < Kp->size(); i++) {
    // Critical damping gains.
    // (*Kd)[i] = 2 * std::sqrt((*Kp)[i]);
    (*Kd)[i] = 1e-3;
  }
  // (*Kd)[0] /= 5;
  *Ki = Eigen::VectorXd::Zero(kAllegroNumJoints);


  // Kp->segment<4>(0).setZero();
  // Kd->segment<4>(0).setZero();
}





}  // namespace allegro_hand
}  // namespace examples
}  // namespace drake