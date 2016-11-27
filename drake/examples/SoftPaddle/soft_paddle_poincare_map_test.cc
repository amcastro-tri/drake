#include "drake/common/drake_path.h"
#include "drake/examples/SoftPaddle/soft_paddle_poincare_map.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace examples {
namespace soft_paddle {
namespace {

using std::make_unique;
using Eigen::Isometry3d;

int do_main(int argc, char* argv[]) {

  double paddle_aim = 0.0; //- 2.0 * M_PI / 180.0;
  double stroke_strength = 0.05;
  double xn = 0.35;
  double zn = 0.4;
  double xnext, znext;

  SoftPaddlePoincareMap<double> poincare_map;

  poincare_map.ComputeNextSate(
      paddle_aim, stroke_strength,
      xn, zn, &xnext, &znext);

  PRINT_VAR(xnext);
  PRINT_VAR(znext);

  return 0;
}

}  // namespace
}  // namespace soft_paddle
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::soft_paddle::do_main(argc, argv);
}
