#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace multibody {
namespace load_model {
namespace {

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

int do_main() {
  const std::string full_name = FindResourceOrThrow(
      "drake/examples/multibody/load_model/double_pendulum.urdf");
  MultibodyPlant<double> plant;
  Parser(&plant).AddModelFromFile(full_name);
  return 0;
}

}  // namespace
}  // namespace load_model
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("Load model file for testing");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::load_model::do_main();
}
