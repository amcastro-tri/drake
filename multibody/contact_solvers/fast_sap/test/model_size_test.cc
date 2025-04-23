#include "drake/multibody/contact_solvers/fast_sap/model_size.h"

#include <numeric>
#include <vector>

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace fast_sap {

GTEST_TEST(ModelSizes, Construction) {
  const int kNumPatches = 3;
  const std::vector<int> kPatchCliques = {1, 2, 1};
  TwoCliquesConstraintSizes patches{kNumPatches, kPatchCliques};

  const int kNumCliques = 2;
  const std::vector<int> kCliquesNv = {6, 22};
  const int kNv = std::accumulate(kCliquesNv.begin(), kCliquesNv.end(), 0);

  ModelSizes model{kNumCliques, kNv, kNumPatches, kCliquesNv, {}, {}, patches};

  EXPECT_EQ(model.num_cliques, kNumCliques);
}

}  // namespace fast_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake