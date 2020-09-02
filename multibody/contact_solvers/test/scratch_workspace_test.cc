#include "drake/multibody/contact_solvers/scratch_workspace.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace {

GTEST_TEST(ScratchWorkspace, NestedScopes) {
  const int kNv = 20;
  const int kNc = 30;
  const int kMaxVectors = 6;
  ScratchWorkspace<double> w(kNv, kNc, kMaxVectors);
  GrantScratchWorkspaceAccess<double> a1(w);
  auto& xc11 = a1.xc_sized_vector();
  auto& xc12 = a1.xc_sized_vector();
  ASSERT_EQ(xc11.size(), 3 * kNc);
  ASSERT_EQ(xc12.size(), 3 * kNc);
  EXPECT_EQ(w.num_xc_vectors(), 2);
  {  // A nested scope.
    GrantScratchWorkspaceAccess<double> a2(w);
    auto& xc21 = a2.xc_sized_vector();
    auto& xc22 = a2.xc_sized_vector();

    // Expected sizes for the new vectors.
    ASSERT_EQ(xc21.size(), 3 * kNc);
    ASSERT_EQ(xc22.size(), 3 * kNc);

    // Vectors from the previous scope should still be valid.
    ASSERT_EQ(xc11.size(), 3 * kNc);
    ASSERT_EQ(xc12.size(), 3 * kNc);

    EXPECT_EQ(w.num_xc_vectors(), 4);
  }
  EXPECT_EQ(w.xc_vectors_capacity(), 4);

  {  // Another nested scope.
    GrantScratchWorkspaceAccess<double> a2(w);
    auto& xc21 = a2.xc_sized_vector();

    // Expected sizes for the new vectors.
    ASSERT_EQ(xc21.size(), 3 * kNc);

    EXPECT_EQ(w.num_xc_vectors(), 3);
    EXPECT_EQ(w.xc_vectors_capacity(), 4);
  }

  {  // Another nested scope.
    GrantScratchWorkspaceAccess<double> a2(w);
    auto& xc21 = a2.xc_sized_vector();
    auto& xc22 = a2.xc_sized_vector();
    auto& xc23 = a2.xc_sized_vector();

    // Expected sizes for the new vectors.
    ASSERT_EQ(xc21.size(), 3 * kNc);
    ASSERT_EQ(xc22.size(), 3 * kNc);
    ASSERT_EQ(xc23.size(), 3 * kNc);

    EXPECT_EQ(w.num_xc_vectors(), 5);
    EXPECT_EQ(w.xc_vectors_capacity(), 5);
  }
  EXPECT_EQ(w.num_xc_vectors(), 2);
  EXPECT_EQ(w.xc_vectors_capacity(), 5);

  // Within nested loops.
  for (int i = 0; i < 10; ++i) {
    GrantScratchWorkspaceAccess<double> a3(w);
    auto& xc31 = a3.xc_sized_vector();
    auto& xc32 = a3.xc_sized_vector();
    ASSERT_EQ(xc31.size(), 3 * kNc);
    ASSERT_EQ(xc32.size(), 3 * kNc);
    EXPECT_EQ(w.num_xc_vectors(), 4);
    for (int j = 0; j < 10; ++j) {
      GrantScratchWorkspaceAccess<double> a4(w);
      auto& xc41 = a4.xc_sized_vector();
      auto& xc42 = a4.xc_sized_vector();
      ASSERT_EQ(xc41.size(), 3 * kNc);
      ASSERT_EQ(xc42.size(), 3 * kNc);
      EXPECT_EQ(w.num_xc_vectors(), 6);
      EXPECT_EQ(w.xc_vectors_capacity(), 6);
    }
    EXPECT_EQ(w.num_xc_vectors(), 4);
    EXPECT_EQ(w.xc_vectors_capacity(), 6);
  }
}

}  // namespace
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake