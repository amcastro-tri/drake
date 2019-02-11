#include "drake/common/test_utilities/disable_malloc.h"

#include <cstdlib>
#include <stdexcept>

#include <gtest/gtest.h>

// A global variable to help prevent the compiler from optimizing out the call
// to malloc.  (Without this, it can reason that malloc-free is a no-op.)
volatile void* g_dummy;

namespace drake {
namespace test {
namespace {

// Calls malloc (and then immediately frees).
void CallMalloc() {
  void* dummy = malloc(16);
  g_dummy = dummy;
  free(dummy);
  if (g_dummy == nullptr) { throw std::runtime_error("null dummy"); }
}

GTEST_TEST(ReplaceMallocTest, BasicTest) {
  CallMalloc();  // Malloc is OK.
  {
    DisableMalloc guard;
    // The guarded code would go here; malloc is NOT ok.
  }
  CallMalloc();  // Malloc is OK again.
}

GTEST_TEST(ReplaceMallocDeathTest, BasicTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  CallMalloc();  // Malloc is OK.
  ASSERT_DEATH({
      DisableMalloc guard;
      CallMalloc();  // Malloc is NOT ok.
    }, "abort due to malloc while DisableMalloc is in effect");
}

}  // namespace
}  // namespace test
}  // namespace drake
