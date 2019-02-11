#include "drake/common/test_utilities/disable_malloc.h"

#include <atomic>
#include <cstdlib>
#include <iostream>

namespace {

// Returns the number of instances of DisableMalloc created.
std::atomic_int& instance_count() noexcept {
  // The C++ standard requires that std::atomic_int has a trivial destructor.
  // Therefore, we can have a function-static global without never_destroyed.
  static std::atomic_int g_instance_count;
  return g_instance_count;
}

// Aborts the program iff malloc has been disabled.
void guard_malloc() noexcept {
  if (instance_count().load() == 0) { return; }

  // Report an error (and re-enable malloc while doing so!).
  // TODO(jwnimmer-tri) It would be nice to print a backtrace here.
  instance_count() = 0;
  std::cerr << "abort due to malloc while DisableMalloc is in effect";
  std::cerr << std::endl;
  std::abort();
}

}  // namespace

namespace drake {
namespace test {
DisableMalloc::DisableMalloc() { ++instance_count(); }
DisableMalloc::~DisableMalloc() { --instance_count(); }
}  // namespace test
}  // namespace drake

// https://www.gnu.org/software/libc/manual/html_node/Replacing-malloc.html#Replacing-malloc
extern "C" void* __libc_malloc(size_t);
extern "C" void* __libc_free(void*);
extern "C" void* __libc_calloc(size_t, size_t);
extern "C" void* __libc_realloc(void*, size_t);
void* malloc(size_t size) {
  guard_malloc();
  return __libc_malloc(size);
}
void free(void* ptr) {
  __libc_free(ptr);
}
void* calloc(size_t nmemb, size_t size) {
  guard_malloc();
  return __libc_calloc(nmemb, size);
}
void* realloc(void* ptr, size_t size) {
  guard_malloc();
  return __libc_realloc(ptr, size);
}
