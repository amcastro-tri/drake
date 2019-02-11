#pragma once

namespace drake {
namespace test {

/// Instantiate this class in a unit test scope where malloc (and realloc,
/// etc.) should be disallowed.  When at least one instance of class alive in a
/// process, all calls to glibc malloc (etc.) will return failure.
///
/// This utility does not work on macOS.
class DisableMalloc final {
 public:
  /// Makes malloc fail.
  DisableMalloc();

  /// Allows malloc to succeed again.
  ~DisableMalloc();

  /// @name Does not allow copy, move, or assignment
  //@{
  DisableMalloc(const DisableMalloc&) = delete;
  void operator=(const DisableMalloc&) = delete;
  DisableMalloc(DisableMalloc&&) = delete;
  void operator=(DisableMalloc&&) = delete;
  //@}
};

}  // namespace test
}  // namespace drake
