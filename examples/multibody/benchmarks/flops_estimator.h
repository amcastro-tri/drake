#pragma once

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace examples {
namespace {

class BenchTimer {
 public:
  void start() { start_ = the_clock::now(); }

  void stop() {
    const the_clock::time_point end = the_clock::now();
    the_clock::duration duration = end - start_;
    best_ = std::min(best_, duration);
    worst_ = std::max(worst_, duration);
    total_ += duration;
    const double duration_secs = to_seconds(duration);
    total_squared_ += duration_secs * duration_secs;
    ++num_tries_;
  }

  /// Elapsed time since the last call to start(), in seconds.
  double elapsed() const {
    const the_clock::time_point end = the_clock::now();
    the_clock::duration duration = end - start_;
    return to_seconds(duration);
  }

  int num_tries() const { return num_tries_; }

  // Total time in seconds.
  double total() const { return to_seconds(total_); }

  double mean() const { return total() / num_tries(); }

  double std_dev() const {
    return std::sqrt(total_squared_ / num_tries() - mean() * mean());
  }

  // Best time in seconds.
  double best() const { return to_seconds(best_); }

  // Worst time in seconds.
  double worst() const { return to_seconds(worst_); }

  static constexpr double resolution() { return resolution_; }

 private:
  typedef std::chrono::steady_clock the_clock;
  // Timer resolution in seconds.
  static constexpr double resolution_ =
      static_cast<double>(the_clock::period::num) / the_clock::period::den;
  the_clock::time_point start_;

  static double to_seconds(const the_clock::duration& t) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t).count() /
           1e9;
  }

  // Statistics, in ms.
  the_clock::duration best_{std::chrono::hours(114)};  // the universe's age.
  the_clock::duration worst_{0};
  the_clock::duration total_{0};
  int num_tries_{0};
  double total_squared_{0};
};

class FlopsEstimator {
 public:
  FlopsEstimator() : samples_(GenerateArbitrarySamples()) {}

  void RunTests(int num_tries, int num_operations) {
    // Send the results somewhere so that the compiler doesn't turn our loops
    // into no-ops.
    std::ofstream gonowhere("/dev/null");
    for (int try_number = 0; try_number < num_tries; ++try_number) {
      gonowhere << DoAddition(num_operations);
      gonowhere << DoMultiplication(num_operations);
      gonowhere << DoDivision(num_operations);
    }
    // Estimate FLOPS.
    // We found that BenchTimer::best() is the most stable statistics we can
    // get given that there is a very real lower bound to this number imposed by
    // the hardware capabilites.
    // Typically BenchTimer::worst() can be as much as 40% larger that best().
    add_flops_ = num_operations / add_timer_.best();
    mul_flops_ = num_operations / mul_timer_.best();
    div_flops_ = num_operations / div_timer_.best();
  }

  double DoAddition(int num_operations) {
    const int num_samples = samples_.size();
    double result = 0.0;
    add_timer_.start();
    for (int i = 0; i < num_operations; i++) {
      result += samples_[i % num_samples];
    }
    add_timer_.stop();
    return result;
  }

  double DoMultiplication(int num_operations) {
    const int num_samples = samples_.size();
    double result = 1.0;
    mul_timer_.start();
    for (int i = 0; i < num_operations; i++) {
      result *= samples_[i % num_samples];
    }
    mul_timer_.stop();
    return result;
  }

  double DoDivision(int num_operations) {
    const int num_samples = samples_.size();
    double result = 1.0;
    div_timer_.start();
    for (int i = 0; i < num_operations; i++) {
      result /= samples_[i % num_samples];
    }
    div_timer_.stop();
    return result;
  }

  double add_flops() { return add_flops_; }
  double mul_flops() { return mul_flops_; }
  double div_flops() { return div_flops_; }

  // TODO(): consider removing these?
  const BenchTimer& add_timer() { return add_timer_; }
  const BenchTimer& mul_timer() { return mul_timer_; }
  const BenchTimer& div_timer() { return div_timer_; }

 private:
  // An arbitrary number of floting points, just enough to foul the compiler not
  // to perform any clever optimization. Small enough samples though so that
  // they still fit in the cache to avoid cache misses which would affect the
  // estimations.
  static constexpr int half_samples_ = 8;
  static constexpr int num_samples_ = 2 * half_samples_;

  // Helper method to generate a fixed size array filled with (and even number
  // of) arbitrary values.
  // However, there is a restriction on how we generate these values; for each
  // arbitrary value we also generate ...
  static std::array<double, num_samples_> GenerateArbitrarySamples() {
    // unsigned seed =
    // std::chrono::system_clock::now().time_since_epoch().count();
    constexpr unsigned seed = 1234;  // We always use the same seed.
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    // This must fit in cache or else it'll affect performance measurements.
    const int half_samples = half_samples_;
    const int num_samples = 2 * half_samples;
    std::array<double, num_samples_> samples;
    for (int i = 0; i < num_samples; i += 2) {
      double s = distribution(generator);
      samples[i] = s;
      samples[i + 1] = 1.0 / s;
    }
    return samples;
  }

  const std::array<double, num_samples_> samples_;
  BenchTimer add_timer_;
  BenchTimer mul_timer_;
  BenchTimer div_timer_;
  double add_flops_{};
  double mul_flops_{};
  double div_flops_{};
};

}  // namespace
}  // namespace examples
}  // namespace drake
