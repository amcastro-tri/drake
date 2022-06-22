#include <iostream>
#include <chrono>

#include <drake/math/autodiff.h>
#include <drake/math/autodiff_gradient.h>

using namespace drake;

/** 
 * A normal function that computes y = f(x) using autodiff.
 */
VectorX<AutoDiffXd> normal_function(MatrixX<double> A, VectorX<AutoDiffXd> x) {
  return x.transpose()*A*x;
}

/**
 * A fancy function that computes y = f(x) with floating point math, then uses our
 * special insights to compute dy/dx analytically. These are then loaded back
 * into the derivatives() of y. 
 */
VectorX<AutoDiffXd> fancy_function(MatrixX<double> A, VectorX<AutoDiffXd> x) {

  VectorX<double> val = math::ExtractValue(x);
  MatrixX<double> grad = math::ExtractGradient(x);

  // Compute the main result using floating point math
  VectorX<double> res = val.transpose()*A*val;

  // Compute the derivatives() analytically. 
  MatrixX<double> dy_dx = val.transpose()*(A + A.transpose());
  MatrixX<double> deriv = dy_dx*grad;  // chain rule

  // Load those into a newly computed result.
  // Note that other versions of InitializeAutoDiff may end up being
  // more useful to us, such as the one that takes a pointer to y 
  // (the resulting autodiff matrix) rather than returning it.
  VectorX<AutoDiffXd> y = math::InitializeAutoDiff(res, deriv);

  return y;
}


int main() {
  // Size of the vector x
  int n = 100;

  // Define original vector
  VectorX<double> x(n);
  x.setRandom(n,1);
  VectorX<AutoDiffXd> x_ad = math::InitializeAutoDiff(x);
 
  // Define the matrix that we use for the test function y = x'Ax
  MatrixX<double> A(n,n);
  A.setRandom(n,n);

  // Set up some timers
  auto st = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> elapsed_normal;
  std::chrono::duration<float> elapsed_fancy;
 
  // Do some maths
  st = std::chrono::high_resolution_clock::now();
  auto y_normal = normal_function(A, x_ad);
  elapsed_normal = std::chrono::high_resolution_clock::now() - st;

  st = std::chrono::high_resolution_clock::now();
  auto y_fancy = fancy_function(A, x_ad);
  elapsed_fancy = std::chrono::high_resolution_clock::now() - st;

  // Print the results
  std::cout << "normal method: " << elapsed_normal.count() << " seconds" << std::endl;
  std::cout << "fancy method: " << elapsed_fancy.count() << " seconds" << std::endl;

  std::cout << std::endl;

  // Sanity check
  auto value_diff = math::ExtractValue(y_normal) - math::ExtractValue(y_fancy);
  auto deriv_diff = math::ExtractGradient(y_normal) - math::ExtractGradient(y_fancy);

  if (value_diff.norm() < 1e-12) {
    std::cout << "values match" << std::endl;
  }
  if (deriv_diff.norm() < 1e-12) {
    std::cout << "gradients match" << std::endl;
  }

  return 0;
}
