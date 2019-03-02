#include <algorithm>
#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <random>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsing/parser.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::SpatialVelocity;

DEFINE_bool(mbp_inertia, false, "Compute M with MBP.");
DEFINE_bool(mbp_inverse_dyn, true, "Compute ID with MBP.");
DEFINE_bool(mbp_enable_caching, true, "Enable caching for MBP.");

namespace drake {
namespace examples {
namespace {

typedef std::chrono::steady_clock the_clock;

class Timer {
 public:
  Timer() : beg_(Clock::now()) {}
  void reset() { beg_ = Clock::now(); }
  double elapsed() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() -
                                                                 beg_)
        .count();
#if 0              
              return std::chrono::duration_cast<second_>(Clock::now() -
                                                                beg_)
              .count();
#endif
  }

 private:
  // typedef std::chrono::high_resolution_clock Clock;
  typedef std::chrono::steady_clock Clock;
  typedef std::chrono::duration<double, std::ratio<1>> second_;
  std::chrono::time_point<Clock> beg_;
};

double AdditionTime() {
  double addRes = 1;
  auto start = the_clock::now();
  for (int i = 0; i < 5 * 100000000; i++) {
    addRes += double(1.1);
    addRes += double(1.2);
    addRes += double(1.3);
    addRes += double(1.4);
    addRes += double(1.501);
    addRes += double(1.6);
    addRes += double(1.7);
    addRes += double(1.8);
    addRes += double(1.9);
    addRes += double(2.007);
  }
  auto stop = the_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start) / 5.0;
  std::cout << "AdditionTime: " << duration.count() << " ms.\n";
  return addRes;
}

double MultiplicationTime() {
  double mulRes = 1;
  auto start = the_clock::now();
  for (int i = 0; i < 100000000; i++) {
    mulRes *= double(0.501);
    mulRes *= double(0.2501);
    mulRes *= double(0.201);
    mulRes *= double(0.101);
    mulRes *= double(1.000000001);
    mulRes *= double(1 / 1.000000002);  // done at compile time
    mulRes *= double(1 / .101);
    mulRes *= double(1 / .201);
    mulRes *= double(1 / .2501);
    mulRes *= double(1 / .501);
  }
  auto stop = the_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "MultiplicationTime: " << duration.count() << " ms.\n";
  return mulRes;
}

double DoNothing(double, double) {
  return 0;
}

double DoAddition(double a, double b) {
  return a + b;
}

double DoMultiplication(double a, double b) {
  return a * b;
}

double DoSqrt(double a, double b) {
  return std::sqrt(b);
}

class OperationsTest {
 public:
  OperationsTest(int num_reps) : num_reps_(num_reps),samples_(GenerateSamples()) {
  }

  void RunTests() {
    // Send the results somewhere so that the compile doesn't turn our loops
    // into no-ops.
    std::ofstream gonowhere("/dev/null");
    gonowhere << DoAddition() << std::endl;
    gonowhere << DoMultiplication() << std::endl;
    gonowhere << DoDivision() << std::endl;
    gonowhere << DoSqrt() << std::endl;
    gonowhere << DoExp() << std::endl;
    gonowhere << DoCos() << std::endl;
    gonowhere << DoSin() << std::endl;
    gonowhere << DoSinCos() << std::endl;
  }

  double DoAddition() {
    const int num_samples = samples_.size();
    double result = 0.0;
    auto start = the_clock::now();    
    for (int i = 0; i < num_reps_; i++) {
      result += samples_[i % num_samples];
    }
    auto stop = the_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Addition: " << duration.count() << " ms.\n";

    return result;
  }

  double DoMultiplication() {
    const int num_samples = samples_.size();
    double result = 1.0;
    auto start = the_clock::now();    
    for (int i = 0; i < num_reps_; i++) {
      result *= samples_[i % num_samples];
    }
    auto stop = the_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Multiplication: " << duration.count() << " ms.\n";

    return result;
  }

  double DoDivision() {
    const int num_samples = samples_.size();
    double result = 1.0;
    auto start = the_clock::now();    
    for (int i = 0; i < num_reps_; i++) {
      result /= samples_[i % num_samples];
    }
    auto stop = the_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Division: " << duration.count() << " ms.\n";

    return result;
  }

  double DoSqrt() {
    const int num_samples = samples_.size();
    double result = 0.0;
    auto start = the_clock::now();    
    for (int i = 0; i < num_reps_; i++) {
      result += sqrt(samples_[i % num_samples]);
    }
    auto stop = the_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Sqrt: " << duration.count() << " ms.\n";

    return result;
  }

  double DoExp() {
    const int num_samples = samples_.size();
    double result = 0.0;
    auto start = the_clock::now();    
    for (int i = 0; i < num_reps_; i++) {
      result += exp(samples_[i % num_samples]);
    }
    auto stop = the_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Exp: " << duration.count() << " ms.\n";

    return result;
  }

  double DoCos() {
    const int num_samples = samples_.size();
    double result = 0.0;
    auto start = the_clock::now();    
    for (int i = 0; i < num_reps_; i++) {
      result += cos(samples_[i % num_samples]);
    }
    auto stop = the_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Cos: " << duration.count() << " ms.\n";

    return result;
  }

  double DoSin() {
    const int num_samples = samples_.size();
    double result = 0.0;
    auto start = the_clock::now();    
    for (int i = 0; i < num_reps_; i++) {
      result += sin(samples_[i % num_samples]);
    }
    auto stop = the_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Sin: " << duration.count() << " ms.\n";

    return result;
  }

  double DoSinCos() {
    const int num_samples = samples_.size();
    double result = 0.0;
    auto start = the_clock::now();    
    for (int i = 0; i < num_reps_; i++) {
      result += sin(samples_[i % num_samples]);
      result += cos(samples_[i % num_samples]);
    }
    auto stop = the_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "SinCos: " << duration.count() << " ms.\n";

    return result;
  }

 private:
  static std::vector<double> GenerateSamples() {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    const int half_samples = 1;
    const int num_samples = 2 * half_samples;
    std::vector<double> samples(num_samples);
    for (int i = 0; i < num_samples; i+=2) {
      double s = distribution(generator);
      samples[i] = s;
      samples[i+1] = 1.00000001 / s;
    }
    return samples;
  }
  const int num_reps_;
  const std::vector<double> samples_;
};

double OperationTest(const std::string& name,
                     double(*op)(double, double)) {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> distribution(0.0, 1.0);

  const int num_samples = 1024 * distribution(generator) + 1024;
  auto generate_samples = [&]() {
    std::vector<double> samples(num_samples);
    std::generate(
        samples.begin(), samples.end(),
        [&generator, &distribution]() { return distribution(generator); });
    return samples;
  };

  const std::vector<double> samples1 = generate_samples();
  const std::vector<double> samples2 = generate_samples();

  //auto mult = [](double a, double b) { return a*b; };

  auto start = the_clock::now();
  double result = 1.0;
  for (int i = 0; i < 100000000; i++) {
    //result += op(samples1[i%num_samples], samples2[i%num_samples]);
    result = op(result, 1.000000001);
    #if 0
    result = op(result, samples1[(i+1)%num_samples]);
    result = op(result, samples1[(i+2)%num_samples]);
    result = op(result, samples1[(i+3)%num_samples]);
    result = op(result, samples1[(i+4)%num_samples]);
    result = op(result, samples1[(i+5)%num_samples]);
    result = op(result, samples1[(i+6)%num_samples]);
    result = op(result, samples1[(i+7)%num_samples]);
    result = op(result, samples1[(i+8)%num_samples]);
    result = op(result, samples1[(i+9)%num_samples]);
#endif
    //result = std::max(result,
     //                 op(samples1[i % num_samples], samples2[i % num_samples]));
  }
  auto stop = the_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << name << ": " << duration.count() << " ms.\n";

  return result;
}

int do_main()
{
#if 0  
    double total;
    Timer tmr;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    const int num_samples = 1024;
    std::vector<double> samples(num_samples);
    std::generate(
        samples.begin(), samples.end(),
        [&generator, &distribution]() { return distribution(generator); });


#define randf() ((double) rand()) / ((double) (RAND_MAX))
#define OP_TEST(name, expr)               \
    total = 0.0;                          \
    srand(42);                            \
    tmr.reset();                          \
    for (int i = 0; i < 100000000; i++) { \
        double r1 = samples[i%num_samples]; \
        double r2 = samples[(i+100)%num_samples]; \
        (void) r1;                        \
        (void) r2;                        \
        total += expr;                    \
    }                                     \
    double name = tmr.elapsed();          \
    printf(#name);                        \
    printf(" %.7f  %.7f %.7f\n", total, name, name - baseline);

#endif

    //std::cout << AdditionTime() << std::endl;
    (void)AdditionTime;
    std::cout << MultiplicationTime() << std::endl;

    OperationsTest(1e9).RunTests();

  (void) DoNothing;
  (void) DoAddition;
  (void)DoSqrt;
  (void)OperationTest;
  (void)DoMultiplication;
#if 0
    //std::cout << OperationTest("NoOp",
    //                           [](double a, double b) { return 0; }) << std::endl;
    std::cout << OperationTest("DoNothing", DoNothing) << std::endl;
    (void) DoAddition;
    std::cout << OperationTest("Summation", DoAddition) << std::endl;
    std::cout << OperationTest("Multiplication", DoMultiplication) << std::endl;
    std::cout << OperationTest("Sqrt", DoSqrt) << std::endl;
    #if 0
    std::cout << OperationTest("Summation",
                               [](double a, double b) { return a + b; }) << std::endl;
    std::cout << OperationTest("Subtraction",
                               [](double a, double b) { return a - b; }) << std::endl;
    std::cout << OperationTest("Multiplication",
                               [](double a, double b) { return a * b; }) << std::endl;
    std::cout << OperationTest("Division",
                               [](double a, double b) { return a / b; }) << std::endl;
                               #endif
#endif

    // time the baseline code:
    //   for loop with no extra math op
    //OP_TEST(baseline, 1.0)

    // time various floating point operations.
    //   subtracts off the baseline time to give
    //   a better approximation of the cost
    //   for just the specified operation
    //OP_TEST(plus, r1 + r2)
    //OP_TEST(minus, r1 - r2)
    //OP_TEST(mult, r1 * r2)    
    //OP_TEST(div, r1 / r2)
    //OP_TEST(sqrt, sqrt(r1))
    //OP_TEST(sin, sin(r1))
    //OP_TEST(cos, cos(r1))
    //OP_TEST(tan, tan(r1))
    //OP_TEST(atan, atan(r1))
    //OP_TEST(exp, exp(r1))

    return 0; 
}

#if 0
int do_main() {
  const int nq = 7;
  const int num_reps = 10000;
  //const int num_autodiff_reps = 50;
  const int id_num_reps = 10 * num_reps;

  // Load a model of an Iiwa arm.
  MultibodyPlant<double> plant;
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_primitive_collision.urdf"));
  plant.AddForceElement<drake::multibody::UniformGravityFieldElement>(
      -9.81 * Eigen::Vector3d::UnitZ());
  plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("base"),
    math::RigidTransform<double>(Eigen::Vector3d::Zero()).GetAsIsometry3());
  plant.Finalize();

  auto context = plant.CreateDefaultContext();
  if (FLAGS_mbp_enable_caching) {
    context->EnableCaching();
  }

  // ===========================================================================
  // MBP MASS MATRIX
  // ===========================================================================
  if (FLAGS_mbp_inertia) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2 * nq);
    Eigen::MatrixXd M(nq, nq);
    {
      x.setLinSpaced(1, 5);
      x.segment(nq, nq).setZero();
      context->get_mutable_continuous_state_vector().SetFromVector(x);
      plant.CalcMassMatrixViaInverseDynamics(*context, &M);
      PRINT_VARn(M);
      PRINT_VAR((M - M.transpose()).norm());
    }

    auto start = my_clock::now();
    for (int i = 0; i < num_reps; i++) {
      x(0) = i;
      context->get_mutable_continuous_state_vector().SetFromVector(x);
      plant.CalcMassMatrixViaInverseDynamics(*context, &M);
    }
    auto stop = my_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(plant)" << std::to_string(num_reps) << "x inertia "
                                                                    "calculations took "
              <<
              duration.count() << " milliseconds." << std::endl;
  }


  // Build and test multibody plant w/autodiff
  // std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
  //   plant.ToAutoDiffXd();

#if 0
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(plant);

  auto context_autodiff =
    plant_autodiff->CreateDefaultContext();
  context_autodiff->EnableCaching();

  start =  my_clock::now();
  MatrixX<AutoDiffXd> M_autodiff(nq, nq);
  for (int i = 0; i < num_autodiff_reps; i++) {
    x(0) = i;
    context_autodiff->get_mutable_continuous_state_vector().
        SetFromVector(math::initializeAutoDiff(x));
    plant_autodiff->CalcMassMatrixViaInverseDynamics(
        *context_autodiff, &M_autodiff);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(plant)" << std::to_string(num_autodiff_reps) << "xinertia autodiff calculations took " <<
      duration.count() << " milliseconds." << std::endl;
#endif

  // ===========================================================================
  // RBT INVERSE DYNAMICS
  // ===========================================================================
  if (FLAGS_rbt_inverse_dyn) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2 * nq);
    Eigen::VectorXd desired_vdot(nq);
    auto start = my_clock::now();
    RigidBodyTree<double>::BodyToWrenchMap external_wrenches;

    for (int i = 0; i < id_num_reps; i++) {
      x = Eigen::VectorXd::Constant(2 * nq, i);
      desired_vdot = Eigen::VectorXd::Constant(nq, i);
      auto cache =
          rigid_body_plant.get_rigid_body_tree().doKinematics(x.head(nq),
                                                              x.tail(nq));
      rigid_body_plant.get_rigid_body_tree().inverseDynamics(cache,
                                                             external_wrenches,
                                                             desired_vdot);
    }
    auto stop = my_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(rigid_body_plant)" << std::to_string(id_num_reps) <<
              "x inverse dynamics calculations took " <<
              duration.count() << " milliseconds." << std::endl;
  }
#if 0
  start =  my_clock::now();
    RigidBodyTree<AutoDiffXd>::BodyToWrenchMap external_wrenches_autodiff;

  for (int i = 0; i < num_autodiff_reps; i++) {
    x = Eigen::VectorXd::Constant(2*nq, i);
    desired_vdot = Eigen::VectorXd::Constant(nq, i);
    auto cache = rigid_body_plant.get_rigid_body_tree().doKinematics(
        math::initializeAutoDiff(x.head(nq)),
        math::initializeAutoDiff(x.tail(nq)));
    rigid_body_plant.get_rigid_body_tree().inverseDynamics(cache,
        external_wrenches_autodiff, math::initializeAutoDiff(desired_vdot));
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(rigid_body_plant)" << std::to_string(num_autodiff_reps) <<
  "x autodiff inverse dynamics calculations took "<<
      duration.count() << " milliseconds." << std::endl;
#endif

  // ===========================================================================
  // MBP INVERSE DYNAMICS
  // ===========================================================================
  if (FLAGS_mbp_inverse_dyn) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2 * nq);
    Eigen::VectorXd desired_vdot(nq);
    multibody::MultibodyForces<double> external_forces(plant);
    std::vector<SpatialAcceleration<double>> A_WB_array(
        plant.num_bodies());
    // Creates problems with drake::SpatialForce in eigen_types.h!!
    std::vector<drake::multibody::SpatialForce<double>> F_BMo_W_array(
        plant.num_bodies());
    VectorX<double> tau_id(plant.num_velocities());

    //CALLGRIND_START_INSTRUMENTATION;
    ///////////////////////////////////
    // ID does NO heap allocation.
    auto start = my_clock::now();
    for (int i = 0; i < id_num_reps; i++) {
      x = Eigen::VectorXd::Constant(2 * nq, i);
      desired_vdot = Eigen::VectorXd::Constant(nq, i);
      context->get_mutable_continuous_state_vector().SetFromVector(x);
      {
        // I only put this as proof that no heap allocation is being done.
        test::DisableMalloc guard;
        plant.CalcInverseDynamics(*context, desired_vdot,
                                            external_forces, &A_WB_array,
                                            &F_BMo_W_array, &tau_id);
      }
    }
    auto stop = my_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(plant (no heap))" << std::to_string(id_num_reps)
              << "x inverse dynamics calculations took " << duration.count()
              << " milliseconds." << std::endl;
    //CALLGRIND_STOP_INSTRUMENTATION;
    //CALLGRIND_DUMP_STATS;

    ///////////////////////////////////
    // ID WITH heap allocation.
    start = my_clock::now();
    for (int i = 0; i < id_num_reps; i++) {
      x = Eigen::VectorXd::Constant(2 * nq, i);
      desired_vdot = Eigen::VectorXd::Constant(nq, i);
      context->get_mutable_continuous_state_vector().SetFromVector(x);
      {
        // test::DisableMalloc guard; // This guard would not pass.
        plant.CalcInverseDynamics(*context, desired_vdot,
                                            external_forces);
      }
    }
    stop = my_clock::now();
    duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "(plant (with heap))" << std::to_string(id_num_reps)
              << "x inverse dynamics calculations took " << duration.count()
              << " milliseconds." << std::endl;
  }

#if 0
  start =  my_clock::now();
  multibody::MultibodyForces<AutoDiffXd> external_forces_autodiff(
      *plant_autodiff);

  for (int i = 0; i < num_autodiff_reps; i++) {
    x = Eigen::VectorXd::Constant(2*nq, i);
    desired_vdot = Eigen::VectorXd::Constant(nq, i);
    context_autodiff->get_mutable_continuous_state_vector().SetFromVector(math::initializeAutoDiff(x));
    plant_autodiff->CalcInverseDynamics(*context_autodiff, math::initializeAutoDiff(desired_vdot),
                                        external_forces_autodiff);
  }
  stop =  my_clock::now();
  duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "(plant)" << std::to_string(num_autodiff_reps) << "xautodiff inverse dynamics calculations took " <<
      duration.count() << " milliseconds." << std::endl;
#endif

  return 0;
}
#endif

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
