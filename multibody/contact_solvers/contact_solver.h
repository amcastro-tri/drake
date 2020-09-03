#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"
#include "drake/multibody/contact_solvers/point_contact_data.h"
#include "drake/multibody/contact_solvers/system_dynamics_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {

/// The result from ContactSolver::SolveWithGuess() used to report the
/// success or failure of the solver.
enum class ContactSolverResult {
  /// Successful computation.
  kSuccess = 0,

  /// The solver could not find a solution at the specified tolerances.
  kFailure = 1,
};

/// This class defines a general interface for all of our contact solvers. By
/// having a common interace, client code such as MultibodyPlant only needs to
/// learn how to talk to %ContactSolver, allowing to swap contact solvers that
/// share this commong interface without having to re-wire the client's
/// internals.
///
/// <h3> Mechanical systems and state </h3>
///
/// In what follows, we describe the state of the system by the vector of
/// generalized positions q, or configuration, and the vector of generalized
/// velocities v. q is a vector of size `nq` and v a vector of size `nv`. Even
/// though in general `nq != nv`, the kinematic mapping `q̇ = N(q)⋅v` relates
/// the generalized velocities to time derivatives of the generalized positions.
///
/// For instance, the dynamics of rigid multibody systems can be stated as:
/// <pre>
///   M(q)⋅v̇ + C(q,v) = τₑₓₜ(q, v)
///   q̇ = N(q)⋅v
/// </pre>
/// where M(q) is the mass matrix, `C(q,v)` contains the Coriolis and
/// centrifugal contributions and `τₑₓₜ(q, v)` are external contributions,
/// including actuation for robotic applications.
/// Here we will focus on discretizing these equations at the velocity level in
/// which the unknowns of the system are the generalized velocities.
/// For instance, if we use a semi-implicit Euler approximation with a time step
/// dt, given the previous step velocity `v₀`, we approximate the accelerations
/// as `v̇ = (v − v₀)/dt`, the generalized positions derivatives as `q̇ =
/// N(q₀)⋅v` and the next step positions as `q = q₀ + dt⋅q̇`. Therefore the
/// discrete momentum equations will read: <pre>
///   M(q₀)⋅(v−v₀) + dt⋅C(q₀,v) = dt⋅τₑₓₜ(q₀, v)
/// </pre>
///
/// Using this framework in terms of generalized positions and velocities we can
/// describe any mechanical system. For instance, for FEM models, the
/// configuration q will corresponds to the Lagrangian coordinates of material
/// points in a solid and in this case q̇ = v, i.e. `N(q)` is the identity
/// mapping.
///
/// <h3> Contact constraints </h3>
///
/// Generally, we are interested on solving a set of momentum equations subject
/// to contact constraints of the form: <pre>
///   F(v) = Jcᵀ⋅γ
///   s.t. Contact constraints.
/// </pre>
/// where γ concatenates the all nc contact impulses γᵢ ∈ ℝ³ into a vector of
/// size 3nc and `Jc`, of size `3nc x nv`, is the "contact Jacobian" defined
/// such that contact velocities vc are given by vc = Jc⋅v.
/// `F(v)` describes the balance of momentum already discretized at the
/// velocity level. As an example, consider the dynamics of rigid bodies
/// discretized using the semi-implicit Euler scheme presented earlier. In this
/// case we have `F(v) = M(q₀)⋅(v−v₀) - dt⋅C(q₀,v) - dt⋅τₑₓₜ(q₀, v)`.
///
/// With "Contact constraints" we mean:
/// 1. Contact forces follow Coulomb's law of friction, i.e. γᵢ is inside the
///    friction cone.
/// 2. The friction component of γ, which we refer to as β, satisfies the
///    principle of maximum dissipation for sliding contacts.
/// 3. The normal component of γ, which we refer to as π, is non-negative
///    i.e. always a repulsive force (adhesive or “sticky" contact needs special
///    consideration).
///
/// This interface leaves open how exactly this constraints are imposed so that
/// specific solvers have the freedom to choose other model approximations. For
/// instance, we allow solvers to accommodate for the convex relaxation of
/// friction [Anitescu, 2006], regularization of constraints [Lacoursiere
/// et al. 2011] or compliant contact [Castro et al., 2019]. While the
/// general formulation of frictional contact is a Non-linear Complementarity
/// Problem (NCP), %ContactSolver's interface makes no assumption on the
/// underlying solver therefore also allowing for optimization based methods,
/// [Todorov, 2014; [Kaufman et al., 2008].
///
/// <h3> Solving the discrete contact problem </h3>
///
/// A general approach for solving the contact problem will include a predictor
/// step to compute velocities v* satisfying the predictor equations
/// `F(v*) = 0`. That is, v* corresponds to the velocities the system would
/// evolve with in the absence of contact forces, see for instance [Duriez,
/// 2013] for a case in which `F(v*)` = 0 is highly non-linear. Recall that
/// `F(v)` describes the "discrete" balance of momentum at the velocity level.
/// We already approximated the configuration `q(v*)`. For instance, for
/// implicit Euler we'd write `q(v*) = q₀+ dt⋅v*`.
/// The next step velocity is then approximated as `v = v* + Δv` where Δv is
/// computed in a corrector step satisfying the equation: <pre>
///   F(q(v* + Δv), v* + Δv) = F(v* + Δv) = Jcᵀ⋅γ
/// </pre>
/// We can linearize this equation at v*, leading to: <pre>
///   F(v*) + A⋅Δv = Jcᵀ⋅γ
/// </pre>
/// where we defined A = ∇F(v*) as the Jacobian of F with respect to generalized
/// velocities v, evaluated at v*.
/// Since v* satisfies the predictor's equation F(v*) = 0, the equation for the
/// impulses simplifies to: <pre>
///   A⋅Δv = Jcᵀ⋅γ
/// </pre>
///
/// As an example of application, consider the rigid multibody dynamics
/// equations discretized using an explicit approach for all non-contact forces,
/// as for instance in [Castro et al., 2019]. In this case F(q, v) takes the
/// form: <pre>
///   F(v) = M(q₀)⋅(v−v₀) − dt⋅τ₀
/// </pre>
/// where τ₀ includes external forces as well as Coriolis and centrifugal terms.
/// In this case A = ∇F = M, v* = v₀ + dt⋅M⁻¹⋅τ₀.
///
/// As a second example, consider the simulation of soft bodies with frictional
/// contact for which, without diving into the details, the momentum equations
/// can be briefly summarized as:
/// <pre>
///   F(v) = M⋅(v−v₀) + dt⋅Fᵢₙₜ(q, v)
/// </pre>
/// where with `Fᵢₙₜ(q, v)` we denote the term containing the contribution due
/// to internal stresses in the deformable object. Using a predictor as in
/// [Duriez, 2013], `F(v*) = 0`, leads to the system's dynamics matrix
/// `A(v*) = M + dt⋅D + dt²⋅K`, where `M` is the mass matrix, `D = ∂Fᵢₙₜ/∂v` is
/// the damping matrix and `K = ∂Fᵢₙₜ/∂q` is the stiffness matrix. For the
/// modeling of large deformations `Fᵢₙₜ(q, v)` is a non-linear function of both
/// q and v and therefore v* requires the solution of the non-linear system of
/// equations `Fᵢₙₜ(q(v*), v*) = 0` usually with a Newton method. As a side
/// effect of this solution the operator form of `A⁻¹` at v* will be available,
/// typically as a factorization of the sparse matrix `A`.
///
/// It should be noted that while in the previous two examples we used the
/// backward Euler method to obtain a discrte approximation in time for
/// `F(q, v)`, %ContactSolver does not make this or any other assumption on the
/// time stepping scheme chosen to obtain `F(v)`. In the previous examples any
/// other methods such as Crank–Nicolson, Newmark (very common in elasticity) or
/// Linear Multistep could be used to achieve higher order approximations or
/// desired stability properties.
///
/// Therefore, %ContactSolver needs the following information to properly define
/// the contact problem:
/// - The inverse operator of A and the predicted velocity v*. This essentially
/// defines the "dynamics" of the system. This is specified as a
/// SystemDynamicsData with SetSystemDynamicsData().
/// - Contact information. Initial penetration, contact Jacobian, possibly
/// stiffness and damping, friction coefficients, etc. This is specified as a
/// PointContactData with SetPointContactData().
///
/// Once the problem is set, SolveWithGuess() is used to invoke the solver.
/// Methods such as GetImpulses() and GetVelocities() are then used to retrieve
/// the solution γ and v, respectively.
///
// TODO(amcastro-tri): add sections specific to bilateral and unilateral
// constraints.
///
/// <h3> References: </h3>
/// - [Anitescu, 2006] Anitescu, M., 2006. Optimization-based simulation of
/// nonsmooth rigid multibody dynamics. Mathematical Programming, 105(1),
/// pp.113-143.
/// - [Castro et al., 2019] Castro, A.M., Qu, A., Kuppuswamy, N., Alspach, A.
/// and Sherman, M., 2020. A Transition-Aware Method for the Simulation of
/// Compliant Contact with Regularized Friction. IEEE Robotics and Automation
/// Letters, 5(2), pp.1859-1866.
/// - [Duriez, 2013] Duriez, C., 2013. Real-time haptic simulation of medical
/// procedures involving deformations and device-tissue interactions (Doctoral
/// dissertation).
/// - [Kaufman et al., 2008] Kaufman, D.M., Sueda, S., James, D.L. and Pai,
/// D.K., 2008. Staggered projections for frictional contact in multibody
/// systems. In ACM SIGGRAPH Asia 2008 papers (pp. 1-11).
/// - [Lacoursiere et al. 2011] Lacoursiere, C. and Linde, M., 2011. Spook: a
/// variational time-stepping scheme for rigid multibody systems subject to dry
/// frictional contacts. UMINF report, 11.
/// - [Todorov, 2014] Todorov, E., 2014, May. Convex and analytically-invertible
/// dynamics with contacts and constraints: Theory and implementation in MuJoCo.
/// In 2014 IEEE International Conference on Robotics and Automation (ICRA) (pp.
/// 6054-6061). IEEE.
///
/// @tparam_nonsymbolic_scalar
template <typename T>
class ContactSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactSolver);
  ContactSolver() = default;
  virtual ~ContactSolver() = default;

  /// Sets the information describing the dynamics of the physical system.
  /// Derived classes will keep a reference to the input `data` and therefore it
  /// is required that it outlives this object.
  virtual void SetSystemDynamicsData(const SystemDynamicsData<T>* data) = 0;

  /// Sets the information describing the set of possible discrete contacts.
  /// Derived classes will keep a reference to the input `data` and therefore it
  /// is required that it outlives this object.
  virtual void SetPointContactData(const PointContactData<T>* data) = 0;

  /// Returns the number of generalized velocities in accordance to the data
  /// specified by the last call to SetSystemDynamicsData().
  virtual int num_velocities() const = 0;

  /// Returns the number of contacts in accordance to the data specified by the
  /// last call to SetPointContactData().
  virtual int num_contacts() const = 0;

  /// Generic interface to invoke the contact solver given an initial guess
  /// `v_guess`. Some solvers might decide to ingore this guess, refer to each
  /// solver specific documentation to find out how this gets used.
  /// `dt` is the time step used by the physics engine to advance the solution
  /// in time. In some solvers, `dt` is often needed for the computation of
  /// Baumgarte-like stabilization terms.
  // TODO(amcastro-tri): Consider an API that also provides a guess to contact
  // forces, though usually not possible given the contact set changes as time
  // progresses.
  virtual ContactSolverResult SolveWithGuess(const VectorX<T>& v_guess) = 0;

  /// Retrive contact impulses γ, with units of [Ns]. Of size 3*num_contacts().
  /// Refer to MergeNormalAndTangent() for storage details.
  virtual const VectorX<T>& GetImpulses() const = 0;

  /// Retrive generalized velocities v, with units of [m/s]. Of size
  /// num_velocities().
  virtual const VectorX<T>& GetVelocities() const = 0;

  /// Retrieve generalized contact impulses τc = Jcᵀ⋅γ, of size
  /// num_velocities().
  virtual const VectorX<T>& GetGeneralizedContactImpulses() const = 0;

  /// Retrive contact velocities vc = Jc⋅v, with units of [m/s]. Of size
  /// num_velocities().
  virtual const VectorX<T>& GetContactVelocities() const = 0;

  /// Returns a copy to the vector π of normal impulses, with units of [Ns]. Of
  /// size num_contacts().
  /// Refer to ExtractNormal() for storage details.
  void CopyNormalImpulses(VectorX<T>* pi) const {
    DRAKE_DEMAND(pi != nullptr);
    ExtractNormal(GetImpulses(), pi);
  }

  /// Returns a copy to the vector β of normal impulses, with units of [Ns]. Of
  /// 2*size num_contacts().
  /// Refer to ExtractTangent() for storage details.
  void CopyFrictionImpulses(VectorX<T>* beta) const {
    DRAKE_DEMAND(beta != nullptr);
    ExtractTangent(GetImpulses(), beta);
  }

  /// Retrives the vector of normal contact velocities vn, of size
  /// num_contacts(). Refer to ExtractNormal() for storage details.
  void CopyNormalContactVelocities(VectorX<T>* vn) const {
    DRAKE_DEMAND(vn != nullptr);
    ExtractNormal(GetContactVelocities(), vn);
  }

  /// Retrives the vector of tangent contact velocities vt, of size
  /// 2*num_contacts().
  /// Refer to ExtractTangent() for storage details.
  void CopyTangentialContactVelocities(VectorX<T>* vt) const {
    DRAKE_DEMAND(vt != nullptr);
    ExtractTangent(GetContactVelocities(), vt);
  }

 protected:
  /// Helper method to form the Delassus operator. Most solvers will need to
  /// form it whether if used directly, as part of a pre-processing stage or to
  /// just determine scaling factors.
  ///
  /// Computes W = G * Mi * Jᵀ each j-th column at a time by multiplying with
  /// basis vector ej (zero vector with a "1" at the j-th element).
  ///
  /// @pre G must have size 3nc x nv.
  /// @pre Mi must have size nv x nv.
  /// @pre J must have size 3nc x nv.
  /// @pre J must provide an implementation to MultiplyByTranspose().
  /// @pre W is not nullptr and is of size 3nc x 3nc.
  void FormDelassusOperatorMatrix(const LinearOperator<T>& G,
                                  const LinearOperator<T>& Mi,
                                  const LinearOperator<T>& J,
                                  Eigen::SparseMatrix<T>* W) const {
    DRAKE_DEMAND(G.rows() == 3 * num_contacts());
    DRAKE_DEMAND(G.cols() == num_velocities());
    DRAKE_DEMAND(Mi.rows() == num_velocities());
    DRAKE_DEMAND(Mi.cols() == num_velocities());
    DRAKE_DEMAND(J.rows() == 3 * num_contacts());
    DRAKE_DEMAND(J.cols() == num_velocities());
    DRAKE_DEMAND(W->rows() == 3 * num_contacts());
    DRAKE_DEMAND(W->cols() == 3 * num_contacts());

    const int nv = num_velocities();
    const int nc = num_contacts();

    Eigen::SparseVector<T> ej(3 * nc);
    // ei.makeCompressed();   // Not available for SparseVector.
    ej.coeffRef(0) = 1.0;  // Effectively allocate one non-zero entry.

    Eigen::SparseVector<T> JTcolj(nv);
    Eigen::SparseVector<T> MiJTcolj(nv);
    Eigen::SparseVector<T> Wcolj(3 * nc);
    // Reserve maximum number of non-zeros.
    JTcolj.reserve(nv);
    MiJTcolj.reserve(nv);
    Wcolj.reserve(3 * nc);

    // Loop over the j-th column.
    for (int j = 0; j < W->cols(); ++j) {
      // By changing the inner index, we change what entry is the non-zero with
      // value 1.0.
      *ej.innerIndexPtr() = j;

      // Reset to nnz = 0. Memory is not freed.
      JTcolj.setZero();
      MiJTcolj.setZero();
      Wcolj.setZero();

      // Apply each operator in sequence.
      J.MultiplyByTranspose(ej, &JTcolj);  // JTcolj = JT * ej
      Mi.Multiply(JTcolj, &MiJTcolj);
      G.Multiply(MiJTcolj, &Wcolj);
      W->col(j) = Wcolj;
    }
  }
};
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::ContactSolver)
