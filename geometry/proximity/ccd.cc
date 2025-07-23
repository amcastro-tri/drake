#include "drake/geometry/proximity/ccd.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/math/real_roots.h"

namespace drake {
namespace geometry {
namespace internal {

namespace {

/*
  Test whether a point lies inside a triangle using barycentric coordinates
  (when the point is assumed to be co-planar with the triangle).

  Let e0 = t1 - t0, e1 = t2 - t0, and r = p - t0.

  This function solves for x = [u, v] that minimizes the squared distance:

    || r - B·x ||^2   where B = [e0; e1]

  This least squares minimizer is given by:

    B·Bᵀ x = B·r

  This implementation builds that 2×2 system and solves it with LDLᵀ.
  If the solution satisfies u ∈ [0,1] and v ∈ [0,1] and u + v < 1, the point p
  lies inside the triangle [t0, t1, t2].
*/
bool is_coplanar_point_inside_triangle(const Vector3d& p,
                                       const Vector3d& t0,
                                       const Vector3d& t1,
                                       const Vector3d& t2) {
  Eigen::Matrix<double, 2, 3> B;
  B.row(0) = t1 - t0;
  B.row(1) = t2 - t0;
  const Eigen::Matrix2d A = B * B.transpose();
  const Eigen::Vector2d b = B * (p - t0);
  const Eigen::Vector2d x = A.ldlt().solve(b);
  return x[0] >= 0 && x[1] >= 0 && x[0] + x[1] <= 1;
}

/*
  Test whether two co-planar 3D segments intersect using the closest-points
  formulation.

  This solves for parameters (s, t) that minimize the squared distance
    || (ea0 + s*(ea1-ea0)) - (eb0 + t*(eb1-eb0)) ||^2
  on the infinite lines, by setting partial derivatives to zero.

  Derivation:
    Let da = ea1 - ea0, db = eb1 - eb0, r = ea0 - eb0.
    Minimize F(s,t) = || r + s*da - t*db ||^2.
    ∂F/∂s = 2(r·da) + 2s(da·da) - 2t(da·db) = 0
    ∂F/∂t = -2(r·db) - 2s(da·db) + 2t(db·db) = 0

    With a = da·da, b = da·db, e = db·db, c = da·r, f = db·r,
    the normal equations are:
        [ a  -b ] [ s ] = [ -c ]
        [ -b  e ] [ t ]   [  f ] .

  This implementation builds that 2×2 system and solves it with LDLᵀ.
  If the solution satisfies s ∈ [0,1] and t ∈ [0,1], the closest points on the
  infinite lines lie within the bounds of both segments.
*/
bool are_coplanar_edges_intersecting(const Vector3d& ea0,
                                     const Vector3d& ea1,
                                     const Vector3d& eb0,
                                     const Vector3d& eb1) {
  const Vector3d eb_to_ea = ea0 - eb0;
  const Vector3d ea = ea1 - ea0;
  const Vector3d eb = eb1 - eb0;

  Eigen::Matrix<double, 2, 2> coefMtr;
  coefMtr(0, 0) = ea.squaredNorm();
  coefMtr(0, 1) = coefMtr(1, 0) = -eb.dot(ea);
  coefMtr(1, 1) = eb.squaredNorm();

  Eigen::Vector2d rhs;
  rhs[0] = -eb_to_ea.dot(ea);
  rhs[1] = eb_to_ea.dot(eb);

  const Eigen::Vector2d x = coefMtr.ldlt().solve(rhs);
  return 0 <= x[0] && x[0] <= 1 && 0 <= x[1] && x[1] <= 1;
}

/*
  Let ([x,y,z] := x⋅(y × z)) be the scalar triple product.
  With A(t) = a + tα, B(t) = b + tβ, C(t) = c + tγ

     A(t)⋅(B(t) × C(t)) = [a + tα, b + tβ, c + tγ] = p₀ + p₁ t + p₂ t² + p₃ t³

  with coefficients:

    p₀ = [a,b,c]                      =  a⋅(b × c),
    p₁ = [α,b,c] + [a,β,c] + [a,b,γ]  =  α⋅(b × c) + a⋅(β × c) + a⋅(b × γ)
    p₂ = [α,β,c] + [α,b,γ] + [a,β,γ]  =  α⋅(β × c) + α⋅(b × γ) + a⋅(β × γ)
    p₃ = [α,β,γ]                      =  α⋅(β × γ)

  simplifying:

    p₀ =  a⋅X₀,
    p₁ =  α⋅X₀ + a⋅X₁
    p₂ =  α⋅X₁ + a⋅X₂
    p₃ =  α⋅X₂

  Where:
    X₀ = (b × c)
    X₁ = (β × c) + (b × γ)
    X₂ = (β × γ)
*/
std::array<double, 4> cubic(const Vector3d& a,
                            const Vector3d& b,
                            const Vector3d& c,
                            const Vector3d& alpha,
                            const Vector3d& beta,
                            const Vector3d& gamma) {
  const Vector3d X0 = b.cross(c);
  const Vector3d X1 = beta.cross(c) + b.cross(gamma);
  const Vector3d X2 = beta.cross(gamma);
  return {a.dot(X0), alpha.dot(X0) + a.dot(X1), alpha.dot(X1) + a.dot(X2),
          alpha.dot(X2)};
}

}  // namespace

/*
  Given a point p(t) and a triangle [v₀(t), v₁(t), v₂(t)],
  where the parameteric form of each point is:

    p(t)  = p₀  + t⋅(p₁ - p₀)
    v₀(t) = v₀₀ + t⋅(v₀₁ - v₀₀)
    v₁(t) = v₁₀ + t⋅(v₁₁ - v₁₀)
    v₂(t) = v₂₀ + t⋅(v₂₁ - v₂₀)

  If the p(t) intesects the triangle [v₀(t), v₁(t), v₂(t)] on t = [0, 1],
  then it must happen when all points are co-planar. In other words:

    A(t)⋅(B(t) × C(t)) = 0

  Where:

    A(t) =  p(t) - v₀(t) = a + t⋅α
    B(t) = v₁(t) - v₀(t) = b + t⋅β
    C(t) = v₂(t) - v₀(t) = c + t⋅γ

    a = p₀  - v₀₀
    b = v₁₀ - v₀₀
    c = v₂₀ - v₀₀
    α = (p₁ - p₀)   - (v₀₁ - v₀₀)
    β = (v₁₁ - v₁₀) - (v₀₁ - v₀₀)
    γ = (v₂₁ - v₂₀) - (v₀₁ - v₀₀)

  This function:
    - Forms the polynomial A(t)⋅(B(t) × C(t))
    - Solves for the real roots in the interval [0, 1]
    - For each valid root, r, in ascending order:
        if p(r) is inside the triangle [v₀(r), v₁(r), v₂(r)]
          set toi = r
          return true
    - return false
*/
bool point_triangle_ccd(const Vector3d& p0,
                        const Vector3d& v00,
                        const Vector3d& v10,
                        const Vector3d& v20,
                        const Vector3d& p1,
                        const Vector3d& v01,
                        const Vector3d& v11,
                        const Vector3d& v21, double* toi) {
  const Vector3d delta_p = p1 - p0;
  const Vector3d delta_v0 = v01 - v00;
  const Vector3d delta_v1 = v11 - v10;
  const Vector3d delta_v2 = v21 - v20;
  const Vector3d a = p0 - v00;
  const Vector3d b = v10 - v00;
  const Vector3d c = v20 - v00;
  const Vector3d alpha = delta_p - delta_v0;
  const Vector3d beta = delta_v1 - delta_v0;
  const Vector3d gamma = delta_v2 - delta_v0;

  const std::array<double, 4> f = cubic(a, b, c, alpha, beta, gamma);

  for (const double r : math::cubic_real_roots(f[3], f[2], f[1], f[0])) {
    if (r >= 0 && r <= 1) {
      const Vector3d p_r = p0 + r * delta_p;
      const Vector3d v0_r = v00 + r * delta_v0;
      const Vector3d v1_r = v10 + r * delta_v1;
      const Vector3d v2_r = v20 + r * delta_v1;
      if (is_coplanar_point_inside_triangle(p_r, v0_r, v1_r, v2_r)) {
        *toi = r;
        return true;
      }
    }
  }

  return false;
}

/*
  Given edges [p₀(t), p₁(t)] and [q₀(t), q₁(t)]
  where the parameteric form of each point is:

    p₀(t) = p₀₀ + t⋅(p₀₁ - p₀₀)
    p₁(t) = p₁₀ + t⋅(p₁₁ - p₁₀)
    q₀(t) = q₀₀ + t⋅(q₀₁ - q₀₀)
    q₁(t) = q₁₀ + t⋅(q₁₁ - q₁₀)

  If the edges intesect for some t = [0, 1], then it must happen when all points
  are co-planar. In other words:

    A(t)⋅(B(t) × C(t)) = 0

  Where:

    A(t) = p₁(t) - p₀(t) = a + t⋅α
    B(t) = q₀(t) - p₀(t) = b + t⋅β
    C(t) = q₁(t) - p₀(t) = c + t⋅γ

    a = p₁₀ - p₀₀
    b = q₀₀ - p₀₀
    c = q₁₀ - p₀₀
    α = (p₁₁ - p₁₀) - (p₀₁ - p₀₀)
    β = (q₀₁ - q₀₀) - (p₀₁ - p₀₀)
    γ = (q₁₁ - q₁₀) - (p₀₁ - p₀₀)

  This function:
    - Forms the polynomial A(t)⋅(B(t) × C(t))
    - Solves for the real roots in the interval [0, 1]
    - For each valid root, r, in ascending order:
        if the edges intersect at r
          set toi = r
          return true
    - return false
*/
bool edge_edge_ccd(const Vector3d& p00,
                   const Vector3d& p10,
                   const Vector3d& q00,
                   const Vector3d& q10,
                   const Vector3d& p01,
                   const Vector3d& p11,
                   const Vector3d& q01,
                   const Vector3d& q11, double* toi) {
  const Vector3d delta_p0 = p01 - p00;
  const Vector3d delta_p1 = p11 - p10;
  const Vector3d delta_q0 = q01 - q00;
  const Vector3d delta_q1 = q11 - q10;
  const Vector3d a = p10 - p00;
  const Vector3d b = q00 - p00;
  const Vector3d c = q10 - p00;
  const Vector3d alpha = delta_p1 - delta_p0;
  const Vector3d beta = delta_q0 - delta_p0;
  const Vector3d gamma = delta_q1 - delta_p0;

  const std::array<double, 4> f = cubic(a, b, c, alpha, beta, gamma);

  for (const double r : math::cubic_real_roots(f[3], f[2], f[1], f[0])) {
    if (r >= 0 && r <= 1) {
      const Vector3d p0_r = p00 + r * delta_p0;
      const Vector3d p1_r = p10 + r * delta_p1;
      const Vector3d q0_r = q00 + r * delta_q0;
      const Vector3d q1_r = q10 + r * delta_q1;
      if (are_coplanar_edges_intersecting(p0_r, p1_r, q0_r, q1_r)) {
        *toi = r;
        return true;
      }
    }
  }

  return false;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
