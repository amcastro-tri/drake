#include "drake/geometry/proximity/inflate_mesh.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <iostream>

#include "drake/common/fmt_eigen.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace geometry {
namespace internal {
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

#if 0
namespace {

/* Implements the QP program to inflate a mesh by a given margin amount.
 This can be written as:
   min 1/2‖u‖|²
   s.t. uᵢ⋅n̂ₐ ≥ δ
 where for each i-th surface we add a linear constraint involving
 all adjacent faces to vertex i with normal n̂ₐ. */
class InflateProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InflateProgram);

  // Create an optimization program to inflate `mesh` and amount `margin.`
  // @note `mesh` is aliased and must remain valid for the lifetime of this
  // class.
  // @pre mesh is not nullptr.
  InflateProgram(const VolumeMesh<double>* mesh, double margin);

  VolumeMesh<double> Solve() const;

 private:
  // Makes a dimensionless version of program described in the class's
  // documentation to minimize round-off errors. We use the margin δ to define a
  // dimensionless displacement ũ = u/δ, and write the dimensionless program as:
  //   min 1/2‖ũ‖|²
  //   s.t. ũᵢ⋅n̂ₐ ≥ 1
  //
  // N.B. The program is "separable", i.e. we could solve the displacement for
  // each surface vertex separately, independently of all other vertices. That
  // strategy could be considered for instance for thread parallelization.
  void MakeProgram();

  const VolumeMesh<double>& mesh_;
  double margin_{0.0};
  std::vector<std::unique_ptr<solvers::MathematicalProgram>> prog_;
  std::vector<VectorX<symbolic::Variable>> u_;  // displacements
  // Map from surface indexes (i.e. indexes into u) to vertices in the original
  // volume mesh, i.e. mesh_.
  std::vector<int> surface_to_volume_vertices_;
};

InflateProgram::InflateProgram(const VolumeMesh<double>* mesh, double margin)
    : mesh_{*mesh}, margin_(margin) {
  DRAKE_DEMAND(margin >= 0);
  DRAKE_DEMAND(mesh != nullptr);
  if (margin > 0) MakeProgram();
}

void InflateProgram::MakeProgram() {  
  const TriangleSurfaceMesh<double> mesh_surface =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          mesh_, &surface_to_volume_vertices_);
  const int num_surface_vertices = mesh_surface.num_vertices();
  DRAKE_DEMAND(ssize(surface_to_volume_vertices_) == num_surface_vertices);

  const int num_vars = 3;
  prog_.resize(num_surface_vertices);
  u_.resize(num_surface_vertices);
  for (int v = 0; v < num_surface_vertices; ++v) {
    auto& p = prog_[v];
    p = std::make_unique<solvers::MathematicalProgram>();
    u_[v] = p->NewContinuousVariables(num_vars);
    p->AddQuadraticCost(MatrixXd::Identity(num_vars, num_vars),
                        VectorXd::Zero(num_vars), u_[v], true /* it is convex */);
  }

  // Determine adjacent faces to each vertex on the surface.
  std::vector<std::vector<int>> adjacent_faces(
      num_surface_vertices);  // indexed by surface vertex.
  for (int e = 0; e < mesh_surface.num_elements(); ++e) {
    const SurfaceTriangle& triangle = mesh_surface.element(e);
    for (int i = 0; i < 3; ++i) {
      const int surface_vertex = triangle.vertex(i);
      adjacent_faces[surface_vertex].push_back(e);
    }
  }

  // For each surface vertex we add as many linear constraints as adjacent
  // faces to that vertex. These constraints enforce that the mesh actually
  // inflates (avoiding deflation regions).
  for (int v = 0; v < num_surface_vertices; ++v) {
    const std::vector<int>& faces = adjacent_faces[v];

    // One linear constraint per face.
    const int num_faces = faces.size();
    const VectorXd lb = VectorXd::Ones(num_faces);
    const VectorXd ub =
        VectorXd::Constant(num_faces, std::numeric_limits<double>::infinity());
    MatrixXd A(num_faces, 3);
    for (int f = 0; f < num_faces; ++f) {
      const Vector3d& normal = mesh_surface.face_normal(faces[f]);
      A.row(f) = normal.transpose();
    }
    auto& p = prog_[v];
    if (v==0 || v==68) {
      std::cout << fmt::format("v: {}. p: {}. A:\n {}\n", v, fmt_eigen(mesh_surface.vertex(v).transpose()), fmt_eigen(A));
    }
    p->AddLinearConstraint(A, lb, ub, u_[v]);
  }
}

VolumeMesh<double> InflateProgram::Solve() const {
  if (margin_ == 0) return mesh_;

  const int num_surf_vertices =  ssize(surface_to_volume_vertices_);
  VectorXd u = VectorXd::Zero(3 * num_surf_vertices);

  // N.B. By experimentation with meshes of different complexity, we determined
  // that Clarabel performed best in terms of both accuracy and computational
  // performance using solver default parameters.
  solvers::ClarabelSolver solver;
  bool failure = false;
  for (int v = 0; v < num_surf_vertices; ++v) {
    auto& p = prog_[v];
    const solvers::MathematicalProgramResult result = solver.Solve(*p);    
    if (!result.is_success()) {
      const solvers::ClarabelSolver::Details& details = result.get_solver_details<solvers::ClarabelSolver>();
      std::cout << fmt::format("Program fails for v: {}. Stat: {}\n", v, details.status);
      failure = true;      
    }
    // The solution corresponds to the dimensionless displacements ũ for each
    // vertex of the input volume mesh. Scaling by the margin, gives us the
    // displacements u.
    u.segment<3>(3 * v) = margin_ * result.get_x_val();
  }
  if (failure) {
    throw std::runtime_error(
          "Failure to inflate mesh. Unless there is a bug, the procedure to "
          "apply margins to non-convex meshes is guaranteed to succeed. You "
          "might also want to check your volume mesh is not somehow "
          "degenerate. "
          "Otherwise, please open a Drake issue.");
  }

  // First copy all vertices.
  std::vector<Vector3d> vertices = mesh_.vertices();

  // Apply displacement to each surface vertex.
  for (int s = 0; s < ssize(surface_to_volume_vertices_); ++s) {
    const int v = surface_to_volume_vertices_[s];
    vertices[v] += u.segment<3>(3 * s);
  }

  std::vector<VolumeElement> tetrahedra = mesh_.tetrahedra();
  return VolumeMesh<double>(std::move(tetrahedra), std::move(vertices));
}

}  // namespace
#endif

std::unique_ptr<solvers::MathematicalProgram> MakeVertexProgram(
    const TriangleSurfaceMesh<double>& surface,
    const std::vector<int>& incident_faces) {
  auto prog = std::make_unique<solvers::MathematicalProgram>();
  const int nv = 3;
  auto u = prog->NewContinuousVariables(nv);
  prog->AddQuadraticCost(MatrixXd::Identity(nv, nv), VectorXd::Zero(nv), u,
                        true /* it is convex */);
  // For each surface vertex we add as many linear constraints as incident
  // faces to that vertex. These constraints enforce that the mesh actually
  // inflates (avoiding deflation regions).
  // One linear constraint per face.
  const int num_faces = incident_faces.size();
  const VectorXd lb = VectorXd::Ones(num_faces);
  const VectorXd ub =
      VectorXd::Constant(num_faces, std::numeric_limits<double>::infinity());
  MatrixXd A(num_faces, 3);
  for (int f = 0; f < num_faces; ++f) {
    const Vector3d& normal = surface.face_normal(incident_faces[f]);
    A.row(f) = normal.transpose();
  }
  prog->AddLinearConstraint(A, lb, ub, u);

  return prog;
}

VolumeMesh<double> MakeInflatedMesh(const VolumeMesh<double>& mesh,
                                    double margin) {
  DRAKE_THROW_UNLESS(margin >= 0);

  // Surface mesh and map to volume vertices.
  std::vector<int> surface_to_volume_vertices;
  const TriangleSurfaceMesh<double> mesh_surface =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          mesh, &surface_to_volume_vertices);
  const int num_surface_vertices = mesh_surface.num_vertices();
  DRAKE_DEMAND(ssize(surface_to_volume_vertices) == num_surface_vertices);

  // Determine adjacent faces to each vertex on the surface.
  std::vector<std::vector<int>> adjacent_faces(
      num_surface_vertices);  // indexed by surface vertex.
  for (int e = 0; e < mesh_surface.num_elements(); ++e) {
    const SurfaceTriangle& triangle = mesh_surface.element(e);
    for (int i = 0; i < 3; ++i) {
      const int surface_vertex = triangle.vertex(i);
      adjacent_faces[surface_vertex].push_back(e);
    }
  }

  std::vector<Vector3d> u(num_surface_vertices);

  // Attempt to solve QP for each surface vertex.
  bool failure = false;
  for (int s = 0; s < num_surface_vertices; ++s) {
    const std::vector<int>& faces = adjacent_faces[s];
    std::unique_ptr<solvers::MathematicalProgram> prog =
        MakeVertexProgram(mesh_surface, faces);

    solvers::ClarabelSolver solver;
    const solvers::MathematicalProgramResult result = solver.Solve(*prog);
    if (!result.is_success()) {
      failure = true;
      const solvers::ClarabelSolver::Details& details =
          result.get_solver_details<solvers::ClarabelSolver>();
      std::cout << fmt::format("Program fails for v: {}. Stat: {}\n", s,
                               details.status);
    }
    // The solution corresponds to the dimensionless displacements ũ for each
    // vertex of the input volume mesh. Scaling by the margin, gives us the
    // displacements u.
    u[s] = margin * result.get_x_val();
  }

  if (failure) {
    throw std::runtime_error(
          "Failure to inflate mesh. Unless there is a bug, the procedure to "
          "apply margins to non-convex meshes is guaranteed to succeed. You "
          "might also want to check your volume mesh is not somehow "
          "degenerate. "
          "Otherwise, please open a Drake issue.");
  }  

  // Apply displacement to each surface vertex.
  std::vector<Vector3d> vertices = mesh.vertices();
  for (int s = 0; s < num_surface_vertices; ++s) {
    const int v = surface_to_volume_vertices[s];
    vertices[v] += u[s];
  }

  std::vector<VolumeElement> tetrahedra = mesh.tetrahedra();
  return VolumeMesh<double>(std::move(tetrahedra), std::move(vertices));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
