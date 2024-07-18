#include "drake/geometry/proximity/inflate_mesh.h"

#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/clarabel_solver.h"

#include <iostream>

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

class InflateProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InflateProgram);

  // Crate an optimization program to inflate `mesh` and amount `margin.`
  explicit InflateProgram(VolumeMesh<double> mesh, double margin);

  VolumeMesh<double> Solve() const;

 private:
#if 0
  class VolumeConstraint : public Constraint {
   public:
    // Volume constraint for the i-th vertex.
    VolumeConstraint(int i);

   private:
    template <typename T>
    void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                       VectorX<T>* y) const {
      using std::exp;
      y->resize(2);
      (*y)(0) = x(1) - exp(x(0));
      (*y)(1) = x(2) - exp(x(1));
    }

    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                AutoDiffVecXd* y) const override {
      DoEvalGeneric(x, y);
    }

    void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
                VectorX<symbolic::Expression>* y) const override {
      DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
    }
  };
#endif

  VolumeMesh<double> mesh_;
  double margin_{0.0};
  std::unique_ptr<solvers::MathematicalProgram> prog_;
  VectorX<symbolic::Variable> u_;  // displacements
  std::vector<int> surface_to_volume_vertices_;
};

InflateProgram::InflateProgram(VolumeMesh<double> mesh, double margin)
    : mesh_{std::move(mesh)}, margin_(margin), prog_{new solvers::MathematicalProgram()} {
  const TriangleSurfaceMesh<double> mesh_surface =
      ConvertVolumeToSurfaceMeshWithBoundaryVertices(
          mesh_, &surface_to_volume_vertices_);
  const int num_surface_vertices = mesh_surface.num_vertices();
  DRAKE_DEMAND(ssize(surface_to_volume_vertices_) == num_surface_vertices);

  const int num_vars = 3 * num_surface_vertices;
  u_ = prog_->NewContinuousVariables(num_vars, "u");

  prog_->AddQuadraticCost(MatrixXd::Identity(num_vars, num_vars),
                          VectorXd::Zero(num_vars), u_,
                          true /* it is convex */);

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

  if (margin > 0) {
    // For each surface vertex we add as many linear constraints as adjacent
    // faces to that vertex. These constraints enforce that the mesh actually
    // inflates (avoiding deflation regions).
    for (int v = 0; v < num_surface_vertices; ++v) {
      const std::vector<int>& faces = adjacent_faces[v];

      // One linear constraint per face.
      const int num_faces = faces.size();
      const VectorXd lb = VectorXd::Ones(num_faces);
      const VectorXd ub = VectorXd::Constant(
          num_faces, std::numeric_limits<double>::infinity());
      MatrixXd A(num_faces, 3);
      for (int f = 0; f < num_faces; ++f) {
        const Vector3d& normal = mesh_surface.face_normal(faces[f]);
        A.row(f) = normal.transpose();
      }
      prog_->AddLinearConstraint(A, lb, ub, u_.segment<3>(3 * v));
    }
  }

  // auto constraint =
  // std::make_shared<EckhardtConstraint>(set_sparsity_pattern);
  // prog_->AddConstraint(constraint, x_);
}

VolumeMesh<double> InflateProgram::Solve() const {
  const solvers::SolverId id = ChooseBestSolver(*prog_);      
  std::cout << fmt::format("Best solver: {}\n", id.name());  

  solvers::GurobiSolver solver;  
  //solvers::MosekSolver solver;  
  //solvers::ClarabelSolver solver;  
  //solvers::IpoptSolver solver;
  //solvers::NloptSolver solver;
  const solvers::MathematicalProgramResult result =  solver.Solve(*prog_);
  //const solvers::MathematicalProgramResult result = solvers::Solve(*prog_);
  if (!result.is_success()) {
    throw std::runtime_error("Failure to inflate mesh.");
  }

  if (margin_ == 0) return mesh_;

  // The solution corresponds to the displacements u for each vertex of the
  // input volume mesh.
  // N.B. The solution is computed in dimensionless units to reduce round-off
  // errors.
  const VectorXd u = margin_ * result.get_x_val();


  // First copy all vertices.
  std::vector<Vector3d> vertices = mesh_.vertices();

  // Apply displacement to each surface vertex.
  for (int s = 0; s < ssize(surface_to_volume_vertices_); ++s) {
    const int v = surface_to_volume_vertices_[s];
    vertices[v] += u.segment<3>(3 * v);
  }

  std::vector<VolumeElement> tetrahedra = mesh_.tetrahedra();
  return VolumeMesh<double>(std::move(tetrahedra), std::move(vertices));
}

}  // namespace

VolumeMesh<double> MakeInflatedMesh(const VolumeMesh<double>& mesh,
                                    double margin) {
  InflateProgram program(mesh, margin);
  return program.Solve();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
