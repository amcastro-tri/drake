#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
    namespace geometry {
        namespace internal {

// Helper methods for MakeCylinderMesh().
#ifndef DRAKE_DOXYGEN_CXX

typedef enum {
    VERTEX_INTERNAL,
    VERTEX_CAP,
    VERTEX_SIDE
} CylinderVertexType;


//Project the point p to the side of the cylinder in the XY direction
template <typename T>
Vector3<T> ProjectOntoCylinderSide(const Vector3<T>& p) {
    Vector3<T> p_proj = Vector3<T>(p[0], p[1], 0.0);
    auto len = p_proj.norm();
    p_proj.normalize();
    return p + (1 - len)*p_proj;
}

//Project midpoint of two cap vertices so that it's length from the medial axis is the
//mean of the two vertices lengths from the medial axis.
template <typename T>
Vector3<T> ProjectOntoCylinderCap(const Vector3<T>& X, const Vector3<T>& Y) {

    Vector3<T> V = (X+Y)/2.0;

    Vector3<T> v_proj = Vector3<T>(V[0], V[1], 0.0);
    Vector3<T> x_proj = Vector3<T>(X[0], X[1], 0.0);
    Vector3<T> y_proj = Vector3<T>(Y[0], Y[1], 0.0);

    auto length = v_proj.norm();
    auto desired_length = (x_proj.norm() + y_proj.norm()) / 2.0;

    v_proj.normalize();

    return V + (desired_length - length)*v_proj;
}

template <typename T>
Vector3<T> chooseProjection(const Vector3<T>& X, const Vector3<T>& Y, CylinderVertexType v_type) {

    //  Project boundary vertices onto the surface of the cylinder.
    const Vector3<T> V = (X+Y)/2.0;

    if (v_type == VERTEX_SIDE) {
        return ProjectOntoCylinderSide(V);
    }
    else if(v_type == VERTEX_CAP) {
        return ProjectOntoCylinderCap(X, Y);
    }
    //TODO (joemasterjohn)
    // Testing this for quality, refactor when decided on right methodology
    else if(v_type == VERTEX_INTERNAL) {
        return ProjectOntoCylinderCap(X, Y);
    } else {
        return V;
    }
}

// Refines a tetrahedron defined by vertices A, B, C, and D
// into 8 new tetrahedra. Vector is_boundary stores the CylinderVertexType, which
// determines if and how the generated vertices are projected onto the boundary.
// The return is a pair with: 1) a VolumeMesh including the original and
// new vertices, and 2) a vector of booleans that, similar to is_boundary,
// storing the CylinderVertexType of all vertices.
template <typename T>
std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>> RefineCylinderTetrahdron(
        const Vector3<T>& A, const Vector3<T>& B, const Vector3<T>& C,
        const Vector3<T>& D, const std::vector<CylinderVertexType>& is_boundary) {
    // Aliases to the indexes, just to ease writing the code.
    // That is, index a corresponds to vertex A, index b to vertex B, etc.
    const VolumeVertexIndex a(0), b(1), c(2), d(3), e(4), f(5), g(6), h(7), i(8),
            j(9);

    // Each tetrahedra is split into 8 sub-tetrahedra.
    std::vector<VolumeElement> tetrahedra;
    std::vector<VolumeVertex<T>> vertices;  // Original 4 plus 6, one per edge.
    Vector3<T> E = (A + B) / 2.0;
    Vector3<T> F = (A + C) / 2.0;
    Vector3<T> G = (A + D) / 2.0;
    Vector3<T> H = (B + C) / 2.0;
    Vector3<T> I = (B + D) / 2.0;
    Vector3<T> J = (C + D) / 2.0;

    // Mark the boundary type of the vertices.
    std::vector<CylinderVertexType> split_vertex_is_boundary(10);

    //Original vertices remain the same
    split_vertex_is_boundary[a] = is_boundary[a];
    split_vertex_is_boundary[b] = is_boundary[b];
    split_vertex_is_boundary[c] = is_boundary[c];
    split_vertex_is_boundary[d] = is_boundary[d];

    // New vertices inherit from their two parents with min priority.
    // If at least one parent is internal, this vertex is internal
    // If no parent is internal, but at least one is on the cap, this vertex is on the cap
    // New vertex is only on the side of the cylinter if BOTH parents are on the side
    split_vertex_is_boundary[e] = std::min(is_boundary[a], is_boundary[b]);
    split_vertex_is_boundary[f] = std::min(is_boundary[a], is_boundary[c]);
    split_vertex_is_boundary[g] = std::min(is_boundary[a], is_boundary[d]);
    split_vertex_is_boundary[h] = std::min(is_boundary[b], is_boundary[c]);
    split_vertex_is_boundary[i] = std::min(is_boundary[b], is_boundary[d]);
    split_vertex_is_boundary[j] = std::min(is_boundary[c], is_boundary[d]);

    E = chooseProjection(A, B, split_vertex_is_boundary[e]);
    F = chooseProjection(A, C, split_vertex_is_boundary[f]);
    G = chooseProjection(A, D, split_vertex_is_boundary[g]);
    H = chooseProjection(B, C, split_vertex_is_boundary[h]);
    I = chooseProjection(B, D, split_vertex_is_boundary[i]);
    J = chooseProjection(C, D, split_vertex_is_boundary[j]);

    // Place the new vertices in the vector of vertices.
    vertices.emplace_back(A);
    vertices.emplace_back(B);
    vertices.emplace_back(C);
    vertices.emplace_back(D);
    vertices.emplace_back(E);
    vertices.emplace_back(F);
    vertices.emplace_back(G);
    vertices.emplace_back(H);
    vertices.emplace_back(I);
    vertices.emplace_back(J);

    // The four tetrahedra at the corners.
    tetrahedra.emplace_back(a, e, f, g);
    tetrahedra.emplace_back(b, h, e, i);
    tetrahedra.emplace_back(f, h, c, j);
    tetrahedra.emplace_back(j, g, i, d);

    // TODO(joemasterjohn): Split along EJ, FI and GH and choose the partition with
    // best quality factor as described in [Everett, 1997], see the class's
    // documentation.
    // Currently we arbitrarily choose to partition along the GH edge.
    auto split_along_gh = [&]() {
        std::vector<VolumeElement> inner_tets;
        inner_tets.emplace_back(g, h, i, e);
        inner_tets.emplace_back(g, f, h, e);
        inner_tets.emplace_back(g, i, h, j);
        inner_tets.emplace_back(g, h, f, j);
        return inner_tets;
    };

    // Split the internal octahedron EFGHIJ into four sub-tetrahedra.
    auto inner_tets = split_along_gh();
    std::copy(inner_tets.begin(), inner_tets.end(), back_inserter(tetrahedra));

    return std::make_pair(
            VolumeMesh<T>(std::move(tetrahedra), std::move(vertices)),
            split_vertex_is_boundary);
}

// Makes the initial mesh for refinement_level = 0.
// TODO(joemasterjohn): Document this much better for the complicated indexing scheme
template <typename T>
std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>> MakeCylinderMeshLevel0(const double& height, const double& radius) {
    std::vector<VolumeElement> tetrahedra;
    std::vector<VolumeVertex<T>> vertices;

    std::size_t subdivisions = static_cast<std::size_t>(std::max(1.0, std::floor(height / radius)));

    const double top_z = (height/2.0);
    const double bot_z = -(height/2.0);

    for(std::size_t i = 0; i <= subdivisions; i++) {

        const double t = (1.0*i) / subdivisions;

        const double z = (1-t)*top_z + (t)*bot_z;

        vertices.emplace_back( radius,     0.0, z);   // vi + 0
        vertices.emplace_back(    0.0,  radius, z);   // vi + 1
        vertices.emplace_back(-radius,     0.0, z);   // vi + 2
        vertices.emplace_back(    0.0, -radius, z);   // vi + 3
        vertices.emplace_back(    0.0,     0.0, z);   // vi + 4
    }

    using V = VolumeVertexIndex;

    for(std::size_t j = 0; j < subdivisions; j++) {
        for (std::size_t i = 0; i < 4; i++) {

            const std::size_t a = 5*j + i;
            const std::size_t b = 5*j + ((i+1)%4);
            const std::size_t c = 5*j + 4;
            const std::size_t d = 5*(j+1) + i;
            const std::size_t e = 5*(j+1) + ((i+1)%4);
            const std::size_t f = 5*(j+1) + 4;

            tetrahedra.emplace_back(V(a), V(c), V(b), V(f));
            tetrahedra.emplace_back(V(a), V(b), V(e), V(f));
            tetrahedra.emplace_back(V(a), V(e), V(d), V(f));
        }
    }

    //Most vertices start on the side
    //Two are cap vertices
    //There are subdivisions - 2 interval vertices
    std::vector<CylinderVertexType> is_boundary(5*(subdivisions+1), VERTEX_SIDE);
    is_boundary[4] = VERTEX_CAP;
    is_boundary[5*subdivisions + 4] = VERTEX_CAP;

    for(std::size_t i = 1; i < subdivisions; i++) {
        is_boundary[5*i + 4] = VERTEX_INTERNAL;
    }

    return std::make_pair(
            VolumeMesh<T>(std::move(tetrahedra), std::move(vertices)), is_boundary);
}

// Splits a mesh by calling RefineCylinderTetrahdron() on each tetrahedron of `mesh`.
// `is_boundary` is a vector describing the CylinderVertexType of each vertex in `mesh`
template <typename T>
std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>> RefineCylinderMesh(
        const VolumeMesh<T>& mesh, const std::vector<CylinderVertexType>& is_boundary) {
    std::vector<VolumeElement> split_mesh_tetrahedra;
    std::vector<VolumeVertex<T>> split_mesh_vertices;
    std::vector<CylinderVertexType> split_is_boundary;
    for (const auto& t : mesh.tetrahedra()) {
        const auto& A = mesh.vertex(t.vertex(0)).r_MV();
        const auto& B = mesh.vertex(t.vertex(1)).r_MV();
        const auto& C = mesh.vertex(t.vertex(2)).r_MV();
        const auto& D = mesh.vertex(t.vertex(3)).r_MV();

        // Vector that stores the CylinderVertexType of A, B, C, and D
        std::vector<CylinderVertexType> tet_is_boundary;
        tet_is_boundary.push_back(is_boundary[t.vertex(0)]);
        tet_is_boundary.push_back(is_boundary[t.vertex(1)]);
        tet_is_boundary.push_back(is_boundary[t.vertex(2)]);
        tet_is_boundary.push_back(is_boundary[t.vertex(3)]);

        const std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>> split_pair =
                RefineCylinderTetrahdron<T>(A, B, C, D, tet_is_boundary);
        const VolumeMesh<T>& split_tet = split_pair.first;
        const std::vector<CylinderVertexType>& split_tet_is_boundary = split_pair.second;
        const int num_vertices = static_cast<int>(split_mesh_vertices.size());

        std::copy(split_tet.vertices().begin(), split_tet.vertices().end(),
                  back_inserter(split_mesh_vertices));
        std::copy(split_tet_is_boundary.begin(), split_tet_is_boundary.end(),
                  back_inserter(split_is_boundary));

        // Add the new tets. Offset the vertex indexes to account for the indexing
        // within the full mesh.
        std::transform(split_tet.tetrahedra().begin(), split_tet.tetrahedra().end(),
                       back_inserter(split_mesh_tetrahedra),
                       [num_vertices](const VolumeElement& tet) {
                           using V = VolumeVertexIndex;
                           return VolumeElement(V(tet.vertex(0) + num_vertices),
                                                V(tet.vertex(1) + num_vertices),
                                                V(tet.vertex(2) + num_vertices),
                                                V(tet.vertex(3) + num_vertices));
                       });
    }

    return std::make_pair(VolumeMesh<T>(std::move(split_mesh_tetrahedra),
                                        std::move(split_mesh_vertices)),
                          split_is_boundary);
}
#endif

/// This method implements a variant of the generator described in
/// [Everett, 1997]. It is based on a recursive refinement of an initial
/// (refinement_level = 0) coarse mesh representation of a cylinder with
/// given height and radius. The initial mesh discretizes a rectangular
/// prism, subdividing to make roughly regular tetrahedra
/// At each refinement level each tetrahedron is split into eight new tetrahedra by splitting
/// each edge in half. When splitting an edge formed by vertices on the
/// surface of the sphere, the newly created vertex is projected back onto the
/// surface of the sphere.
///
/// @throws std::exception if refinement_level is negative.
/// @throws std::exception if height is non-positive.
/// @throws std::exception if radius is non-positive.
///
/// [Everett, 1997]  Everett, M.E., 1997. A three-dimensional spherical mesh
/// generator. Geophysical Journal International, 130(1), pp.193-200.
template <typename T>
VolumeMesh<T> MakeCylinderMesh(double height, double radius, int refinement_level) {

    DRAKE_THROW_UNLESS(height > 0);
    DRAKE_THROW_UNLESS(radius > 0);
    DRAKE_THROW_UNLESS(refinement_level >= 0);

    std::pair<VolumeMesh<T>, std::vector<CylinderVertexType>> pair = MakeCylinderMeshLevel0<T>(height, radius);
    VolumeMesh<T>& mesh = pair.first;
    std::vector<CylinderVertexType>& is_boundary = pair.second;

    for (int level = 1; level <= refinement_level; ++level) {
        auto split_pair = RefineCylinderMesh<T>(mesh, is_boundary);
        mesh = split_pair.first;
        is_boundary = split_pair.second;
        DRAKE_DEMAND(mesh.vertices().size() == is_boundary.size());
    }

    return std::move(mesh);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
