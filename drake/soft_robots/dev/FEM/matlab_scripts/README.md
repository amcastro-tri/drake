# Matlab Scripts

## Simple Mesh Generators
 - **gen_rectangle_mesh.m**: Generates a homogenous triangulation for a rectangular domain of size _Lx x Ly_. Type `help gen_rectangle_mesh` for details.

## Demos

### Sparsity patterns and ordering.

- **sparsity_pattern_demo.m**: visualizes the sparsity pattern of the mass matrix for a rectangular domain generated with `gen_rectangle_mesh.m`. The demo also comparse the sparsity patter obtained by using different ordering of the mesh nodes.

![Mass matrix sparsity pattern](./images/sparsity_pattern.png)