function [x y tri] = gen_rectangle_mesh(Lx, Ly, nx, ny)
% Mesh the rectangle region [0, Lx] x [0, Ly] using a
% mesh of triangles.
% Lx: length in the x direction.
% Ly: length in the y direction.
% nx: Number of nodes in the x direction.
% nx: Number of nodes in the y direction.
%
% The resulting mesh will contain nx x ny nodes and
% 2 * (nx-1) * (ny-1) triangles.
%
% You can plot a function defined on the regions as, E.g.:
% [x y tri] = gen_rectangle_mesh(4, 4, 20, 20); x = x-2; y=y-2;
% trimesh(tri, x, y, x.*exp(-x.^2-(y).^2))
x1d = linspace(0, Lx, nx);
y1d = linspace(0, Ly, ny);

[X, Y] = meshgrid(x1d, y1d);

nnodes = nx * ny;
x = reshape(X, nnodes, 1);
y = reshape(Y, nnodes, 1);

tri = delaunay(x,y);
