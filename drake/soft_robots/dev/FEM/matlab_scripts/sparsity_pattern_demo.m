
% Generate a mesh.
[x y tri] = gen_rectangle_mesh(2, 1, 20, 20);

% Assemble pattern into a sparse matrix.
S = assemble_sparsity_matrix(tri, 1.0);

figure

subplot(2,2,1);
spy(S)
title(['Original sparsity pattern. nnz = ' int2str(nnz(S))])
ax1 = gca;

r = symrcm(S);
subplot(2,2,2);
spy(S(r,r))
title(['Reverse Cuthill-McKee. nnz = ' int2str(nnz(S))])
ax2 = gca;

r = colperm(S);
subplot(2,2,3);
spy(S(r,r))
title(['COLPERM: Column count reordering. nnz = ' int2str(nnz(S))])
ax3 = gca;

r = symamd(S);
subplot(2,2,4);
spy(S(r,r))
title(['SYMAMD: Approximate minimum degree. nnz = ' int2str(nnz(S))])
ax4 = gca;

