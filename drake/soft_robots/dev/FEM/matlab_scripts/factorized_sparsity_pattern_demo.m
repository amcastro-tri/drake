
% Generate a mesh.
[x y tri] = gen_rectangle_mesh(2, 1, 20, 20);

% Assemble pattern into a sparse matrix.
S = assemble_sparsity_matrix(tri, -1);

% Make a postive definite matrix from S.
A = S + S';
d = abs(sum(A)) + 1;
A = A + diag(sparse(d));

% Verify the matrix is positive definite:
[~,p] = chol(A);
if p~= 0
   ' Matrix is not SPD!'
  return
end

figure

subplot(2,2,1);
L = chol(A,'lower');
spy(L)
title(['Original sparsity pattern. nnz = ' int2str(nnz(L))])
ax1 = gca;

r = symrcm(A);
subplot(2,2,2);
L = chol(A(r,r),'lower');
spy(L)
title(['Reverse Cuthill-McKee. nnz = ' int2str(nnz(L))])
ax2 = gca;

r = colperm(A);
subplot(2,2,3);
L = chol(A(r,r),'lower');
spy(L)
title(['COLPERM: Column count reordering. nnz = ' int2str(nnz(L))])
ax3 = gca;

r = symamd(A);
subplot(2,2,4);
L = chol(A(r,r),'lower');
spy(L)
title(['SYMAMD: Approximate minimum degree. nnz = ' int2str(nnz(L))])
ax4 = gca;

