function S = assemble_sparsity_matrix(tri, v)
  % S = assemble_sparsity_matrix(tri, v)
  % Given an array of connectivities tri, assemble a sparse matrix so that its 
  % sparsity pattern corresponds to that of a FEM mass matrix.
  % You can view this sparsity pattern with spy(S).
  % v: What value entry to assemble into the matrix.

  nel = size(tri)(1);
  nnodes = max(max(max(tri)));
  nen = 3; % Only for triangles.
  
  ik = zeros(9 * nel, 1);
  jk = zeros(9 * nel, 1);

  k = 1;
  for iel = 1:nel      
    for ia = 1:nen
      for ib = 1:nen
        ik(k) = tri(iel, ia);
        jk(k) = tri(iel, ib);
        k = k + 1;
      end
    end
  end

  S = sparse(ik, jk, v * ones(size(ik)), nnodes, nnodes);
