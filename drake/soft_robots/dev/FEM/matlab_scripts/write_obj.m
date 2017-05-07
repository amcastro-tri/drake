function write_obj(x, y, tri, filename)
% x: column vector containing x coordiantes.
% y: column vector containing y coordiantes.
% tri: matrix of size ntri x 3 where ntri is the number of triangles.
% filename: the name of the file. 
  nnodes = length(x);
  nels = size(tri)(1);
  
  fileID = fopen(filename,'w');

  for v = 1:nnodes      
    fprintf(fileID,'v %14.8f %14.8f %14.8f\n', x(v), y(v), 0.0);
  end

  fprintf(fileID,'f %d %d %d\n', tri');

  fclose(fileID);

