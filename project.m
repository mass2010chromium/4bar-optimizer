function retval = project(v1, v2)
  retval = dot(v1, v2) * unit(v1) / norm(v1);
  return;
endfunction
