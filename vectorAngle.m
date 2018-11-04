function retval = vectorAngle(A1, A2)
  retval = real(acos(dot(A1, A2)/(norm(A1) * norm(A2))));
  return
endfunction
