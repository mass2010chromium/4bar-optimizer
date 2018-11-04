function retval = rotateVector(vector, normal, radians)
  b1=unit(vector);
  b2=unit(cross(normal, vector));
  retval=norm(vector)*(b1*cos(radians)+b2*sin(radians));
  return;
endfunction
