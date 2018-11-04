function retval = momentvstheta2(pivotX, pivotY, AOA, cp_pos, force)
  r = rotateVector([cp_pos - pivotX, pivotY, 0], [0,0,1], AOA);
  retval = cross(r, force);
  return;
endfunction
