function retval = propagateMoment(momentV, link1, link2, link3)
  fHat = unit(link2);
  tmp = cross(link1, fHat);
  F = fHat * (momentV(3) / tmp(3));
  retval = cross(link3, F);
  return
end
