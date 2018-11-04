function retval = B_load(theta) 
  t = rad2deg(theta);
  x = 2.33;
  y = 0.55;
  cp = -0.0575 * t + 4.1736;
  force = [0.3848 * t + 6.4, - (0.2737 * t + 12.467), 0];
  retval = momentvstheta2(x, y, theta, cp, force);
  %retval = 1;
  return;
end
