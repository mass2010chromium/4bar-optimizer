function retval = C_load(theta) 
  t = rad2deg(theta);
  x = 2.25;
  y = 0.55;
  cp = -0.0446 * t + 4.3058;
  force = [0.2484 * t + 10.101, - (0.4486 * t + 12.556), 0];
  retval = momentvstheta2(x, y, theta, cp, force);
  %retval = 1;
  return;
end
