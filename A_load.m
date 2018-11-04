function retval = A_load(theta) 
  t = rad2deg(theta);
  x = 1.9;
  y = 0.5;
  cp = -0.0407 * t + 2.7028;
  force = [0.3418 * t + 5.1171, - (0.6475 * t + 12.269), 0];
  retval = momentvstheta2(x, y, theta, cp, force);
  %retval = 1;
  return;
end
