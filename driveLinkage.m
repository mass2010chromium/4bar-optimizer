function retval = driveLinkage(previous, driverPt, driverR, drivenPt, drivenR)
  points=circleIntersection( driverPt,driverR, drivenPt,drivenR );
  x1=points{1};
  x2=points{2};
  delta1=norm(x1-previous);
  delta2=norm(x2-previous);
  if (delta1 < delta2)
    retval = x1;
    return;
  else
    retval = x2;
    return;
  end
end
