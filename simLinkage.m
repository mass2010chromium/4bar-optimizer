function retval = simLinkage(steps, A_pos, A_len, A_initial, A_dTheta, A_loadFunc, ...
  B_pos, B_len, B_initial, B_loadFunc, C_pos, C_len, C_initial, C_loadFunc)
  
  a_vector = [thetaR2D(A_initial, A_len), 0];
  b_startVector = [thetaR2D(B_initial, B_len), 0];
  c_startVector = [thetaR2D(C_initial, C_len), 0];
  b_vector = b_startVector;
  c_vector = c_startVector;
  
  lastX_B = B_pos + b_vector;
  lastX_C = C_pos + c_vector;
  
  link_AB = norm(lastX_B - (A_pos + a_vector));
  link_BC = norm(lastX_C - lastX_B);
  
  step = A_dTheta / steps;
  retval = cell(steps + 1, 3);
  %retval = [];
  for i=0:(steps)
    a_relativeAngle = step * i;
    theta = A_initial + a_relativeAngle;
    a_vector = [thetaR2D(theta, A_len), 0];
    newPos_A = A_pos + a_vector;
    
    newPos_B = driveLinkage(lastX_B, newPos_A, link_AB, B_pos, B_len);
    b_vector = newPos_B - B_pos;
    b_relativeAngle = vectorAngle(b_vector, b_startVector);
    arm_BA = newPos_B - newPos_A;
    
    newPos_C = driveLinkage(lastX_C, newPos_B, link_BC, C_pos, C_len);
    c_vector = newPos_C - C_pos;
    c_relativeAngle = vectorAngle(c_vector, c_startVector);
    arm_CB = newPos_C - newPos_B;
    
    c_moment = C_loadFunc(c_relativeAngle);
    b_load = B_loadFunc(b_relativeAngle);
    b_moment = b_load + propagateMoment(c_moment, c_vector, arm_CB, b_vector);
    a_load = A_loadFunc(a_relativeAngle);
    a_moment = propagateMoment(b_moment, b_vector, arm_BA, a_vector);
    
    %if (!isreal(b_startVector))
    %  disp("B starting pos not real!");
    %end
    %if (!isreal(c_startVector))
    %  disp("C starting pos not real!");
    %end
    
    %if (!isreal(b_vector))
    %  disp("B arm not real!");
    %end
    %if (!isreal(c_vector))
    %  disp("C arm not real!");
    %end
    
    %if (!isreal(c_relativeAngle))
    %  disp("C angle not real!");
    %elseif (!isreal(b_relativeAngle))
    %  disp("B angle not real!");
    %elseif (!isreal(c_moment))
    %  disp("C load not real!");
    %elseif (!isreal(b_load))
    %  disp("B load not real!");
    %elseif (!isreal(b_moment))
    %  disp("B propagated load not real!");
    %elseif (!isreal(a_load))
    %  disp("A load not real!");
    %elseif (!isreal(a_moment))
    %  disp("A propagated load not real!");
    %end
    
    
    force = a_moment * link_AB / norm(cross(a_vector, arm_BA));
    
    %retval(i+1) = norm(force);
    retval{i + 1, 1} = norm(force);
    retval{i + 1, 2} = [a_relativeAngle, newPos_A, newPos_B, newPos_C];
    retval{i + 1, 3} = [a_moment(3), b_moment(3), c_moment(3), b_load(3), a_load(3), b_relativeAngle, c_relativeAngle];
    %retval{i + 1} = [a_relativeAngle, a_moment, newPos_A, newPos_B, newPos_C];
  end
  %disp("Finished simulating")
  return;
end