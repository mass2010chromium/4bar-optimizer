% Resolves a linkage after disturbing it by changing angles. Solves for
% link B length by minimizing error in connecting link (C) length. 
% Returns -1 for no solution.
% 
% pos1, pos2: Driver link initial and final end points.
% B_pos: Driven link pivot.
% B_len_old: Old link B length.
% B_min, B_max: Max and min length of B accepted.
% deltaThetaB: link B rotation angle.
% newThetaB: Angle from B initial position to horizontal.
% initial_step: Starting radius step size.
% tolerance: Connecting link error tolerance.
function retval = linkageResolveAnalytic(pos1, pos2, B_pos, B_len_old, ...
    B_min, B_max, deltaThetaB, newThetaB, initial_step, tolerance) 
  
  B_len = B_len_old;
  step = initial_step;
  
  newB_iDir = [thetaR2D(newThetaB, 1), 0];
  newB_fDir = [thetaR2D(newThetaB + deltaThetaB, 1), 0];
  
  iter = 0;
  
  min_error = 99;
  min_i = 10000;
  while (iter < 25)
    %disp("Iteration!")
    for i=-5:5
      % Step along B link length.
      tmp_len = B_len + step * i;
      newB_i = newB_iDir * tmp_len;
      newB_f = newB_fDir * tmp_len;
      C_i = B_pos + newB_i - pos1;
      C_f = B_pos + newB_f - pos2;
      len_error = abs(norm(C_i) - norm(C_f));
      
      % Checking for linkage going over center
      tmpArr1 = cross(newB_i, C_i);
      tmpArr2 = cross(newB_f, C_f);
      if (tmpArr1 (3) > 0)
        len_error = 99;
      end
      if (tmpArr2(3) > 0)
          len_error = 99;
      end
      
      %disp(i);
      %disp(len_error);
      
      % If error in length of link C is smaller than prev. error, take note
      if ((len_error < min_error) && ~(B_len > B_max || B_len < B_min))
        min_i = i;
        min_error = len_error;
      end
    end
    %disp("Finished Map!")
    %disp(min_error)
    %disp(min_i)
    B_len = B_len + step * min_i;
    min_i = 0;
    %disp(B_len)
    if (min_error < tolerance)  
      %disp("driver pos1")
      %disp(pos1)
      %disp("driven Angle")
      %disp(newThetaB)
      %disp("driven length")
      %disp(B_len)
      
      
      % We're done!
      retval = B_len;
      return;
    end
    if (abs(min_i) ~= 5 && abs(min_i) ~= 10000)
      %disp("Shrinking Step")
      step = step / 10;
      %disp(step);
    end
    iter = iter + 1;
  end
  %disp("Cannot converge");
  %disp(min_error);
  %disp(min_i);
  %disp(step);
  retval = -1;
  return;
end
