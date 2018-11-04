format long g
clear all

A_dTheta = deg2rad(32.07947464)       % INPUT %
A_pos = [0, 0, 0]                     % INPUT %
A_initial =  deg2rad(140)    % INPUT %
A_len = 4                             % INPUT %


B_dTheta = deg2rad(51.99132797)       % INPUT %
B_pos = [4.21308699, 4.95864331, 0]   % INPUT %
B_minR = 2                            % INPUT %
B_maxR = 3                            % INPUT %
B_initial = deg2rad(115)     % INPUT %
B_initialR = 2.27642922               % INPUT %
B_len = B_initialR;


C_dTheta = deg2rad(76.77000000)       % INPUT %
C_pos = [5.56237114, 10.01912241, 0]  % INPUT %
C_minR = 1.5                          % INPUT %
C_maxR = 2.5                          % INPUT %
C_initial = deg2rad(110)     % INPUT %
C_initialR = 1.56309571               % INPUT %
C_len = C_initialR;

A_loadFunc = @A_load;
B_loadFunc = @B_load;
C_loadFunc = @C_load;

% ------------------------------------------------- %
% 
% Converted parameters
% None atm :(
% 
% ------------------------------------------------- %
% 
% Solver Parameters
% 
lStep_i = 0.05              % Length step %
aStep_i = deg2rad(5)     % Angle step %
eval_aSteps = 40            % Force Evaluation angle step count %
l_tol = 0.00001             % Length tolerance when resolving linkage %

A_curAngle = A_initial;
B_curAngle = B_initial;
C_curAngle = C_initial;

iter_limit = 2

x = zeros(1000);
y = zeros(1000);
z = zeros(1000);
h = zeros(1000);

aStep = aStep_i;
opt_val = 99;
iter = 1;
depth = 0;
cache = containers.Map('KeyType','char','ValueType','double')

n = 1;

minForceData = [];
minAngleData = [];
minMomentData = [];

while (true)
  lastOptVal = opt_val;
  opt_angleA = A_curAngle;
  opt_angleB = B_curAngle;
  opt_angleC = C_curAngle;
  opt_B_len = B_len;
  opt_C_len = C_len;
  for A_index=-iter_limit:iter_limit
    for B_index=-iter_limit:iter_limit
      C_index = 0;
      %for C_index=-iter_limit:iter_limit
        
        angleA1 = A_curAngle + A_index * aStep;
        angleA2 = angleA1 + A_dTheta;
        
        angleB1 = B_curAngle + B_index * aStep;
        angleB2 = angleB1 + B_dTheta;
        
        angleC1 = C_curAngle + C_index * aStep;
        angleC2 = angleC1 + C_dTheta;
        
        key = mat2str([angleA1, angleB1, angleC1]);
        
        if (isKey(cache, key))
          val = cache(key);
        else
          A1 = A_pos + [thetaR2D(angleA1, A_len), 0];
          A2 = A_pos + [thetaR2D(angleA2, A_len), 0];
          
          tmp_B_len = linkageResolveAnalytic(A1, A2, B_pos, B_len, ...
                B_minR, B_maxR, B_dTheta, angleB1, lStep_i, l_tol);
          if (tmp_B_len ~= -1)
            B1 = B_pos + [thetaR2D(angleB1, tmp_B_len), 0];
            B2 = B_pos + [thetaR2D(angleB2, tmp_B_len), 0];
            
            
            tmp_C_len = linkageResolveAnalytic(B1, B2, C_pos, C_len, ...
                  C_minR, C_maxR, C_dTheta, angleC1, lStep_i, l_tol);
            if (tmp_C_len ~= -1)
                endC1 = C_pos + [thetaR2D(angleC1, tmp_C_len), 0];
              C2 = C_pos + [thetaR2D(angleC2, tmp_C_len), 0];
              
              results = simLinkage(eval_aSteps, ...
                    A_pos, A_len, angleA1, A_dTheta, A_loadFunc, ...
                    B_pos, B_len, angleB1, B_loadFunc, ...
                    C_pos, C_len, angleC1, C_loadFunc);
              
              force_data = cell2mat(results(:,1));
              angle_data = cell2mat(results(:,2));
              moment_data = cell2mat(results(:,3));
              
              %force_data = results{2,:};
              
              %disp(force_data);
              %force_data = results;
              val = max(force_data) - min(force_data); % Change to whatever function
            else
              val = -1;
            end
          else
            val = -1;
          end
        end
        
        x(n) = angleA1;
        y(n) = angleB1;
        z(n) = angleC1;
        h(n) = val;
        n = n + 1;
        
        cache(key) = val;
        
        if (val ~= -1 && val < opt_val) 
          minForceData = force_data;
          minAngleData = angle_data;
          minMomentData = moment_data;
          opt_val = val;
          opt_angleA = angleA1;
          opt_angleB = angleB1;
          opt_angleC = angleC1;
          opt_B_len = tmp_B_len;
          opt_C_len = tmp_C_len;
        end
        
        %if (A_index == B_index && B_index == C_index && C_index == 0)
        %  disp("writing csv at 0, 0, 0")
        %  csvwrite("data.csv",cell2mat(results'))
        %end
      %end
    end
  end
  
  A_curAngle = opt_angleA;
  B_curAngle = opt_angleB;
  C_curAngle = opt_angleC;
  
  B_len = opt_B_len;
  C_len = opt_C_len;
  
  % Debug break
  % break;
  
  if (iter > 10)
    disp("Failed to converge!");
    break;
  end
  if (lastOptVal == opt_val)
    if (depth >= 2)
      disp("Hit depth limit! Finished iteration.")
      break;
    end
    disp("Increasing search resolution");
    disp("step=");
    disp(aStep);
    iter = 1;
    depth = depth + 1;
    aStep = aStep / (iter_limit);
  end
  
  
  disp("Finished Iteration");
  disp(iter);
  disp("Opt value:");
  disp(opt_val);
  disp("A angle:");
  disp(A_curAngle);
  disp("B angle:");
  disp(B_curAngle);
  disp("C angle:");
  disp(C_curAngle);
  disp("--------");
  iter = iter + 1;
end

disp("Final Results");
disp("------------");
disp("A angle:");
disp(rad2deg(A_curAngle));
disp("B angle:");
disp(rad2deg(B_curAngle));
disp("C angle:");
disp(rad2deg(C_curAngle));
disp("A length:");
disp(A_len);
disp("B length:");
disp(B_len);
disp("C length:");
disp(C_len);

forceAngleData = {minForceData, minAngleData, minMomentData};
csvwrite("forceVsAngle.csv", cell2mat(forceAngleData));

%results = {x; y; z; h};
%csvwrite("data.csv", cell2mat(results));
x=x(1:n-1);
y=y(1:n-1);
z=z(1:n-1);
h=h(1:n-1);
scatter3(x, y, z, [], h);

%A1 = A + [thetaR2D(driver_initial, driver_R), 0];
%A2 = A + [thetaR2D(driver_initial + driver_dTheta, driver_R), 0];
%
%B_len = driven1_initialR;
%
%B_len = linkageResolveAnalytic(A1, A2, B, B_len, driven1_minR, driven1_maxR, driven1_dTheta, driven1_initial, lStep_i, l_tol)
%
%B1 = B + [thetaR2D(driven1_initial, B_len), 0];
%B2 = B + [thetaR2D(driven1_initial + driven1_dTheta, B_len), 0];
%
%C_len = driven2_initialR;
%
%C_len = linkageResolveAnalytic(B1, B2, C, C_len, driven2_minR, driven2_maxR, driven2_dTheta, driven2_initial, lStep_i, l_tol)
