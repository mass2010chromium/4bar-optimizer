format long g
clear all

A=[0,0,0] % Pivot 1
B=[4.21308699, 4.95864331, 0] % Pivot 2
C=[5.56237114, 10.01912241, 0] % Pivot 3
R1=3.16680213
R2=1.92342793
R3=1.25583860

%initial_A=unit([-1.93744561,0.17654158,0]) %Starting position of arm A
%initial_B=unit([-0.95764908,0.36851247,0]) %Starting position of arm B
%initial_C=unit([-0.65130335,-0.00875342,0]) %Starting position of arm C
initial_A = unit([thetaR2D(deg2rad(130), R1), 0])
initial_B = unit([thetaR2D(deg2rad(110), R2), 0])
initial_C = unit([thetaR2D(deg2rad(120), R3), 0])
%initial_A = unit([thetaR2D(1.244943355447555, R1), 0])
%initial_B = unit([thetaR2D(1.469453691798994, R2), 0])
%initial_C = unit([thetaR2D(1.928104531761819, R3), 0])

angle=deg2rad(32.07947464)

steps=300

step = angle / steps

load_B=[0,0,1] %Moment at B
load_C=[0,0,1] %Moment at C

arm_A=R1*initial_A;
arm_B=R2*initial_B;
arm_C=R3*initial_C;

arm_AB=(B+arm_B) - (A+arm_A);
AB=norm(arm_AB)
arm_BC=(C+arm_C) - (B+arm_B);
BC=norm(arm_BC)

result=cell(steps + 1, 1);
%result{1} = ["initialA=",initial_A,"","",""];
%result{2} = ["initialB=",initial_B,"","",""];
%result{3} = ["thetaOffset_A", "Ax","Ay","Az","Bx","By","Bz"];

a_vector=arm_A;
b_vector=arm_B;
c_vector=arm_C;

lastX_B=B+arm_B %Previous B-AB intersection point
lastX_C=C+arm_C %Previous C-BC intersection point

result{1}=[0, [0,0,0], A, B, C, [0, 0, 0], [0,0,0]];
%m_b = propagateMoment(C_load(0), c_vector, lastX_B - lastX_C, b_vector);
%m_a = propagateMoment(B_load(0) + m_b, b_vector, arm_A + A - lastX_B, a_vector) + A_load(0);
%result{2}=[0, m_a * AB / norm(cross(a_vector, arm_A + A - lastX_B)), A + arm_A, lastX_B, lastX_C];
%result{2}=[0, m_a, A + arm_A, lastX_B, lastX_C];

for i = 0:steps
  theta = i * step;
  a_vector=rotateVector(a_vector,[0,0,1],step);
  %points=circleIntersection(  );
  %x1=points{1};
  %x2=points{2};
  %delta1=norm(x1-lastX);
  %delta2=norm(x2-lastX);
  newPos_B = driveLinkage(lastX_B, A+a_vector,AB, B,R2);
  lastX_B = newPos_B;
  newPos_C = driveLinkage(lastX_C, newPos_B,BC, C,R3);
  lastX_C = newPos_C;
  %if (delta1 < delta2)
  %  b_vector=x1-B;
  %  lastX=x1;
  %else
  %  b_vector=x2-B;
  %  lastX=x2;
  %endif
  b_vector = newPos_B - B;
  c_vector = newPos_C - C;
  
  ab_vector=-(A+a_vector) + (newPos_B);
  bc_vector=(newPos_C) - (newPos_B);
  
  c_angle = vectorAngle(c_vector, initial_C);
  b_angle = vectorAngle(b_vector, initial_B);
  a_angle = theta;
  c_load = C_load(c_angle);
  m_c = c_load;
  m_b = B_load(b_angle) + propagateMoment(m_c, c_vector, bc_vector, b_vector);
  m_a = A_load(theta) + propagateMoment(m_b,b_vector, ab_vector, a_vector);
  
  f_linkA = m_a * AB / norm(cross(a_vector, ab_vector));
  
  %result{i + 2}=[i*step, m_a, A+a_vector, newPos_B, newPos_C];
  result{i + 2}=[i*step, f_linkA, A+a_vector, newPos_B, newPos_C, m_c(3), m_b(3), m_a(3), c_angle, b_angle, a_angle];
end

csvwrite("data.csv",cell2mat(result'))
