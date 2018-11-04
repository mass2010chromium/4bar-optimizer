format long g
clear all

A=[0,0,0] % Pivot 1
B=[4.21308699, 4.95864331, 0] % Pivot 2
C=[5.56237114, 10.01912241, 0] % Pivot 3
Ax = [-2.7347, 2.9191, 0] % Spreadsheet input
%B = [4.2131, 4.9586, 0] % Spreadsheet input
Bx = [4.1652, 7.324, 0] % Spreadsheet input
%C = [5.5624, 10.019, 0] % Spreadsheet input
Cx = [5.2222, 11.27, 0] % Spreadsheet input

loadC = C_load(0) % Type in angles of three elements to check
loadB = B_load(0)
loadA = A_load(0)

linkC = [-0.33559586, 1.25246082, 0]
linkBC = [1.05298519, 3.94741162, 0]
linkB = [-0.03929690, 2.36552830, 0]
linkAB = [6.90304295, 4.39993174, 0]
linkA = [-2.72925287, 2.92423987, 0]
%linkC = Cx - C
%linkB = Bx - B
%linkA = Ax - A
%linkBC = linkC - linkB;
%linkAB = linkB - linkA;

moment_B = loadB + propagateMoment(loadC, linkC, linkBC, linkB)
moment_A = loadA + propagateMoment(moment_B, linkB, linkAB, linkA)

force_A = moment_A(3) / cross(linkA, -linkAB)(3) * norm(linkAB)