clc
clear all

syms n

disp('State Matrix of the Given Dynamics');

A = [zeros(3) eye(3);
     3*n^2 0 0 0 2*n 0;
     0 0 0 -2*n 0 0;
     0 0 -n^2 0 0 0]
 
disp('State Matrix Eigen Values');

eig(A)

disp('Let mu = 0.012 and n = 100');

mu = 0.012;
n = 100;
m = 100;

A = [zeros(3) eye(3);
     3*n^2 0 0 0 2*n 0;
     0 0 0 -2*n 0 0;
     0 0 -n^2 0 0 0];
 
B_0 = [0 0 0; 0 0 0; 0 0 0; 1/m 0 0; 0 1/m 0; 0 0 1/m]
B_1 = [0 0; 0 0; 0 0; 1/m 0; 0 0; 0 1/m]
B_2 = [0 0; 0 0; 0 0; 0 0; 1/m 0; 0 1/m]
B_3 = [0 0; 0 0; 0 0; 1/m 0; 0 1/m; 0 0]

disp('For B_0, the rank of the controllability matrix is,');

rank(ctrb(A,B_0))

disp('For B_1, the rank of the controllability matrix is,');

rank(ctrb(A,B_1))

disp('For B_2, the rank of the ontrollability matrix is,');

rank(ctrb(A,B_2))

disp('For B_3, the rank of the controllability matrix is,');

rank(ctrb(A,B_3))

