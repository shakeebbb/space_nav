clc
clear all

%% 
% Shakeeb Ahmad
% ECE 595 - Spacecraft Navigation Class
% Instructor - Dr. Christopher Petersen

% HW 2 - Problem 4
% Code to check stability of the linearized attitude dynamics

%% Eigen Value Analysis

syms I1 I2 I3 a b

A = [eye(3) zeros(3);
     0 I1*b I1*a 0 0 0;
     I2*b 0 0 0 0 0;
     I3*a 0 0 0 0 0;
     ]
 
 eig(A)
