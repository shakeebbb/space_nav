clc
clear all

%% 
% Shakeeb Ahmad
% ECE 595 - Spacecraft Navigation Class

% HW 1 - Problem 4 (B)

%%

G=6.6742e-11;  % Universial gravitational constant [N m^2 / kg^2]
M = 5.9717e+17; % Mass of the Planet

rG = [40000 12800 129]'; % deputy position vector in ECI Frame
vG = [0.032 3.090 0]'; % deputy velocity vector in ECI Frame

[rM, vM] = ECI2Rel(rG,vG,G,M)

[rG, vG] = Rel2ECI(rM,vM,G,M,0)