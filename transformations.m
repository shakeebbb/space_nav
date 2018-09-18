clc
clear all

%% 
% Shakeeb Ahmad
% ECE 595 - Spacecraft Navigation Class

% HW 1 - Problem 4 (B)

%%

rG = [40000 12800 129]';
vG = [0.032 3.090 0]';

rM = ECI2Rel(rG,vG);