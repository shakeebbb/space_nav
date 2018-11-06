clc
clear all
close all

%% ECE 595 Spacecraft Navigation and Controls
% Homework 3 - Fall 2018

%% 

odeoptions=odeset('RelTol', 1e-10, 'AbsTol',1e-12);

G = 6.6742e-11;  % Universial gravitational constant [N m^2 / kg^2]
M = 5.972e24; % Mass of the Earth
m = 100; % Mass of the Spacecraft
mu = G * (M + m); 
useExternalForce = 0; % 0 : To simulate without external force, 1 : To simulate with external force
[t, y] = ode45(@(t,y) eom_fun(t,y,useExternalForce), [0 1200*60*60], [42000000; 0; 0;0; 4000; 0], odeoptions);

plot(t,y(:, 1:3));
title('Trajectory Plot Against Time');
legend({'$x$', '$y$', '$z$'}, 'Interpreter', 'latex', 'FontSize', 15);
xlabel('t (s)');
ylabel('State Vector');

figure

plot3(y(:, 1),y(:, 2),y(:, 3));
title('Path Plot in Eulidean Coordinates');
xlabel('x (km)');
ylabel('y (km)');
zlabel('z (km)');

grid on