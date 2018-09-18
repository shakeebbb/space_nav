clc
clear all

%% 
% Shakeeb Ahmad
% ECE 595 - Spacecraft Navigation Class

% Problem 4 (B)

%%
F = [0; 0; 0];

odeoptions=odeset('RelTol', 1e-10, 'AbsTol',1e-12);
Ro=42000000;
G=6.6742e-11;  % Universial gravitational constant [N m^2 / kg^2]
M = 5.972e24; % Mass of the Earth
m = 100; % Mass of the Spacecraft
mu = G * (M + m); % 
[t, y] = ode45(@(t,y) eom_fun(t,y,F), [0 1*60*60], [Ro; 0; 0;0; sqrt(mu/Ro); 0], odeoptions);

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