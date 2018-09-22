%% Description
% Function to define the orbital dynamics of the
% two-body problem along with SRP.

% Called from orbital_elements.m

%% 
% Shakeeb Ahmad
% ECE 595 - Spacecraft Navigation Class
% HW2 - Problem 1 (B)

%% Function Definition

function dydt = SRP_fun(t, y, G, M, m, A)

% G : Universial gravitational constant [N m^2 / kg^2]
% M : Mass of the Earth
% m : Mass of the Spacecraft
mu = G * (M + m); % 
%n = 0.012;
%h = 1;

% dydt = [y(4); ...
%         y(5); ...
%         y(6); ...
%         -mu * y(1) / (y(1)^2 + y(2)^2 + y(3)^2)^3/2 + F(1)/m; ...
%         -mu * y(2) / (y(1)^2 + y(2)^2 + y(3)^2)^3/2  + F(2)/m; ...
%         -mu * y(3) / (y(1)^2 + y(2)^2 + y(3)^2)^3/2  + F(3)/m];
    
v = y(4:6);
dydt(1:3, 1) = v;
r = y(1:3);

F = SRP_force(r,v,A,m,t); % External Perturbation Forces

dydt(4:6, 1) = -mu * r / (r' * r)^(3/2) + F/m;

end