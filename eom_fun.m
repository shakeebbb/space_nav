%% Description
% Function to define the orbital dynamics of the
% two-body problem along with external perturbation forces.

% Called from eom.m

%% 
% Shakeeb Ahmad
% ECE 595 - Spacecraft Navigation Class
% HW1 - Problem 4 (A) and (F)

% Problem 4 (A) if useExternalForce = 0
% Problem 4 (B) if useExternalForce = 1

% When the external force in the velocity direction is applied
% (useExternalForce = 1), the spacecraft's orbit changes.

%% Function Definition

function dydt = eom_fun(t,y,useExternalForce)

G = 6.6742e-11;  % Universial gravitational constant [N m^2 / kg^2]
M = 5.972e24; % Mass of the Earth
m = 100; % Mass of the Spacecraft
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
if useExternalForce == 1
F = v/norm(v); % Force of 1 N in the direction of velocity
else
F = [0;0;0]; % External Perturbation Forces
end
dydt(4:6, 1) = -mu * r / (r' * r)^(3/2) + F/m;

end