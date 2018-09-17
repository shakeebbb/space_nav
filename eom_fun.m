%% Description
%Function to define the orbital dynamics of the
%two-body problem along with external perturbation forces.

%% Function Definition

function dydt = eom_fun(t,y,F)

G=6.6742e-11;  % Universial gravitational constant [N m^2 / kg^2]
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
dydt(4:6, 1) = -mu * r / (r' * r);

end