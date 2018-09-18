%% Description
% Function to convert to Earth Centered Fixed (ECI) from Relative frame of
% reference

% Called from transformations.m

%% 
% Shakeeb Ahmad
% ECE 595 - Spacecraft Navigation Class

% HW1 - Problem 4 (C)

%%

function [rG, vG] = Rel2ECI(rM, vM, G, M, t) % Input r and r dot column vectors in Relative Frame

mu = G * M;
n = sqrt(mu/norm(rM));

theta_bar = [cos(n*t) -sin(n*t) 0; sin(n*t) cos(n*t) 0; 0 0 1]; % Transformation Matrix
ix = [1;0;0];
iy = [0;1;0];
iz = [0;0;1];

theta = [(theta_bar*ix)'; (theta_bar*iy)'; (theta_bar*iz)'];

rG = ( rM + [norm(rM); 0; 0]) \ theta; % Output r vector in ECI Frame

vG = ( vM + [-n*rG(2); sqrt(mu/norm(rM)) + n* rG(1); 0] ) \ theta; % Output r dot vector in Relative Frame

end