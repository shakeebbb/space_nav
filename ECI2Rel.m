%% Description
% Function to convert from Earth Centered Fixed (ECI) to Relative frame of
% reference

% Called from transformations.m

%% 
% Shakeeb Ahmad
% ECE 595 - Spacecraft Navigation Class

% HW1 - Problem 4 (C)

%%

function [rM, vM] = ECI2Rel(rG, vG, G, M) % Input r and r dot column vectors in ECI Frame

i = rG ./ norm(rG); % i unit vector
k = cross(rG, vG) ./ norm(cross(rG, vG)); % j unit vector
j = cross(k,i);

theta = [i'; j'; k']; % Matrix of the basis vectors

rM = theta * rG - [norm(rG); 0; 0]; % Output r vector in Relative Frame

mu = G * M;
n = sqrt(mu/norm(rG));
vM = theta * vG - [-n*rM(2); sqrt(mu/norm(rG)) + n* rM(1); 0]; % Output r dot vector in Relative Frame

end
