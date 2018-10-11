
close all
clear all
clc

%% 
% Shakeeb Ahmad
% ECE 595 - Spacecraft Navigation and Controls
% HW 2 - Problem 5

%%

global torque_ON;
torque_ON = 1;

%Simulate Nonlinear Model

tspan = 0:1:120*60;
x0 = [1;0;0;5;0;0];
[t,y] = ode45(@EOM_Att, tspan, x0);

%plot(t,y);
plot(t,y(:,4:6));

figure

%Simulate Linearized Model

tspan = 0:1:120*60;
x0 = [1;0;0;5;0;0];
[t,y] = ode45(@EOM_Att_lin, tspan, x0); 

%plot(t,y);
plot(t,y(:,4:6));
%%
% Functions That Reflects Kinematics and Dynamics of the Spacecraft System

function dxdt = EOM_Att(t,x) % Nonlinear

global torque_ON;

J = diag([100 150 300]);

w = x(4:6,1);

if torque_ON
tau = 0.01 * w / norm(w);
else
tau = [0;0;0];
end

dxdt(1:3,1) = x(4:6,1);
dxdt(4:6,1) = J \ (cross(-w,J*w) + tau);

end

function dwdt = EOM_Att_lin(t,x) % Linear

global torque_ON;

J1 = 100; J2 = 150; J3 = 300;
I1 = ( J2 - J3 ) / J1;
I2 = ( J3 - J1 ) / J2;
I3 = ( J1 - J2 ) / J3;

w = x(4:6,1);

if torque_ON
tau = 0.01 * w / norm(w);
else
tau = [0;0;0];
end

A = [eye(3); 0 I1*w(3) I1*w(2); I2*w(3) I1*w(1) 0; I3*w(2) I3*w(1) 0];
A = [zeros(6,3) A];
B = [zeros(3,3); 1/J1 0 0; 0 1/J2 0; 0 0 1/J3];

dwdt = A*x + B*tau;

end