clc
clear all

%% 
% Shakeeb Ahmad
% ECE 595 - Spacecraft Navigation Class
% Instructor - Dr. Christopher Petersen

% HW 2 - Problem 1 (B)
% Code to plot orbital elements in the presence of the solar radiation
% pressure (SRP)

%% Constants

G=6.6742e-11;   %Universial gravitational constant [N m^2 / kg^2]
M=5.972e24;    %Mass of Earth [ kg ]
mu=G*M;        %Earth's gravitational constant        

%% Spacecraft Initial Conditions

%r=42000000;  %parameter to make coming up with initial conditions
r=(6371+3500)*1000;

sc_pos = [r 0 0];            %Initial spacecraft position in ECI [ m ]
sc_vel = [0 sqrt(mu/r) 0];   %Initial spacecraft velocity in ECI [ m/sec ]s
sc_X=[sc_pos sc_vel]';

m=100;                           %Mass of the spacecraft [ kg ]

thruster_mag=1;                     %Thruster magnitude [ N ]

CD=2;                             %Coefficient of drag
A=1.5;                             %Area for drag [ m^2 ]

%% Simulation Conditions

t_final = 7*60*60;          %Total time of the simulation
t_inc = 60;                  %Time increment of data


%% 2 Body Problem with Solar Radiation Pressure

odeoptions=odeset('RelTol', 1e-10, 'AbsTol',1e-12);
tspan=[0:t_inc:t_final];

[t, y] = ode45(@(t,y) force_fun(t,y,G,M,m,A,CD,0), tspan, sc_X, odeoptions);

plot3(y(:,1),y(:,2),y(:,3));
title('With SRP');
figure

%% 2 Body Problem with Drag Force

odeoptions=odeset('RelTol', 1e-10, 'AbsTol',1e-12);
tspan=[0:t_inc:t_final];

[t, y] = ode45(@(t,y) force_fun(t,y,G,M,m,A,CD,1), tspan, sc_X, odeoptions);

plot3(y(:,1),y(:,2),y(:,3));
title('With Drag Force')