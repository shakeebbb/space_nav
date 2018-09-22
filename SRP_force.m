%% Description

%This code will coumpute the SRP force per unit mass of the spacecraft.
%Assumes that at t=0, the sun is aligned with the X-ECI direction.

%% Inputs


% ECI_Pos:          ECI Position [ m ]
% ECI_Vec:          ECI Velocity [ m/sec ]
%A:                 Area of the spacraft exposed [ m^2 ]
%mass:              Mass of the spacecraft [kg]

%%

%F:                 SRP acceleration perturbation [ m/sec^2 ]


%% History 

%12Sept2018: Initial Rollout 

%% Development

%Developed by: Christopher Petersen
%Date: 1Sept2018


function [Fm]=SRP_force(ECI_Pos, ECI_vel, A, mass,t)


%% Constants
fconst=-4.5e-2;                  
is=23.4349*pi/180;    
r_rate_sun=2*pi/( 365*24*60*60 );


%%
f=fconst*A/(mass);
As=r_rate_sun*t;

Fm= [ f*cos(As) ; f*cos(is)*sin(As) ; f*sin(is)*cos(As) ];





end










    
    