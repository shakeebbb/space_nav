%% Description

%This code will coumpute the drag force per unit mass of the spacecraft

%% Inputs


% ECI_Pos:          ECI Position [ m ]
% ECI_Vec:          ECI Velocity [ m/sec ]
%CD:                Coefficient of Drag
%A:                 Area of the spacraft exposed [ m^2 ]
%mass:              Mass of the spacecraft [kg]

%%

%F:                 Drag acceleration perturbation [ m/sec^2 ]


%% History 

%12Sept2018: Initial Rollout 

%% Development

%Developed by: Christopher Petersen
%Date: 1Sept2018


function [Fm]=Drag_Force(ECI_Pos, ECI_vel, CD, A, mass)


%% Constants
rE = 6371*1000;

%% Calcualte density
r= (ECI_Pos'*ECI_Pos)^(.5);
h=r-rE;
[rho0, expon]=density(h);
rho=rho0*expon;

%% Calculate Force 

nE=7.2921159e-5;        %Rotation of the Earth
d_ra=ECI_vel+ nE*[ECI_Pos(2)  -ECI_Pos(1) 0]';
n_v= ( d_ra'*d_ra )^( .5 );

Fm= -0.5 * CD * ( A/mass ) * rho * n_v *d_ra ; 






end









%% Density Value Computation File

%Created By:    Christopher Petersen

%Updated:       6/12/13

%% Notes

%Simple function to compute the density values in the function: rho=rho0*expon    
%Based upon Valllado table in 8.6

function [rho0, expon]=density(h)

h_ell=h/1000;

atm_table= [25 0 1.225 7.249;...
    30 25 3.899*10^-2 6.349;...
    40 30 1.774*10^-2 6.682;...
    50 40 3.972*10^-3 7.554;...
    60 50 1.057*10^-3 8.382;...
    70 60 3.206*10^-4 7.714;...
    80 70 8.770*10^-5 6.549;...
    90 80 1.905*10^-5 5.799;...
    100 90 3.396*10^-6 5.382;...
    110 100 5.297*10^-7 5.877;...
    120 110 9.661*10^-8 7.263;...
    130 120 2.438*10^-8 9.473;...
    140 130 8.484*10^-9 12.636;...
    150 140 3.845*10^-9 16.149;...
    180 150 2.070*10^-9 22.523;...
    200 180 5.464*10^-10 29.740;...
    250 200 2.789*10^-10 37.105;...
    300 250 7.248*10^-11 45.546;...
    350 300 2.418*10^-11 53.628;...
    400 350 9.518*10^-12 53.298;...
    450 400 3.725*10^-12 58.515;...
    500 450 1.585*10^-12 60.828;...
    600 500 6.967*10^-13 63.822;...
    700 600 1.454*10^-13 71.835;...
    800 700 3.614*10^-14 88.667;...
    900 800 1.170*10^-14 124.64;...
    1000 900 5.245*10^-15 181.05;...
    1001 1000 3.019*10^-15 268.00]; 

ind=find(atm_table(:,2)<= h_ell,1, 'last');

rho0=atm_table(ind,3);
expon=exp(-(h_ell-atm_table(ind,2))/atm_table(ind,4));

end
    
    