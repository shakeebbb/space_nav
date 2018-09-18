%% Description

% This code will prompt the user to input planet states as well as a
% spacecraft state, and at the end will produce a GUI showing the
% spacecraft motion

%% Example Material

% Earth State = [0 0 0 0 0 0 5.972e24 6371*1000];
% Earth Mass = 5.972e24;
% Earth Radius = 6371*1000;

% Spacecraft State = [42000000 0 0 0  3.0806e+03 0 100 0];
% Spacecraft Mass = 100;

% Total Time of Sim = 24*60*60;
% Time Increment 30;
 

%% History 


%% Development

%Developed by: Christopher Petersen
%Date: 2Aug2018

%%

function MultiBody_Final
%% 
clear 
clc
close all

%% Constants

G=6.6742e-11;   %Universial gravitational constant [N m^2 / kg^2]
    

%% Ask for Planet Info

disp('----------------')
disp('Planet Inputs')
disp('----------------')
prompt = 'How Many Planets? ';
N = input(prompt);
Planet_matrix=zeros(N+1,8);
for i=1:N
    s = strcat('      Planet : ', num2str(i));
    disp(s)
    prompt ='          Orbital Pos & Vel [ m, m/sec] : ';
    x = input(prompt);
    while length(x) ~= 6
        disp('        Wrong size vector ')
        prompt ='          Orbital Pos & Vel [ m, m/sec] : ';
        x = input(prompt);
    end
    Planet_matrix(i,1)=x(1);
    Planet_matrix(i,2)=x(2);
    Planet_matrix(i,3)=x(3);
    Planet_matrix(i,4)=x(4);
    Planet_matrix(i,5)=x(5);
    Planet_matrix(i,6)=x(6);
    prompt ='          Planet Mass [ kg ] : ';
    x = input(prompt);
    Planet_matrix(i,7)=x;
    prompt ='          Planet Radius [ m ] : ';
    x = input(prompt);
    Planet_matrix(i,8)=x;
    
end
N=N+1;

%% Ask For Spacecraft IC

disp('----------------')
disp('Spacecraft Inputs')
disp('----------------')

prompt ='     Spacecraft Pos & Vel [ m, m/sec] : ';
x = input(prompt);
while length(x) ~= 6
    disp('     Wrong size vector ')
    prompt ='     Spacecraft Pos & Vel [ m, m/sec] : ';
    x = input(prompt);
end
Planet_matrix(end,1)=x(1);
Planet_matrix(end,2)=x(2);
Planet_matrix(end,3)=x(3);
Planet_matrix(end,4)=x(4);
Planet_matrix(end,5)=x(5);
Planet_matrix(end,6)=x(6);
prompt ='     Spacecraft Mass [ kg ] : ';
x = input(prompt);
Planet_matrix(end,7)=x;

%% Ask For Sim Inputs

disp('----------------')
disp('Sim Inputs')
disp('----------------')
prompt ='     Total Time of Sim [ sec ] : : ';
t_final = input(prompt);
prompt ='     Time Increment [ sec ] : ';
t_inc = input(prompt);



%% Extract Necessary Values

tspan=0:t_inc:t_final;
X_IC=zeros( (N)*6, 1 );
for i=1:N
        X_IC( (i-1)*6+1 : i*6 ) = Planet_matrix( i, 1:6)';   
end
odeoptions=odeset('RelTol', 1e-10, 'AbsTol',1e-12);

%% Simulate ODE

[t_ode, x_ode]=ode45(@(t,x)EOM(t,x,Planet_matrix, N,G), tspan, X_IC, odeoptions); 
sc=x_ode(:,end-5:end);


%% Plot Things

S.fh = figure;
hold on
S.t_ode= t_ode;
S.x_ode= x_ode;
S.sc=x_ode(:,end-5:end);
S.Plot_SCIC = plot3(sc(1,1),sc(1,2), sc(1,3),'go','markersize',12);   
S.Plot_SCFC = plot3(sc(1,1),sc(1,2), sc(1,3),'rx','markersize',12);  
S.N=N;


%Plot Planets
for i=1:N-1
    planet_pos=x_ode( 1, (i-1)*6+1: (i-1)*6+3);
    [x, y, z]=ellipsoid(planet_pos(1), planet_pos(2), planet_pos(3), Planet_matrix(i,8),Planet_matrix(i,8),Planet_matrix(i,8),20);
    colormatrix=zeros(size(x,1), size(x,2), 3);
    for j=1:size(x,1)
        for k=1:size(x,2)
            colormatrix(j,k,:)=[1 0 0];
        end
    end
    S.Planets(i)=surf(x,y,z,colormatrix);
end
S.Planet_matrix=Planet_matrix;

%Plot SC Trajectory
S.scplot=plot3(sc(1,1), sc(1,2), sc(1,3), 'linewidth', 2);

%Other things
S.Legend= legend( [S.Plot_SCIC, S.Plot_SCFC, S.scplot], 'Initial Condition', 'Final Condition', 'Sc Traj');
grid on

%Set Slider
S.sl = uicontrol('style','slide',...
    'position',[0 10 400 20],...
    'min',0,'max',t_final,...
    'callback',{@sl_call,S});

xlabel('X Pos [ m ]')
ylabel('Y Pos [ m ]')
zlabel('Z Pos [ m ]')


end



function Xdot=EOM(t,X,Planet_matrix, N,G)


Xdot=zeros(N*6,1);

i=1;
while i<=N
    
    %% Extract Current Data
    mass=Planet_matrix(i,7);
    r=X( (i-1)*6+1 : (i-1)*6+3 );  
    v=X( (i-1)*6+4 : i*6 );
    
    Xdot( (i-1)*6+1 : (i-1)*6+3 )=v;

    %% Cycle Through Other Objects to Compute Accelerations
    j=1;
    Force=[0;0;0];
    while j<=N
       
        if j~=i
        mass_j=Planet_matrix (j,7);  
        r_j=X( (j-1)*6+1 : (j-1)*6+3 );  
        
        r_rel=r-r_j;
        r_rel_norm=(r_rel'*r_rel)^(3/2);
        Force=Force-G*mass*mass_j*r_rel/r_rel_norm;
             
        end
        
        j=j+1;
    end
    
    Xdot( (i-1)*6+4 : i*6) =Force/mass;
    
    i=i+1; 
end

Xdot=Xdot(:);


end


function [] = sl_call(varargin)
% Callback for the slider.
[h,S] = varargin{[1,3]};  % calling handle and data structure.
slidervalue=get(h,'value');
index_last=find(S.t_ode<=slidervalue);
index_last=index_last(end);

%Set New Spacecraft Trajectory Data
set(S.scplot,'xdata',S.sc(1:index_last,1),'ydata',S.sc(1:index_last,2),'zdata',S.sc(1:index_last,3))
set(S.Plot_SCFC,'xdata',S.sc(index_last,1),'ydata',S.sc(index_last,2),'zdata',S.sc(index_last,3))

%Set New Planet Data
for i=1:S.N-1
    planet_pos=S.x_ode( index_last, (i-1)*6+1: (i-1)*6+3);
    [x, y, z]=ellipsoid(planet_pos(1), planet_pos(2), planet_pos(3), S.Planet_matrix(i,8),S.Planet_matrix(i,8),S.Planet_matrix(i,8),20);
    set(S.Planets(i),'XData',x,'YData',y,'ZData',z);

end




end


