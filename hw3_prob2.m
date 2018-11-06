clc
clear all
close all

%% Shakeeb Ahmad
% Homework 3 Problem 2
% Relative Motion
% Fall 2018
% ECE 595 - Spacecraft Controls and Navigation

%%

G=6.6742e-11;   %Universial gravitational constant [N m^2 / kg^2]
mE=5.972e24;    %Mass of Earth [ kg ]
mu=G*mE;        %Earth's gravitational constant  
R = 6371e3; % Earth's radius

m = 500;

n = sqrt(mu/(R + 530e3)^3)

t_1 = 30*60;
x_init = [-70 0 0 0.1094 0 0.2651]';
r_0 = x_init(1:3);
v_0 = x_init(4:6);

phi_rr = [4-3*cos(n*t_1)     0 0; 
          6*(sin(n*t_1)-n*t_1) 1 0; 
          0                0 cos(n*t_1)]
phi_rv = [1/n*sin(n*t_1)     2/n*(1-cos(n*t_1))         0;
          2/n*(cos(n*t_1)-1) 1/n*(4*sin(n*t_1)-3*n*t_1)   0;
          0                 0                       1/n*sin(n*t_1)]
phi_vr = [3*n*sin(n*t_1)     0 0;
          6*n*(cos(n*t_1)-1) 0 0;
          0                0 -n*sin(n*t_1)]
phi_vv = [cos(n*t_1)      2*sin(n*t_1)      0;
          -2*sin(n*t_1)   4*cos(n*t_1)-3    0;
          0           0             cos(n*t_1)]
      
dV_0 = inv(phi_rv)*(-phi_rr*r_0 )- v_0
dV_f = -phi_vr*r_0 - phi_vv*(v_0 + dV_0)


x_init = x_init + [0;0;0;dV_0]

A = [zeros(3) eye(3);
    3*n^2 0 0 0 2*n 0;
    0 0 0 -2*n 0 0;
    0 0 -n^2 0 0 0];
B = [0 0 0; 0 0 0; 0 0 0; 1/m 0 0; 0 1/m 0; 0 0 1/m];

%% Solve for trajectory for a two impulsive manuever

t_span = [0 t_1];
[t1,x1] = ode45(@(t,x) relativeDynamics(t,x,A,B,zeros(3,6)), t_span, x_init);

x_init = x1(end,1:6)' + [0;0;0;dV_f];
[t2,x2] = ode45(@(t,x) relativeDynamics(t,x,A,B,zeros(3,6)), t_span, x_init);

figure 
plot([t1; t1(end)+t2],[x1(:,1:3);x2(:,1:3)] );
title('Relative Motion Trajectory for a two impulsive manuever');
xlabel('t(s)');
ylabel('states');
lgd = legend('x','y','z');
grid on

%% Designing LQR Controller
Q = eye(6);
R = eye(3)*10000;

K = lqr(A,B,Q,R);

x_init = [-70 0 0 0.1094 0 0.2651];
t_span = [0 40*60];
[t,x] = ode45(@(t,x) relativeDynamics(t,x,A,B,K), t_span, x_init);

figure
plot(t./60,x);
title('LQR based manuever (To converge in 30 min)');
xlabel('t(min)');
ylabel('states');
lgd = legend('x','y','z');
grid on

%% Plotting both the trajectories on the same plot
figure
plot3([x1(:,1);x2(:,1)],[x1(:,2);x2(:,2)],[x1(:,3);x2(:,3)] );
hold on
plot3(x(:,1),x(:,2),x(:,3));
hold off

title('Relative Motion Trajectories');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
lgd = legend('Two impulsive', 'LQR Based');
grid on

%% Function Descriptions

function dxdt = relativeDynamics(t,x,A,B,K)

dxdt = (A - B*K)*x;
%dxdt = A*x; %+B*U;
end

