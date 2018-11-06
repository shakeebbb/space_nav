clc
clear all
close all

%% Shakeeb Ahmad
% Homework 3 Problem 3
% Attitude Control
% Fall 2018
% ECE 595 - Spacecraft Controls and Navigation

J1 = 10;
J2 = 15;
J3 = 20;

A = [zeros(3) eye(3); zeros(3,6)];
B = [zeros(3); diag([1/J1 1/J2 1/J3])];

Q = eye(6);
R = eye(3)*1e7;

K = lqr(A,B,Q,R);
t_span = [0 100];
x_init = [0.1 -2.4 1.2 0 0 0]';
%x_init = [30 20 -10 0 0 0]'*pi/180;

%% LQR to Linear Dynamics (Q = eye(6), R = eye(3)*1e7)

[t,x] = ode45(@(t,x) linearLQR(t,x,A,B,K), t_span, x_init);

plot(t,x);
title('LQR to Linear Dynamics (Q = eye(6), R = eye(3)*1e7)');
xlabel('t(s)');
ylabel('states');
lgd = legend('\phi','\theta','\psi','\omega_1','\omega_2','\omega_3');
grid on

%% LQR to Non-Linear Dynamics (Q = eye(6), R = eye(3)*1e7)

[t,x] = ode45(@(t,x) nonlinearLQR(t,x,J1,J2,J3,K), t_span, x_init);

figure
plot(t,x);
title('LQR to Non-Linear Dynamics (Q = eye(6), R = eye(3)*1e7)');
xlabel('t(s)');
ylabel('states');
lgd = legend('\phi','\theta','\psi','\omega_1','\omega_2','\omega_3');
grid on


%% LQR to Non-Linear Dynamics (Q = eye(6), R = eye(3)*1e-3)
Q = eye(6);
R = eye(3)*1e-3;

K = lqr(A,B,Q,R);

[t,x] = ode45(@(t,x) nonlinearLQR(t,x,J1,J2,J3,K), t_span, x_init);

figure
plot(t,x);
title('LQR to Non-Linear Dynamics (Q = eye(6), R = eye(3)*1e-3)');
xlabel('t(s)');
ylabel('states');
lgd = legend('\phi','\theta','\psi','\omega_1','\omega_2','\omega_3');
grid on
%% DCM Calculations

O_B0 = rotx(x_init(1)*180/pi)*roty(x_init(2)*180/pi)*rotz(x_init(3)*180/pi);
O_Bf = rotz(0*180/pi)*rotx(0*180/pi)*roty(0*180/pi);

O_Bf_B0 = O_Bf * O_B0';

theta_f = acos(0.5*(trace(O_Bf_B0) - 1 ));

[V,D] = eig(O_Bf_B0);

e = V(:,1);

T = 100;

%% DCM to Linear Dynamics

[t,x] = ode45(@(t,x) linearDCM(t,x,J1,J2,J3,theta_f,e,T,A,B), t_span, x_init);

figure
plot(t,x);
title('DCM to Linear Dynamics');
xlabel('t(s)');
ylabel('states');
lgd = legend('\phi','\theta','\psi','\omega_1','\omega_2','\omega_3');
grid on

%% DCM to Non-Linear Dynamics

[t,x] = ode45(@(t,x) nonlinearDCM(t,x,J1,J2,J3,theta_f,e,T), t_span, x_init);

figure
plot(t,x);
title('DCM to Non-Linear Dynamics');
xlabel('t(s)');
ylabel('states');
lgd = legend('\phi','\theta','\psi','\omega_1','\omega_2','\omega_3');
grid on

%% Lyapunov Based Controller to Non-Linear Dynamics
K = 2;
x_init = [eul2quat(x_init(1:3)') 0 0 0];
[t,x] = ode45(@(t,x) nonlinearLyapunov(t,x,J1,J2,J3,K), t_span, x_init);

figure
plot(t,x);
title('Lyapunov Based Controller to Non-Linear Dynamics');
xlabel('t(s)');
ylabel('states');
lgd = legend('\phi','\theta','\psi','\omega_1','\omega_2','\omega_3');
grid on

%% Functions Description

function dxdt = linearLQR(t,x,A,B,K)
dxdt = (A - B*K)*x;
end

function dxdt = nonlinearLQR(t,x,J1,J2,J3,K)

J = diag([J1 J2 J3]);
w = x(4:6);
M = -K*x;

dxdt(1:3,1) = w;
dxdt(4:6,1) = inv(J) * (-cross(w,J*w) + M);
end

function dxdt = nonlinearDCM(t,x,J1,J2,J3,theta_f,e,T)

J = diag([J1 J2 J3]);
w = x(4:6);

w_t = (theta_f*pi) / (2*T) * sin(pi*t / T) * e;
w_dot_t = (theta_f*pi*pi) / (2*T*T) * cos(pi*t / T) * e;
S = [0 -w_t(3) w_t(2); w_t(3) 0 -w_t(1); -w_t(2) w_t(1) 0];

M = J*w_dot_t + S*J*w_t;

dxdt(1:3,1) = w;
dxdt(4:6,1) = inv(J) * (-cross(w,J*w) + M);
end

function dxdt = linearDCM(t,x,J1,J2,J3,theta_f,e,T,A,B)

J = diag([J1 J2 J3]);
w = x(4:6);

w_t = (theta_f*pi) / (2*T) * sin(pi*t / T) * e;
w_dot_t = (theta_f*pi*pi) / (2*T*T) * cos(pi*t / T) * e;
S = [0 -w_t(3) w_t(2); w_t(3) 0 -w_t(1); -w_t(2) w_t(1) 0];

M = J*w_dot_t + S*J*w_t;

dxdt = A*x +B*M;
end

function dxdt = nonlinearLyapunov(t,x,J1,J2,J3,K)

J = diag([J1 J2 J3]);
w = x(5:7);
q = x(1:4);

M = -q(1:3)-K*w;
S = [0 -q(3) q(2); q(3) 0 -q(1); -q(2) q(1) 0];

dxdt(1:3,1) = 0.5*(S + q(4)*eye(3))*w;
dxdt(4) = -0.5*q(1:3)'*w;
dxdt(5:7,1) = inv(J) * (-cross(w,J*w) + M);
end