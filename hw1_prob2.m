clc
clear all

[t, y] = ode45(@diffeq,[0 20],[10; 1]);

plot(t,y);