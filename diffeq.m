function dydt = diffeq(t,y)

mu = 0.012;
n = 2;
h = 1;

dydt = [y(2);(h^2/y(1)^3 - mu/y(1)^n)];