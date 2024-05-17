l = 0.16;
r2 = 2.625;
g = 9.8;
T = 0.01;

Ad = eye(2) + [0 1; -g/l -r2] * T;
Cd = [1 0];
L = place(Ad', Cd', [0.5 0.6]);


