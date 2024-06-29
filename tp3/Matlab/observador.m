a = 250;
b = 25;
c = 250;
d = 0.8529;
e = 1.046;
f = 57.65;
T = 0.01;

A = [0 1 0 0; -f -e -c*d -b*d; 0 0 0 1; 0 0 -c -b];
B = [0; a*d; 0; a];
C = [1 0 0 0; 0 0 1 0];
D = 0;

Ad = eye(4) + A * T;
Bd = B * T;
Cd = C;
Dd = D;

avas = real(eig(Ad));

L = place(Ad', Cd', [0.15 0.15 0.05 0.05])';
L

