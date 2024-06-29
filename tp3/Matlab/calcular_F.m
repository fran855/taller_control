s = tf('s');
T = 0.01;

a = 250;
b = 25;
c = a;

d = 0.8529;
e = 1.046;
f = 57.65;

A = [0 1 0 0; 
     -f -e -c*d -b*d; 
     0 0 0 1; 
     0 0 -c -b];
 
B = [0; a*d; 0; a];

C = [1 0 0 0;
    0 0 1 0];

D = 0;

K = [0.5, 0.001, 0.7, 0.005];
Ad = eye(4) + A * T;
Bd = B;
Cd = C;
Dd = D;
I = eye(4);
F1 = pinv(C*pinv(I-(A-B*K))*B);
F2 = pinv(Cd*pinv(I-(Ad-Bd*K))*Bd);