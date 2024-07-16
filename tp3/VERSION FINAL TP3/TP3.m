%%
close all
clc
clear all

s = tf('s');
T = 0.01;

a = 250;
b = 25;
c = a;

d = 0.8529;
e = 1.046;
f = 57.65;

Gservo = tf(a, [1 b c]);
Gpendulo = tf(d, [1 e f]);
Gtotal = Gpendulo * Gservo;

A = [0 1 0 0; 
     -f -e -c*d -b*d; 
     0 0 0 1; 
     0 0 -c -b];
 
B = [0; a*d; 0; a];

C = [1 0 0 0;
    0 0 1 0];

D = 0;

H = C*inv(s*eye(4)-A)*B + D;

Ad = eye(4) + A * T;
Bd = B * T;
Cd = C;
Dd = D;

autovalores = eig(Ad);
polos_L = autovalores/4;

L = place(Ad', Cd', [0.15 0.15 0.20 0.20])';

K = [0.5 0.001 0.45 0.005];
F = pinv(C*inv(eye(4)-(A-B*K))*B);

K2 = place(Ad, Bd, [0.92 0.93 0.97 0.98]);
disp(K2);